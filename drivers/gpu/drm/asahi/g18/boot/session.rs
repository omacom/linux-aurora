// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! Stage10 owner: synchronous DRM boundary, shallow worker stack. A job that a
//! firmware recovery ends fails alone; any other fault is sticky.
use super::*;
#[path = "compute_session.rs"]
mod compute_session;
#[path = "recovery.rs"]
mod recovery;
use recovery::{Failure, JobEnd, JobFailure, JobPath};
use core::{ops::Range,sync::atomic::AtomicI32};
use crate::{g18::uapi::application::{G18ApplicationRender,G18RenderJobSink,G18RenderRetirement},
    g18::drm::G18DrmRuntime,
    g18::runtime::graphics_job::application::ApplicationJob};

pub(crate) struct ApplicationOwner {
    session: Arc<Mutex<Retained>>,
    failed: Arc<AtomicBool>,
    /// [`JobPath::bit`]s of paths a stranded job quarantined.
    quarantined: Arc<AtomicU32>,
}

impl ApplicationOwner {
    fn admit(&self,path:JobPath)->Result {
        if self.quarantined.load(Ordering::Acquire)&path.bit()==0 {return Ok(());}
        if let Some(session)=self.session.lock().0.as_ref() {
            dev_err!(session.device(),"G18_APP_JOB_FAILED error={:?} refused path={:?} quarantined session=live\n",EIO,path);
        }
        Err(EIO)
    }

    /// Run one task on the session worker. The outcome crosses back as one
    /// word: [`TICKET_RUNNING`], 0, a negative errno for a fatal failure, or
    /// a positive [`JobOutcome`] code for a job that failed alone.
    fn work<F>(&self, task: F) -> core::result::Result<(),Failure>
        where F: FnOnce(&mut G18Session)->core::result::Result<(),Failure> + Send + 'static {
        if self.failed.load(Ordering::Acquire) {return Err(Failure::Fatal(EIO));}
        let session=self.session.clone();let failed=self.failed.clone();
        let quarantined=self.quarantined.clone();
        let ticket=Arc::new(AtomicI32::new(TICKET_RUNNING),GFP_KERNEL)?;let done=ticket.clone();
        kernel::workqueue::system_long().try_spawn(GFP_KERNEL,move || {
            done.store(Self::outcome(&session,&failed,&quarantined,task),Ordering::Release);
        })?;
        let waited=Instant::<Monotonic>::now();
        loop {
            match ticket.load(Ordering::Acquire) {
                TICKET_RUNNING => fsleep(Delta::from_micros(recovery::poll_interval_us(waited.elapsed().as_micros_ceil()))),
                code => {
                    if code==0 {recovery::SUBMIT_LATENCY.record(waited.elapsed().as_micros_ceil());}
                    return Self::decode(code);
                }
            }
        }
    }

    /// Run one job task on the calling thread, with the same outcome as
    /// [`Self::work`]. Only the DRM adapter's job worker calls this: it
    /// starts on a shallow stack, as the session worker does. The submit
    /// latency histogram then counts the time from taking the session to
    /// retirement.
    fn run<F>(&self, task: F) -> core::result::Result<(),Failure>
        where F: FnOnce(&mut G18Session)->core::result::Result<(),Failure> {
        if self.failed.load(Ordering::Acquire) {return Err(Failure::Fatal(EIO));}
        let started=Instant::<Monotonic>::now();
        let code=Self::outcome(&self.session,&self.failed,&self.quarantined,task);
        if code==0 {recovery::SUBMIT_LATENCY.record(started.elapsed().as_micros_ceil());}
        Self::decode(code)
    }

    /// Run `task` on the session and reduce its result to a ticket code.
    fn outcome<F>(session:&Mutex<Retained>,failed:&AtomicBool,quarantined:&AtomicU32,task:F)->i32
        where F: FnOnce(&mut G18Session)->core::result::Result<(),Failure> {
        let mut owner=session.lock();
        let result=if failed.load(Ordering::Acquire) {Err(Failure::Fatal(EIO))} else {
            owner.0.as_mut().ok_or(Failure::Fatal(EIO)).and_then(|session|task(session))
        };
        // A job that a recovery ended fails alone; the firmware session
        // keeps serving other jobs. Anything else ends the session.
        let result=match result {
            Ok(()) => Ok(()),
            Err(Failure::Job(failure)) => match owner.0.as_ref() {
                Some(session) if !session.any_crashed() => {
                    if let JobFailure::Stranded(_,path) | JobFailure::Unbound(_,path,_)=&failure {
                        quarantined.fetch_or(path.bit(),Ordering::AcqRel);
                    }
                    log_job_failure(session,&failure);
                    Err(Failure::Job(failure))
                }
                _ => Err(Failure::Fatal(EIO)),
            },
            Err(fatal) => Err(fatal),
        };
        if let Err(Failure::Fatal(error))=result {
            failed.store(true,Ordering::Release);
            if let Some(session)=owner.0.as_mut() {
                if let Some(job)=session.application_job.as_mut() {job.quarantine();}
                if let Some(memory)=session.memory.as_mut() {memory.quarantine_application_compute();}
                dev_err!(session.device(),"G18_APP_FAIL error={:?} owners=retained no-retry\n",error);
                session.dump_fault_evidence("application worker");
                if let Some(memory)=session.memory.as_ref() {memory.log_trace_journal(&session.dev);}
            }
            let _=owner.stop("application fault");
        }
        match result {
            Ok(())=>0,
            Err(Failure::Fatal(error))=>error.to_errno(),
            Err(Failure::Job(JobFailure::Recovered(_) | JobFailure::Unbound(..) | JobFailure::Released))=>JobOutcome::Recovered as i32,
            Err(Failure::Job(JobFailure::Stranded(..) | JobFailure::Retained))=>JobOutcome::Stranded as i32,
        }
    }

    /// The submitting side of a ticket code: the worker logged the failure.
    fn decode(code:i32)->core::result::Result<(),Failure> {
        match code {
            0 => Ok(()),
            code if code==JobOutcome::Recovered as i32 => Err(Failure::Job(JobFailure::Released)),
            code if code==JobOutcome::Stranded as i32 => Err(Failure::Job(JobFailure::Retained)),
            errno => Err(Failure::Fatal(Error::from_errno(errno))),
        }
    }

    /// Fail one client's job. Its VM is poisoned only while the firmware may
    /// still reference the job's memory; a job whose queues retired after a
    /// recovery fails alone, as a faulting job does upstream.
    fn finish(vm:&mmu::Vm,result:core::result::Result<(),Failure>)->Result {
        match result {
            Ok(()) => Ok(()),
            Err(Failure::Job(JobFailure::Released)) | Err(Failure::Job(JobFailure::Recovered(_))) => Err(EIO),
            Err(Failure::Job(_)) => {if let Some(status)=vm.status(){status.record(EIO);}Err(EIO)}
            Err(Failure::Fatal(error)) => {if let Some(status)=vm.status(){status.record(error);}Err(error)}
        }
    }
}

/// Worker ticket values besides 0 and negative errnos.
const TICKET_RUNNING: i32 = 1;
#[repr(i32)]
enum JobOutcome {
    Recovered = 2,
    Stranded = 3,
}

impl G18Runtime for ApplicationOwner {
    fn shutdown(&self) {
        self.failed.store(true,Ordering::Release);
        let _=self.session.lock().stop("DRM unbind");
    }
}
impl G18DrmRuntime for ApplicationOwner {
    fn submit_compute(&self,vm:&mmu::Vm,command:&crate::g18::uapi::command::TranslatedComputeCommand)
        ->Result<G18RenderRetirement> {
        self.admit(JobPath::Compute)?;
        Self::finish(vm,self.run(|session|session.submit_application_compute(vm,command)))?;
        Ok(G18RenderRetirement {stage_ticks:None})
    }
    /// Never waits for the session: while a job holds it, that job reports
    /// a crash itself and sets `failed`.
    fn is_crashed(&self)->bool {
        self.failed.load(Ordering::Acquire)
            || self.session.try_lock().is_some_and(|owner|owner.0.as_ref().map_or(true,|s|s.any_crashed()))
    }
    fn new_user_vm(&self,id:u64,kernel_range:Range<u64>)->Result<mmu::Vm> {
        if self.failed.load(Ordering::Acquire) {return Err(EIO);}
        self.session.lock().0.as_ref().ok_or(EIO)?.uat.new_vm(id,kernel_range)?.with_t8152_lifetime()
    }
}
impl G18RenderJobSink for ApplicationOwner {
    fn submit_render(&self,vm:&mmu::Vm,render:&G18ApplicationRender)->Result<G18RenderRetirement> {
        self.admit(JobPath::Render)?;
        Self::finish(vm,self.run(|session|session.submit_application(vm,render)))?;
        // K8 uses the adapter's architectural-counter bracket until firmware
        // timestamp units and stage boundaries have hardware qualification.
        Ok(G18RenderRetirement {stage_ticks:None})
    }
}

pub(crate) fn start_drm(pdev:&platform::Device<Core<'_>>,probe:T8152Probe)
    ->Result<(Arc<dyn G18Runtime>,Arc<dyn G18DrmRuntime>)> {
    if probe.stage!=STAGE_DRM {return Err(EINVAL);}
    let retained=prepare_firmware_session(pdev,probe)?;
    let session=Arc::pin_init(kernel::new_mutex!(retained,"G18 application session"),GFP_KERNEL)?;
    let owner=Arc::new(ApplicationOwner {session,failed:Arc::new(AtomicBool::new(false),GFP_KERNEL)?,
        quarantined:Arc::new(AtomicU32::new(0),GFP_KERNEL)?},GFP_KERNEL)?;
    owner.work(|session|Ok(session.run()?)).map_err(|failure| match failure {
        Failure::Fatal(error) => error,
        Failure::Job(_) => EIO,
    })?;
    dev_info!(pdev.as_ref(),"G18_DRM_READY firmware=live graphics=on-first-submit timestamps=K8\n");
    crate::g18::debug::g18_mark_drm_live();
    Ok((owner.clone(),owner))
}

fn log_job_failure(session:&G18Session,failure:&JobFailure) {
    match failure {
        JobFailure::Recovered(r) => dev_err!(session.device(),
            "G18_APP_JOB_FAILED error={:?} recovery epoch={} host={} us={} reason={:#x} slot-mask={:#x} queues=retired session=live recoveries={}\n",
            EIO,r.epoch,r.host,r.us,r.cause.reason,r.cause.slot_mask,recovery::recoveries()),
        JobFailure::Stranded(r,path) => dev_err!(session.device(),
            "G18_APP_JOB_FAILED error={:?} recovery epoch={} host={} us={} reason={:#x} slot-mask={:#x} queues=stranded path={:?} quarantined session=live recoveries={}\n",
            EIO,r.epoch,r.host,r.us,r.cause.reason,r.cause.slot_mask,path,recovery::recoveries()),
        JobFailure::Unbound(r,path,unready) => dev_err!(session.device(),
            "G18_APP_JOB_FAILED error={:?} recovery epoch={} host={} us={} reason={:#x} slot-mask={:#x} queues=retired unbound={} path={:?} quarantined session=live recoveries={}\n",
            EIO,r.epoch,r.host,r.us,r.cause.reason,r.cause.slot_mask,unready,path,recovery::recoveries()),
        // Only the submitting thread's summaries; the worker logs the rest.
        JobFailure::Released | JobFailure::Retained => {}
    }
}

impl G18Session {
    #[inline(never)]
    fn submit_application(&mut self,vm:&mmu::Vm,render:&G18ApplicationRender)->core::result::Result<(),Failure> {
        self.poll_idle_faults("application prepare")?;
        if self.application_job.is_none() {
            self.application_job=Some(ApplicationJob::new(self.render_job.take().ok_or(EIO)?)?);
        }
        let job=self.application_job.as_mut().ok_or(EIO)?;
        job.prepare(&self.dev,&self.uat,vm,render)?;
        if job.pool_aliases.is_empty() {
            job.pool_aliases=self.memory.as_mut().ok_or(EIO)?.application_aliases(vm)?;
        }
        let frame=job.frame();let pass=job.pass()?;
        if frame.generation==1 {
            for phase in 1..=3 {
                self.check_faults("application binding")?;
                let queues=self.application_job.as_mut().ok_or(EIO)?.job.queues();
                let records=match phase {1=>queues.install_records(&pass)?,2=>queues.scheduler_records(&pass)?,_=>queues.pool_records(&pass)?};
                let outer_after=self.memory.as_mut().ok_or(EIO)?.publish_application_records(&records)?;
                Pin::new(&mut self.roles[G18Role::Gfx.index()].rtkit).send_message(EP_GFX_INTERRUPTS,0x0083_0000_0000_000a)?;
                let mut passed=false;
                for _ in 0..CONTROL_POLLS {
                    self.check_faults("application binding")?;
                    if self.memory.as_mut().ok_or(EIO)?.compute_outer_counters()?==[outer_after;3]
                        && self.application_job.as_mut().ok_or(EIO)?.job.queues().stage_gate(phase,&pass)? {passed=true;break;}
                    fsleep(Delta::from_millis(1));
                }
                if !passed {return Err(ETIMEDOUT.into());}
                dev_info!(self.device(),"G18_APP_BIND_PASS phase={}\n",phase);
            }
        }
        let job=self.application_job.as_mut().ok_or(EIO)?;
        if job.job.parameter_buffer().count_submitted_render()?!=frame.generation {return Err(EIO.into());}
        self.memory.as_mut().ok_or(EIO)?.audit_application(&self.dev,job)?;
        job.job.queues().prepare_application_pass(&self.dev,&pass,frame)?;
        self.poll_idle_faults("application pre-kick")?;
        let job=self.application_job.as_mut().ok_or(EIO)?;
        let traced=job.traced() || crate::g18::params::g18_graphics_stall_trace.value()!=0;
        let roots=job.publish()?;
        let records=job.job.queues().publish_application_pass(frame)?;
        let outer_after=self.memory.as_mut().ok_or(EIO)?.publish_application_outer(&records,frame)?;
        Pin::new(&mut self.roles[G18Role::Gfx.index()].rtkit).send_message(EP_GFX_INTERRUPTS,0x0083_0000_0000_000a)?;
        // The gallery needs a kick/retirement pair for every isolated client:
        // each client's first frames are logged. Other frames stay sparse;
        // explicit stall tracing records each job.
        if traced {
            crate::g18_kick_info!(self.device(),"G18_APP_KICK frame={} context={} roots={:x?} order=3D,TA stamp={:#x}\n",frame.generation,pass.context_id,roots,frame.stamp());
        }
        // Phases: outer, inner, scheduler submitted, completed, linked, list,
        // stamps, completion masks.
        let kicked=Instant::<Monotonic>::now();let mut first=[0i64;8];
        let end=self.wait_job("application completion",|session,ms,recovered,report| {
            let memory=session.memory.as_mut().ok_or(EIO)?;
            let outer=memory.compute_outer_counters()?;let masks=memory.completion_masks();
            let job=session.application_job.as_mut().ok_or(EIO)?;
            let inner=job.job.queues().counters()?;let stamps=job.job.stamps()?;
            let scheduler=job.job.scheduler_snapshot()?;let list=job.job.queues().completion_list()?;
            // What the firmware owns: both queues consumed and the scheduler
            // retired. After a recovery no stage completes, so the stamps and
            // completion event are not required.
            let consumed=outer==[outer_after;3] && inner==[[frame.inner_after();3];2]
                && scheduler.submitted==frame.submitted() && scheduler.completed==frame.submitted()
                && scheduler.linked==0 && list==[0,job.job.queues().job_list()];
            let completed=stamps==[frame.stamp();2] && masks[0]&0x600==0x600;
            recovery::note_phases(&mut first,&kicked,&[outer==[outer_after;3],inner==[[frame.inner_after();3];2],
                scheduler.submitted==frame.submitted(),scheduler.completed==frame.submitted(),scheduler.linked==0,
                list==[0,job.job.queues().job_list()],stamps==[frame.stamp();2],masks[0]&0x600==0x600]);
            if report {
                dev_info!(session.device(),"G18_APP_WAIT frame={} ms={} recovered={} outer={:?} inner={:?} stamps={:x?} masks={:x?} scheduler={:?} list={:x?}\n",
                    frame.generation,ms,recovered,outer,inner,stamps,masks,scheduler,list);
            }
            if report && !recovered && (5000..6000).contains(&ms) {
                let (state,scene_words,counter,scene)=session.application_job.as_mut().ok_or(EIO)?.job.parameter_buffer().recovery_words()?;
                dev_info!(session.device(),"G18_APP_PB_STALL frame={} counter={} scene={} state={:x?} scene_record={:x?}\n",
                    frame.generation,counter,scene,state,scene_words);
            }
            Ok(consumed && (completed || recovered))
        })?;
        let recovery=match end {
            JobEnd::Retired => match self.poll_faults("application retired")? {
                None => {
                    if recovery::phases_due(frame.generation) {
                        dev_info!(self.device(),"G18_JOB_PHASES frame={} us={:?} (outer inner submitted completed linked list stamps masks)\n",
                            frame.generation,first);
                    }
                    self.application_job.as_mut().ok_or(EIO)?.retired();
                    if traced || frame.generation%256==0 {
                        crate::g18_kick_info!(self.device(),"G18_APP_FRAME_PASS frame={} stamp={:#x} scheduler={} outer={} inner={}\n",
                            frame.generation,frame.stamp(),frame.submitted(),outer_after,frame.inner_after());
                    }
                    return Ok(());
                }
                // Retired, but a recovery followed at once: do not vouch for
                // the frame. Its queues are retired either way.
                Some(recovery) => recovery,
            },
            JobEnd::Recovered(recovery) => recovery,
            JobEnd::Stranded(recovery) => {
                self.application_job.as_mut().ok_or(EIO)?.quarantine();
                dev_err!(self.device(),"G18_APP_STRANDED frame={} epoch={} host={}: queues never retired; render quarantined, owners retained\n",
                    frame.generation,recovery.epoch,recovery.host);
                return Err(Failure::Job(JobFailure::Stranded(recovery,JobPath::Render)));
            }
        };
        // No stage will write this frame's completion stamps; the next
        // frame's audit requires them as its previous stamp.
        let job=self.application_job.as_mut().ok_or(EIO)?;
        job.job.publish_recovered_stamps(frame.previous_stamp(),frame.stamp())?;
        job.retired();
        dev_err!(self.device(),"G18_APP_RECOVERED frame={} epoch={} host={}: queues retired\n",
            frame.generation,recovery.epoch,recovery.host);
        let pb=self.application_job.as_mut().ok_or(EIO)?.job.parameter_buffer();
        let (state,scene_words,counter,scene)=pb.recovery_words()?;
        dev_err!(self.device(),"G18_APP_PB_POSTRECOVERY frame={} counter={} scene={} state={:x?} scene_record={:x?}\n",
            frame.generation,counter,scene,state,scene_words);
        // The next frame needs the queues' firmware bindings as they were; a
        // recovery may reset them. Refuse the path rather than kick into it.
        let queues=self.application_job.as_mut().ok_or(EIO)?.job.queues();
        let unready=queues.render_unready(&pass,frame)?;
        let (status,pool,slot)=queues.render_state(&pass)?;
        dev_err!(self.device(),"G18_APP_RENDER_POSTRECOVERY frame={} unready={:?} status={:x?} pool={:x?} pb={:x?}\n",
            frame.generation,unready,status,pool,slot);
        match unready {
            None => Err(Failure::Job(JobFailure::Recovered(recovery))),
            Some(what) => Err(Failure::Job(JobFailure::Unbound(recovery,JobPath::Render,what))),
        }
    }
}
