// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#ifndef ASAHI_HEADER
#define ASAHI_HEADER "drm/asahi_drm.h"
#endif
#include ASAHI_HEADER

static_assert(sizeof(struct drm_asahi_cmd_header) == 8, "command header");
static_assert(sizeof(struct drm_asahi_attachment) == 24, "attachment");
static_assert(sizeof(struct drm_asahi_timestamps) == 16, "timestamps");
static_assert(offsetof(struct drm_asahi_cmd_compute, ts_end_copies) == 64,
	      "compute prefix");
static_assert(offsetof(struct drm_asahi_cmd_render, ts_frag_end_copies) == 256,
	      "render prefix");

static int emit(unsigned int type, const void *payload, size_t size)
{
	struct drm_asahi_cmd_header header = {
		.cmd_type = type,
		.size = size,
		.vdm_barrier = DRM_ASAHI_BARRIER_NONE,
		.cdm_barrier = DRM_ASAHI_BARRIER_NONE,
	};

	return fwrite(&header, sizeof(header), 1, stdout) != 1 ||
	       fwrite(payload, size, 1, stdout) != 1;
}

/* Named C offsets are exported independently of the Rust decoder. */
static void layout(void)
{
	printf("profile.params_required %zu 0\n",
	       offsetof(struct drm_asahi_params_global, gpu_hal_generation) + sizeof(__u32));
	printf("profile.vm_create_size %zu 0\n", sizeof(struct drm_asahi_vm_create));
	printf("profile.hal200 %u 0\n", (unsigned int)DRM_ASAHI_GPU_HAL_200);
	printf("profile.features %u 0\n",
	       DRM_ASAHI_FEATURE_VM_STATUS | DRM_ASAHI_FEATURE_SCHEDULED_QUEUES);
	printf("header %zu 0\n", sizeof(struct drm_asahi_cmd_header));
	printf("attachment %zu 0\n", sizeof(struct drm_asahi_attachment));
	printf("timestamps %zu 0\n", sizeof(struct drm_asahi_timestamps));
	printf("cmd_render.flags %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, flags),
	       sizeof(((struct drm_asahi_cmd_render *)0)->flags));
	printf("cmd_render.isp_zls_pixels %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, isp_zls_pixels),
	       sizeof(((struct drm_asahi_cmd_render *)0)->isp_zls_pixels));
	printf("cmd_render.vdm_ctrl_stream_base %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, vdm_ctrl_stream_base),
	       sizeof(((struct drm_asahi_cmd_render *)0)->vdm_ctrl_stream_base));
	printf("cmd_render.vertex_helper.binary %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, vertex_helper.binary),
	       sizeof(((struct drm_asahi_cmd_render *)0)->vertex_helper.binary));
	printf("cmd_render.vertex_helper.cfg %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, vertex_helper.cfg),
	       sizeof(((struct drm_asahi_cmd_render *)0)->vertex_helper.cfg));
	printf("cmd_render.vertex_helper.data %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, vertex_helper.data),
	       sizeof(((struct drm_asahi_cmd_render *)0)->vertex_helper.data));
	printf("cmd_render.fragment_helper.binary %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, fragment_helper.binary),
	       sizeof(((struct drm_asahi_cmd_render *)0)->fragment_helper.binary));
	printf("cmd_render.fragment_helper.cfg %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, fragment_helper.cfg),
	       sizeof(((struct drm_asahi_cmd_render *)0)->fragment_helper.cfg));
	printf("cmd_render.fragment_helper.data %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, fragment_helper.data),
	       sizeof(((struct drm_asahi_cmd_render *)0)->fragment_helper.data));
	printf("cmd_render.isp_scissor_base %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, isp_scissor_base),
	       sizeof(((struct drm_asahi_cmd_render *)0)->isp_scissor_base));
	printf("cmd_render.isp_dbias_base %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, isp_dbias_base),
	       sizeof(((struct drm_asahi_cmd_render *)0)->isp_dbias_base));
	printf("cmd_render.isp_oclqry_base %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, isp_oclqry_base),
	       sizeof(((struct drm_asahi_cmd_render *)0)->isp_oclqry_base));
	printf("cmd_render.depth.base %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, depth.base),
	       sizeof(((struct drm_asahi_cmd_render *)0)->depth.base));
	printf("cmd_render.depth.comp_base %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, depth.comp_base),
	       sizeof(((struct drm_asahi_cmd_render *)0)->depth.comp_base));
	printf("cmd_render.depth.stride %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, depth.stride),
	       sizeof(((struct drm_asahi_cmd_render *)0)->depth.stride));
	printf("cmd_render.depth.comp_stride %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, depth.comp_stride),
	       sizeof(((struct drm_asahi_cmd_render *)0)->depth.comp_stride));
	printf("cmd_render.stencil.base %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, stencil.base),
	       sizeof(((struct drm_asahi_cmd_render *)0)->stencil.base));
	printf("cmd_render.stencil.comp_base %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, stencil.comp_base),
	       sizeof(((struct drm_asahi_cmd_render *)0)->stencil.comp_base));
	printf("cmd_render.stencil.stride %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, stencil.stride),
	       sizeof(((struct drm_asahi_cmd_render *)0)->stencil.stride));
	printf("cmd_render.stencil.comp_stride %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, stencil.comp_stride),
	       sizeof(((struct drm_asahi_cmd_render *)0)->stencil.comp_stride));
	printf("cmd_render.zls_ctrl %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, zls_ctrl),
	       sizeof(((struct drm_asahi_cmd_render *)0)->zls_ctrl));
	printf("cmd_render.ppp_multisamplectl %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, ppp_multisamplectl),
	       sizeof(((struct drm_asahi_cmd_render *)0)->ppp_multisamplectl));
	printf("cmd_render.sampler_heap %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, sampler_heap),
	       sizeof(((struct drm_asahi_cmd_render *)0)->sampler_heap));
	printf("cmd_render.ppp_ctrl %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, ppp_ctrl),
	       sizeof(((struct drm_asahi_cmd_render *)0)->ppp_ctrl));
	printf("cmd_render.width_px %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, width_px),
	       sizeof(((struct drm_asahi_cmd_render *)0)->width_px));
	printf("cmd_render.height_px %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, height_px),
	       sizeof(((struct drm_asahi_cmd_render *)0)->height_px));
	printf("cmd_render.layers %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, layers),
	       sizeof(((struct drm_asahi_cmd_render *)0)->layers));
	printf("cmd_render.sampler_count %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, sampler_count),
	       sizeof(((struct drm_asahi_cmd_render *)0)->sampler_count));
	printf("cmd_render.utile_width_px %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, utile_width_px),
	       sizeof(((struct drm_asahi_cmd_render *)0)->utile_width_px));
	printf("cmd_render.utile_height_px %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, utile_height_px),
	       sizeof(((struct drm_asahi_cmd_render *)0)->utile_height_px));
	printf("cmd_render.samples %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, samples),
	       sizeof(((struct drm_asahi_cmd_render *)0)->samples));
	printf("cmd_render.sample_size_B %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, sample_size_B),
	       sizeof(((struct drm_asahi_cmd_render *)0)->sample_size_B));
	printf("cmd_render.isp_merge_upper_x %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, isp_merge_upper_x),
	       sizeof(((struct drm_asahi_cmd_render *)0)->isp_merge_upper_x));
	printf("cmd_render.isp_merge_upper_y %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, isp_merge_upper_y),
	       sizeof(((struct drm_asahi_cmd_render *)0)->isp_merge_upper_y));
	printf("cmd_render.bg.usc %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, bg.usc),
	       sizeof(((struct drm_asahi_cmd_render *)0)->bg.usc));
	printf("cmd_render.bg.rsrc_spec %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, bg.rsrc_spec),
	       sizeof(((struct drm_asahi_cmd_render *)0)->bg.rsrc_spec));
	printf("cmd_render.bg_rsrc_spec_hi %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, bg_rsrc_spec_hi),
	       sizeof(((struct drm_asahi_cmd_render *)0)->bg_rsrc_spec_hi));
	printf("cmd_render.eot.usc %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, eot.usc),
	       sizeof(((struct drm_asahi_cmd_render *)0)->eot.usc));
	printf("cmd_render.eot.rsrc_spec %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, eot.rsrc_spec),
	       sizeof(((struct drm_asahi_cmd_render *)0)->eot.rsrc_spec));
	printf("cmd_render.bg_eot_rsrc_spec_hi %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, bg_eot_rsrc_spec_hi),
	       sizeof(((struct drm_asahi_cmd_render *)0)->bg_eot_rsrc_spec_hi));
	printf("cmd_render.partial_bg.usc %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, partial_bg.usc),
	       sizeof(((struct drm_asahi_cmd_render *)0)->partial_bg.usc));
	printf("cmd_render.partial_bg.rsrc_spec %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, partial_bg.rsrc_spec),
	       sizeof(((struct drm_asahi_cmd_render *)0)->partial_bg.rsrc_spec));
	printf("cmd_render.bg_partial_rsrc_spec_hi %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, bg_partial_rsrc_spec_hi),
	       sizeof(((struct drm_asahi_cmd_render *)0)->bg_partial_rsrc_spec_hi));
	printf("cmd_render.partial_eot.usc %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, partial_eot.usc),
	       sizeof(((struct drm_asahi_cmd_render *)0)->partial_eot.usc));
	printf("cmd_render.partial_eot.rsrc_spec %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, partial_eot.rsrc_spec),
	       sizeof(((struct drm_asahi_cmd_render *)0)->partial_eot.rsrc_spec));
	printf("cmd_render.bg_eot_partial_rsrc_spec_hi %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, bg_eot_partial_rsrc_spec_hi),
	       sizeof(((struct drm_asahi_cmd_render *)0)->bg_eot_partial_rsrc_spec_hi));
	printf("cmd_render.isp_bgobjdepth %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, isp_bgobjdepth),
	       sizeof(((struct drm_asahi_cmd_render *)0)->isp_bgobjdepth));
	printf("cmd_render.isp_bgobjvals %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, isp_bgobjvals),
	       sizeof(((struct drm_asahi_cmd_render *)0)->isp_bgobjvals));
	printf("cmd_render.ts_vtx.start.handle %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, ts_vtx.start.handle),
	       sizeof(((struct drm_asahi_cmd_render *)0)->ts_vtx.start.handle));
	printf("cmd_render.ts_vtx.start.offset %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, ts_vtx.start.offset),
	       sizeof(((struct drm_asahi_cmd_render *)0)->ts_vtx.start.offset));
	printf("cmd_render.ts_vtx.end.handle %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, ts_vtx.end.handle),
	       sizeof(((struct drm_asahi_cmd_render *)0)->ts_vtx.end.handle));
	printf("cmd_render.ts_vtx.end.offset %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, ts_vtx.end.offset),
	       sizeof(((struct drm_asahi_cmd_render *)0)->ts_vtx.end.offset));
	printf("cmd_render.ts_frag.start.handle %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, ts_frag.start.handle),
	       sizeof(((struct drm_asahi_cmd_render *)0)->ts_frag.start.handle));
	printf("cmd_render.ts_frag.start.offset %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, ts_frag.start.offset),
	       sizeof(((struct drm_asahi_cmd_render *)0)->ts_frag.start.offset));
	printf("cmd_render.ts_frag.end.handle %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, ts_frag.end.handle),
	       sizeof(((struct drm_asahi_cmd_render *)0)->ts_frag.end.handle));
	printf("cmd_render.ts_frag.end.offset %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, ts_frag.end.offset),
	       sizeof(((struct drm_asahi_cmd_render *)0)->ts_frag.end.offset));
	printf("cmd_compute.flags %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_compute, flags),
	       sizeof(((struct drm_asahi_cmd_compute *)0)->flags));
	printf("cmd_compute.sampler_count %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_compute, sampler_count),
	       sizeof(((struct drm_asahi_cmd_compute *)0)->sampler_count));
	printf("cmd_compute.cdm_ctrl_stream_base %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_compute, cdm_ctrl_stream_base),
	       sizeof(((struct drm_asahi_cmd_compute *)0)->cdm_ctrl_stream_base));
	printf("cmd_compute.cdm_ctrl_stream_end %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_compute, cdm_ctrl_stream_end),
	       sizeof(((struct drm_asahi_cmd_compute *)0)->cdm_ctrl_stream_end));
	printf("cmd_compute.sampler_heap %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_compute, sampler_heap),
	       sizeof(((struct drm_asahi_cmd_compute *)0)->sampler_heap));
	printf("cmd_compute.helper.binary %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_compute, helper.binary),
	       sizeof(((struct drm_asahi_cmd_compute *)0)->helper.binary));
	printf("cmd_compute.helper.cfg %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_compute, helper.cfg),
	       sizeof(((struct drm_asahi_cmd_compute *)0)->helper.cfg));
	printf("cmd_compute.helper.data %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_compute, helper.data),
	       sizeof(((struct drm_asahi_cmd_compute *)0)->helper.data));
	printf("cmd_compute.ts.start.handle %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_compute, ts.start.handle),
	       sizeof(((struct drm_asahi_cmd_compute *)0)->ts.start.handle));
	printf("cmd_compute.ts.start.offset %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_compute, ts.start.offset),
	       sizeof(((struct drm_asahi_cmd_compute *)0)->ts.start.offset));
	printf("cmd_compute.ts.end.handle %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_compute, ts.end.handle),
	       sizeof(((struct drm_asahi_cmd_compute *)0)->ts.end.handle));
	printf("cmd_compute.ts.end.offset %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_compute, ts.end.offset),
	       sizeof(((struct drm_asahi_cmd_compute *)0)->ts.end.offset));
	printf("attachment.pointer %zu %zu\n",
	       offsetof(struct drm_asahi_attachment, pointer),
	       sizeof(((struct drm_asahi_attachment *)0)->pointer));
	printf("attachment.size %zu %zu\n",
	       offsetof(struct drm_asahi_attachment, size),
	       sizeof(((struct drm_asahi_attachment *)0)->size));
	printf("cmd_render.ts_frag_end_copies %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_render, ts_frag_end_copies),
	       sizeof(((struct drm_asahi_cmd_render *)0)->ts_frag_end_copies));
	printf("cmd_compute.ts_end_copies %zu %zu\n",
	       offsetof(struct drm_asahi_cmd_compute, ts_end_copies),
	       sizeof(((struct drm_asahi_cmd_compute *)0)->ts_end_copies));
}

int main(int argc, char **argv)
{
	struct drm_asahi_attachment attachment = {0};

	attachment.pointer = 0xcdb09376593c1f02ULL;
	attachment.size = 0xceb194775a3d2003ULL;

	struct drm_asahi_cmd_render render = {0};

	render.flags = 0x5b3e2104;
	render.isp_zls_pixels = 0x5c3f2205;
	render.vdm_ctrl_stream_base = 0xd1b4977a5d402306ULL;
	render.vertex_helper.binary = 0x5e412407;
	render.vertex_helper.cfg = 0x5f422508;
	render.vertex_helper.data = 0xd4b79a7d60432609ULL;
	render.fragment_helper.binary = 0x6144270a;
	render.fragment_helper.cfg = 0x6245280b;
	render.fragment_helper.data = 0xd7ba9d806346290cULL;
	render.isp_scissor_base = 0xd8bb9e8164472a0dULL;
	render.isp_dbias_base = 0xd9bc9f8265482b0eULL;
	render.isp_oclqry_base = 0xdabda08366492c0fULL;
	render.depth.base = 0xdbbea184674a2d10ULL;
	render.depth.comp_base = 0xdcbfa285684b2e11ULL;
	render.depth.stride = 0x694c2f12;
	render.depth.comp_stride = 0x6a4d3013;
	render.stencil.base = 0xdfc2a5886b4e3114ULL;
	render.stencil.comp_base = 0xe0c3a6896c4f3215ULL;
	render.stencil.stride = 0x6d503316;
	render.stencil.comp_stride = 0x6e513417;
	render.zls_ctrl = 0xe3c6a98c6f523518ULL;
	render.ppp_multisamplectl = 0xe4c7aa8d70533619ULL;
	render.sampler_heap = 0xe5c8ab8e7154371aULL;
	render.ppp_ctrl = 0x7255381b;
	render.width_px = 0x391c;
	render.height_px = 0x3a1d;
	render.layers = 0x3b1e;
	render.sampler_count = 0x3c1f;
	render.utile_width_px = 0x20;
	render.utile_height_px = 0x21;
	render.samples = 0x22;
	render.sample_size_B = 0x23;
	render.isp_merge_upper_x = 0x7b5e4124;
	render.isp_merge_upper_y = 0x7c5f4225;
	render.bg.usc = 0x7d604326;
	render.bg.rsrc_spec = 0x7e614427;
	render.bg_rsrc_spec_hi = 0x7f624528;
	render.eot.usc = 0x80634629;
	render.eot.rsrc_spec = 0x8164472a;
	render.bg_eot_rsrc_spec_hi = 0x8265482b;
	render.partial_bg.usc = 0x8366492c;
	render.partial_bg.rsrc_spec = 0x84674a2d;
	render.bg_partial_rsrc_spec_hi = 0x85684b2e;
	render.partial_eot.usc = 0x86694c2f;
	render.partial_eot.rsrc_spec = 0x876a4d30;
	render.bg_eot_partial_rsrc_spec_hi = 0x886b4e31;
	render.isp_bgobjdepth = 0x896c4f32;
	render.isp_bgobjvals = 0x8a6d5033;
	render.ts_vtx.start.handle = 0x8b6e5134;
	render.ts_vtx.start.offset = 0x8c6f5235;
	render.ts_vtx.end.handle = 0x8d705336;
	render.ts_vtx.end.offset = 0x8e715437;
	render.ts_frag.start.handle = 0x8f725538;
	render.ts_frag.start.offset = 0x90735639;
	render.ts_frag.end.handle = 0x9174573a;
	render.ts_frag.end.offset = 0x9275583b;

	struct drm_asahi_cmd_compute compute = {0};

	compute.flags = 0x9376593c;
	compute.sampler_count = 0x94775a3d;
	compute.cdm_ctrl_stream_base = 0xbeccfb295785b3eULL;
	compute.cdm_ctrl_stream_end = 0xcedd0b396795c3fULL;
	compute.sampler_heap = 0xdeed1b4977a5d40ULL;
	compute.helper.binary = 0x987b5e41;
	compute.helper.cfg = 0x997c5f42;
	compute.helper.data = 0x10f1d4b79a7d6043ULL;
	compute.ts.start.handle = 0x9b7e6144;
	compute.ts.start.offset = 0x9c7f6245;
	compute.ts.end.handle = 0x9d806346;
	compute.ts.end.offset = 0x9e816447;

	size_t render_size =
		offsetof(struct drm_asahi_cmd_render, ts_frag_end_copies);
	size_t compute_size =
		offsetof(struct drm_asahi_cmd_compute, ts_end_copies);

	if (argc == 2 && !strcmp(argv[1], "--layout")) {
		layout();
		return 0;
	} else if (argc == 2 && !strcmp(argv[1], "--full")) {
		render_size = sizeof(render);
		compute_size = sizeof(compute);
	} else if (argc != 1) {
		return 1;
	}

	return emit(DRM_ASAHI_SET_VERTEX_ATTACHMENTS, &attachment,
		    sizeof(attachment)) ||
	       emit(DRM_ASAHI_CMD_RENDER, &render, render_size) ||
	       emit(DRM_ASAHI_CMD_COMPUTE, &compute, compute_size);
}
