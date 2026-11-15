/*
 * Apple T8152 (M6) cores clear architectural state across WFIT, the FP/SIMD
 * registers included. __delay() would execute it from any context, including
 * the cpufreq transition polling that load-driven governors run on a CPU
 * whose current task still has live FP/SIMD state: on J873 (M6) 25-56% of
 * SHA-256 runs gave wrong results under schedutil. Treat WFxT as absent
 * there, as arm64.nowfxt does, so __delay() uses WFE. T8142 (M5) was not
 * measured; it is included because apple_m5_needs_wait_save() groups its
 * cores with T8152 for this state loss.
 *
 * T6050 (M5 Pro and Max, parts 0x064 and 0x065) was not measured for the WFIT
 * state loss either. On the K116s, at EL1 under the boot loader's monitor,
 * ID_AA64ISAR2_EL1.WFxT reads 0, so the capability is not set there today
 * with or without these entries, and __delay() already avoids WFIT on these
 * cores by MIDR. The entries are a precaution for a kernel started at EL2,
 * where the field may read as implemented.
 */
static bool has_wfxt(const struct arm64_cpu_capabilities *entry, int scope)
{
	static const struct midr_range apple_wfxt_loses_state[] = {
		MIDR_ALL_VERSIONS(MIDR_CPU_MODEL(ARM_CPU_IMP_APPLE, APPLE_CPU_PART_T8142_E)),
		MIDR_ALL_VERSIONS(MIDR_CPU_MODEL(ARM_CPU_IMP_APPLE, APPLE_CPU_PART_T8142_P)),
		MIDR_ALL_VERSIONS(MIDR_CPU_MODEL(ARM_CPU_IMP_APPLE, APPLE_CPU_PART_T6050_064)),
		MIDR_ALL_VERSIONS(MIDR_CPU_MODEL(ARM_CPU_IMP_APPLE, APPLE_CPU_PART_T6050_065)),
		MIDR_ALL_VERSIONS(MIDR_CPU_MODEL(ARM_CPU_IMP_APPLE, APPLE_CPU_PART_T8152_E)),
		MIDR_ALL_VERSIONS(MIDR_CPU_MODEL(ARM_CPU_IMP_APPLE, APPLE_CPU_PART_T8152_P)),
		{}
	};

	if (is_midr_in_range_list(apple_wfxt_loses_state))
		return false;
	return has_cpuid_feature(entry, scope);
}

