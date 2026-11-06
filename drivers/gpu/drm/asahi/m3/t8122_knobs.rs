// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! The parameters of the T8122 start experiment (`t8122_start`): their text, defaults, accepted
//! values and their meaning. Plain data only, so that the host tests run it (tools/asahi).

/// A parameter that is not given.
pub(crate) const UNSET: u64 = u64::MAX;
/// A parameter whose text is not one of its accepted values. The experiment refuses to start
/// with it rather than fall back to a default.
pub(crate) const INVALID: u64 = u64::MAX - 1;

/// The Fender window sizes: the ADT sgx reg[1] size less 0x28000 (the difference on T6030), and
/// the whole ADT sgx reg[1].
pub(crate) const FENDER_RULE: u32 = 0x10_4000;
pub(crate) const FENDER_ADT: u32 = 0x12_c000;
/// The GPU clock-generator offsets in SGX: the T8122 table's (`T8122_IO_MAPPINGS`, the default),
/// and the T6030 one.
pub(crate) const CLOCK_GEN_E1C: u64 = 0xe1_c000;
pub(crate) const CLOCK_GEN_E5C: u64 = 0xe5_c000;
/// The SGX setup write of T6030 and T8122 (offset, value).
pub(crate) const T6030_SGX_SETUP: (usize, u32) = (0xd1_4000, 0x7_0001);
/// The default unit masks: the T8122 table's (`m3_soc::T8122_HWDATA_B`), which are T6030's.
pub(crate) const UNIT_MASK_A: u64 = 0x7_0000_0003;
pub(crate) const UNIT_MASK_B: u32 = 0x7;
/// No unit-mask bit outside these is accepted: the default masks.
pub(crate) const UNIT_MASK_A_LIMIT: u64 = 0x7_0000_0003;
pub(crate) const UNIT_MASK_B_LIMIT: u64 = 0x7;
/// The startup and sensor-failure cap, also the fixed ceiling with thermal control off.
pub(crate) const PSTATE_CAP: u32 = 2;
/// The highest performance-state ceiling the experiment accepts. The same as the driver's own
/// limit when thermal control is off (`m3_adt_config::ADT_MAX_PSTATE_LIMIT`). An omitted cap
/// lets thermal control use the published table; explicit experiment caps remain bounded.
pub(crate) const PSTATE_CAP_MAX: u32 = 5;

/// Parse a decimal or `0x`-prefixed hexadecimal `u64`. A leading sign is not accepted.
fn number(text: &str) -> Option<u64> {
    if text.starts_with(['+', '-']) {
        return None;
    }
    match text.strip_prefix("0x").or_else(|| text.strip_prefix("0X")) {
        Some(hex) if !hex.starts_with(['+', '-']) => u64::from_str_radix(hex, 16).ok(),
        Some(_) => None,
        None => text.parse::<u64>().ok(),
    }
}

/// Parse a parameter's text: one of `names`, else a number. Anything else, and the two
/// reserved values, give [`INVALID`].
fn parse(text: &str, names: &[(&str, u64)]) -> u64 {
    let text = text.trim();
    if let Some(&(_, value)) = names.iter().find(|(name, _)| *name == text) {
        return value;
    }
    number(text).filter(|v| *v < INVALID).unwrap_or(INVALID)
}

pub(crate) fn parse_number(text: &str) -> u64 {
    parse(text, &[])
}

pub(crate) fn parse_fender(text: &str) -> u64 {
    parse(text, &[("rule", FENDER_RULE as u64), ("adt", FENDER_ADT as u64)])
}

pub(crate) fn parse_clkgen(text: &str) -> u64 {
    parse(text, &[("e1c", CLOCK_GEN_E1C), ("e5c", CLOCK_GEN_E5C), ("none", 0)])
}

pub(crate) fn parse_sgx_setup(text: &str) -> u64 {
    parse(text, &[("none", 0), ("t6030", 1)])
}

/// The parameters as given: [`UNSET`], [`INVALID`] or a value.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct Raw {
    pub(crate) start: u64,
    pub(crate) initdata_version: u64,
    pub(crate) fender: u64,
    pub(crate) clkgen: u64,
    pub(crate) sgx_setup: u64,
    pub(crate) unit_mask_a: u64,
    pub(crate) unit_mask_b: u64,
    pub(crate) pstate_cap: u64,
}

impl Raw {
    /// No parameter given.
    #[cfg(test)]
    pub(crate) const NONE: Raw = Raw {
        start: UNSET,
        initdata_version: UNSET,
        fender: UNSET,
        clkgen: UNSET,
        sgx_setup: UNSET,
        unit_mask_a: UNSET,
        unit_mask_b: UNSET,
        pstate_cap: UNSET,
    };

    /// Whether any parameter other than `asahi.t8122_start` was given.
    pub(crate) fn values_given(&self) -> bool {
        [
            self.initdata_version,
            self.fender,
            self.clkgen,
            self.sgx_setup,
            self.unit_mask_a,
            self.unit_mask_b,
            self.pstate_cap,
        ]
        .iter()
        .any(|v| *v != UNSET)
    }
}

/// What `asahi.t8122_start` asks for.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Start {
    /// Not given, or 0.
    Off,
    /// 1.
    On,
    /// Anything else.
    Invalid,
}

pub(crate) fn start(raw: &Raw) -> Start {
    match raw.start {
        UNSET | 0 => Start::Off,
        1 => Start::On,
        _ => Start::Invalid,
    }
}

/// The shipped board policy changes only an omitted start parameter. Explicit recovery-off,
/// explicit experimental start, and malformed values retain their original meanings.
/// Hardware resources and firmware identity are admitted separately by the caller.
pub(crate) fn start_for_board(raw: &Raw, admitted_board: bool) -> Start {
    if raw.start == UNSET && admitted_board {
        Start::On
    } else {
        start(raw)
    }
}

/// The GPU clock-generator IO mapping.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum ClockGen {
    /// SGX + the offset, mapped read-only for the firmware.
    At(u64),
    /// No mapping: HwDataB slot 29 stays zero.
    Absent,
}

/// The values of one boot.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct Values {
    pub(crate) initdata_version: u64,
    pub(crate) fender: u32,
    pub(crate) clock_gen: ClockGen,
    pub(crate) sgx_setup: Option<(usize, u32)>,
    pub(crate) unit_mask_a: u64,
    pub(crate) unit_mask_b: u32,
    pub(crate) pstate_cap: u32,
    pub(crate) pstate_cap_explicit: bool,
}

/// Firmware ceiling and initial runtime cap for one T8122 boot.
pub(crate) struct PstateLimits {
    /// None lets the existing policy select the ceiling from the published table.
    pub(crate) ceiling: Option<u32>,
    pub(crate) safe: u32,
}

impl Values {
    /// Only thermal control may lift an omitted experiment cap. Explicit caps retain their
    /// fixed ceiling and initial cap, including when thermal control is disabled.
    pub(crate) fn pstate_limits(&self, thermal_enabled: bool) -> PstateLimits {
        PstateLimits {
            ceiling: (self.pstate_cap_explicit || !thermal_enabled).then_some(self.pstate_cap),
            safe: self.pstate_cap,
        }
    }
}

/// A parameter given with a value it does not accept: its name and what it accepts.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct Refusal {
    pub(crate) name: &'static str,
    pub(crate) accepts: &'static str,
}

/// A parameter's value, its default when not given; None when invalid.
fn given(param: u64, default: u64) -> Option<u64> {
    match param {
        UNSET => Some(default),
        INVALID => None,
        v => Some(v),
    }
}

/// Resolve the parameters. The first parameter with a value it does not accept is the refusal.
pub(crate) fn resolve(raw: &Raw) -> Result<Values, Refusal> {
    let refuse = |name, accepts| Refusal { name, accepts };
    let initdata_version = given(raw.initdata_version, crate::m3::m3_firmware::G15_V14_8_3_INITDATA)
        .filter(|v| *v != 0)
        .ok_or(refuse("t8122_initdata_version", "a nonzero number, at most 0xfffffffffffffffd"))?;
    let fender = match given(raw.fender, FENDER_RULE as u64) {
        Some(v) if v == FENDER_RULE as u64 || v == FENDER_ADT as u64 => v as u32,
        _ => return Err(refuse("t8122_fender", "0x104000 or rule, 0x12c000 or adt")),
    };
    let clock_gen = match given(raw.clkgen, CLOCK_GEN_E1C) {
        Some(0) => ClockGen::Absent,
        Some(v) if v == CLOCK_GEN_E1C || v == CLOCK_GEN_E5C => ClockGen::At(v),
        _ => return Err(refuse("t8122_clkgen", "e1c, e5c or none")),
    };
    let sgx_setup = match given(raw.sgx_setup, 1) {
        Some(0) => None,
        Some(1) => Some(T6030_SGX_SETUP),
        _ => return Err(refuse("t8122_sgx_setup", "none or t6030")),
    };
    let unit_mask_a = given(raw.unit_mask_a, UNIT_MASK_A)
        .filter(|v| *v != 0 && *v & !UNIT_MASK_A_LIMIT == 0)
        .ok_or(refuse("t8122_unit_mask_a", "nonzero, no bits outside 0x700000003"))?;
    let unit_mask_b = given(raw.unit_mask_b, UNIT_MASK_B as u64)
        .filter(|v| *v != 0 && *v & !UNIT_MASK_B_LIMIT == 0)
        .ok_or(refuse("t8122_unit_mask_b", "nonzero, no bits outside 0x7"))? as u32;
    let pstate_cap = given(raw.pstate_cap, PSTATE_CAP as u64)
        .filter(|v| (1..=u64::from(PSTATE_CAP_MAX)).contains(v))
        .ok_or(refuse("t8122_pstate_cap", "1 to 5"))? as u32;
    Ok(Values {
        initdata_version,
        fender,
        clock_gen,
        sgx_setup,
        unit_mask_a,
        unit_mask_b,
        pstate_cap,
        pstate_cap_explicit: raw.pstate_cap != UNSET,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn with(edit: impl Fn(&mut Raw)) -> Raw {
        let mut raw = Raw { start: 1, ..Raw::NONE };
        edit(&mut raw);
        raw
    }

    #[test]
    fn nothing_given_is_off_and_gives_no_values() {
        assert_eq!(start(&Raw::NONE), Start::Off);
        assert!(!Raw::NONE.values_given());
        assert_eq!(start(&with(|r| r.start = 0)), Start::Off);
        assert_eq!(start(&with(|_| {})), Start::On);
        assert_eq!(start(&with(|r| r.start = 2)), Start::Invalid);
        assert_eq!(start(&with(|r| r.start = INVALID)), Start::Invalid);
        assert!(!with(|_| {}).values_given());
        assert!(with(|r| r.pstate_cap = 3).values_given());
        assert!(with(|r| r.fender = INVALID).values_given());
    }

    #[test]
    fn automatic_start_is_board_scoped_and_explicit_values_win() {
        assert_eq!(start_for_board(&Raw::NONE, true), Start::On);
        assert_eq!(start_for_board(&Raw::NONE, false), Start::Off);
        for admitted_board in [false, true] {
            assert_eq!(start_for_board(&with(|r| r.start = 0), admitted_board), Start::Off);
            assert_eq!(start_for_board(&with(|_| {}), admitted_board), Start::On);
            assert_eq!(start_for_board(&with(|r| r.start = 2), admitted_board), Start::Invalid);
            assert_eq!(start_for_board(&with(|r| r.start = INVALID), admitted_board), Start::Invalid);
            assert_eq!(start_for_board(&with(|r| r.start = parse_number("bogus")), admitted_board), Start::Invalid);
        }
    }

    #[test]
    fn defaults_are_the_plan_baseline() {
        let v = resolve(&with(|_| {})).unwrap();
        assert_eq!(v.initdata_version, 0x0c08_e21e_8380_0490);
        assert_eq!(v.fender, 0x10_4000);
        assert_eq!(v.clock_gen, ClockGen::At(0xe1_c000));
        assert_eq!(v.sgx_setup, Some((0xd1_4000, 0x7_0001)));
        assert_eq!((v.unit_mask_a, v.unit_mask_b), (0x7_0000_0003, 7));
        assert_eq!(v.pstate_cap, 2);
        assert!(!v.pstate_cap_explicit);
    }

    #[test]
    fn every_alternative_resolves() {
        let v = resolve(&with(|r| {
            r.initdata_version = parse_number("0x123456789abcdef0");
            r.fender = parse_fender("adt");
            r.clkgen = parse_clkgen("e5c");
            r.sgx_setup = parse_sgx_setup("none");
            r.unit_mask_a = parse_number("1");
            r.unit_mask_b = parse_number("0x1");
            r.pstate_cap = parse_number("5");
        }))
        .unwrap();
        assert_eq!(v.initdata_version, 0x1234_5678_9abc_def0);
        assert_eq!(v.fender, 0x12_c000);
        assert_eq!(v.clock_gen, ClockGen::At(0xe5_c000));
        assert_eq!(v.sgx_setup, None);
        assert_eq!((v.unit_mask_a, v.unit_mask_b, v.pstate_cap), (1, 1, 5));
        let v = resolve(&with(|r| r.clkgen = parse_clkgen("none"))).unwrap();
        assert_eq!(v.clock_gen, ClockGen::Absent);
        // Numbers and names give the same values.
        assert_eq!(parse_fender("0x104000"), parse_fender("rule"));
        assert_eq!(parse_fender(" 0x12C000 "), parse_fender("adt"));
        assert_eq!(parse_clkgen("0xe5c000"), parse_clkgen("e5c"));
        assert_eq!(parse_clkgen("0xe1c000"), parse_clkgen("e1c"));
        assert_eq!(parse_clkgen("0"), parse_clkgen("none"));
        assert_eq!(parse_sgx_setup("1"), parse_sgx_setup("t6030"));
    }

    #[test]
    fn unaccepted_values_refuse_and_name_the_parameter() {
        let cases: [(fn(&mut Raw), &str); 15] = [
            (|r| r.initdata_version = parse_number("g15s"), "t8122_initdata_version"),
            (|r| r.initdata_version = 0, "t8122_initdata_version"),
            (|r| r.fender = parse_fender("0x144000"), "t8122_fender"),
            (|r| r.fender = parse_fender("big"), "t8122_fender"),
            (|r| r.clkgen = parse_clkgen("0xe00000"), "t8122_clkgen"),
            (|r| r.clkgen = parse_clkgen("e5"), "t8122_clkgen"),
            (|r| r.sgx_setup = parse_sgx_setup("2"), "t8122_sgx_setup"),
            (|r| r.unit_mask_a = parse_number("0x700000007"), "t8122_unit_mask_a"),
            (|r| r.unit_mask_a = 0, "t8122_unit_mask_a"),
            (|r| r.unit_mask_b = parse_number("0xf"), "t8122_unit_mask_b"),
            (|r| r.unit_mask_b = 0, "t8122_unit_mask_b"),
            (|r| r.pstate_cap = 0, "t8122_pstate_cap"),
            (|r| r.pstate_cap = 6, "t8122_pstate_cap"),
            (|r| r.pstate_cap = 16, "t8122_pstate_cap"),
            (|r| r.pstate_cap = parse_number("-1"), "t8122_pstate_cap"),
        ];
        for (edit, name) in cases {
            assert_eq!(resolve(&with(edit)).unwrap_err().name, name);
        }
    }

    #[test]
    fn pstate_cap_accepts_one_to_five_only() {
        for cap in 1..=PSTATE_CAP_MAX {
            assert_eq!(resolve(&with(|r| r.pstate_cap = u64::from(cap))).unwrap().pstate_cap, cap);
        }
        for cap in [0u64, 6, 8, 15, 16] {
            assert!(resolve(&with(|r| r.pstate_cap = cap)).is_err(), "cap {cap}");
        }
        assert_eq!(PSTATE_CAP_MAX, 5);
    }

    #[test]
    fn omitted_cap_allows_the_table_only_with_thermal_control() {
        let v = resolve(&with(|_| {})).unwrap();
        let controlled = v.pstate_limits(true);
        assert_eq!(controlled.ceiling, None);
        assert_eq!(controlled.safe, 2);
        let fixed = v.pstate_limits(false);
        assert_eq!(fixed.ceiling, Some(2));
        assert_eq!(fixed.safe, 2);
    }

    #[test]
    fn explicit_caps_preserve_their_ceiling_and_startup_cap() {
        for cap in 1..=PSTATE_CAP_MAX {
            let v = resolve(&with(|r| r.pstate_cap = u64::from(cap))).unwrap();
            assert!(v.pstate_cap_explicit);
            for thermal_enabled in [false, true] {
                let limits = v.pstate_limits(thermal_enabled);
                assert_eq!(limits.ceiling, Some(cap));
                assert_eq!(limits.safe, cap);
            }
        }
    }

    #[test]
    fn a_leading_sign_is_rejected() {
        for text in ["+1", "-1", "0x+1", "0x-1", "+0x10"] {
            assert_eq!(parse_number(text), INVALID, "{text:?}");
        }
        assert!(resolve(&with(|r| r.pstate_cap = parse_number("+3"))).is_err());
    }

    #[test]
    fn reserved_numbers_and_garbage_parse_as_invalid() {
        for text in ["", "0x", "x1", "18446744073709551615", "18446744073709551614", "0x1g"] {
            assert_eq!(parse_number(text), INVALID, "{text:?}");
        }
        assert_eq!(parse_number("18446744073709551613"), INVALID - 1);
        assert_eq!(parse_number(" 0X10 "), 16);
    }

    #[test]
    fn default_masks_are_within_the_limits() {
        assert_eq!(UNIT_MASK_A & !UNIT_MASK_A_LIMIT, 0);
        assert_eq!(u64::from(UNIT_MASK_B) & !UNIT_MASK_B_LIMIT, 0);
        assert_eq!(UNIT_MASK_A_LIMIT, 0x7_0000_0003);
        assert_eq!(UNIT_MASK_B_LIMIT, 0x7);
    }
}
