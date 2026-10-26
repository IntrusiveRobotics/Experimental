# Experimental Firmware Changes — High-Frequency PWM (FW 7.00 port)

Ported 2026-09-02 from `vesc-firmware-bldc-6.05-experimental` (v1–v7 there; see that
tree's EXPERIMENTAL_CHANGES.md for full history and rationale). Base tree:
`vesc-firmware-bldc-7.00`, whose hwconf targets the NEWER board (`IR_FW3_11`):
**VIN_R1 = 68.1k** (rev-2+ divider) and **NTC beta = 4067 K (bench-confirmed
2026-08-31)** were kept from the 7.00 baseline — they are newer facts than the
6.05-experimental tree's 43.2k / 3380 values, which apply to the older board.

## Ported changes

`hwconf/intrusive_robotics/hw_vesc_gan_core.h` / `.c`:
- `HW_HAS_PHASE_SHUNTS` (shunts on phase outputs; enables V0_V7 sampling)
- PD2 rewired from mislabeled phase filters to `CURRENT_FILTER_ON()/OFF()`
  (high = filter in; ON at boot; HFI toggles it). `HW_HAS_PHASE_FILTERS` removed.
- `MCCONF_FOC_F_ZV` 150000 (75 kHz per-FET), `MCCONF_FOC_CONTROL_SAMPLE_MODE` V0_V7
  (replaces the dead `MCCONF_FOC_SAMPLE_V0_V7` define)
- `HW_LIM_FOC_CTRL_LOOP_FREQ` 5000–150000 (240000 in the 240K build variant)
- `FOC_LOOP_RATE_STOCK_MAX_HZ` 31000 / `FOC_LOOP_RATE_TARGET_HZ` 25500
- Baked sensorless startup defaults (openloop 1800 ERPM, rpm-low 0.1, lock 0.1 s,
  openloop time 0.1 s, boost 10 A, start-curr-dec 0.05, temp comp false)
- Fixed doubled `/*` in the license header

`motor/mcpwm_foc.c`:
- **Adaptive loop divider**, adapted to 7.00's ISR structure: the skip counter in
  7.00 sits AFTER the v0/v7 early-return, so it divides loop candidates directly —
  the 6.05 v0/v7 parity trap does not exist here and dividers need not be odd.
  Rate computed from live TIM1->ARR (immune to the config/timer transition race
  that watchdog-reset 6.05 pre-fix). V0_V7 dividers: 30k→1, 60k→3, 120k→5,
  **150k→6 (25 kHz loop)**, 240k→10 (24 kHz loop). Skip compare is `>=`.
  Stock `FOC_CONTROL_LOOP_FREQ_DIVIDER` path retained as fallback when the
  adaptive defines are absent.
- `m_foc_loop_div_now` published; HFI made divider-aware at 4 sites (V4/V5 and
  V2/V3 `foc_hfi_adjust_angle` conversions, classic-HFI FFT buffer, `dt_sw`
  phase-lag comp). Bit-identical to stock at divider 1.
- `SHUNT_PICK_THR`: fixed 900 ticks → `tim->ARR / 6` (933 at stock ≈ unchanged;
  187 at 150k). 7.00's refactored full-Clarke/reconstruction selection had the
  same absolute-ticks bug as 6.05's 500-tick thresholds (fixed there in v6).

## Builds

| Build | Default f_zv | Per-FET | Loop | HW_NAME | VIN_R1 |
|---|---|---|---|---|---|
| `builds/IR_Exp7_150K` | 150 k | 75 k | 25 kHz | IR_Exp7_150K | 68.1k |
| `builds/IR_Exp7_240K` | 240 k | 120 k | 24 kHz | IR_Exp7_240K | 68.1k |

Versioned snapshots (numbering continues the 6.05-experimental series):
- `builds/versions/v8_2026-09-02_port-to-7.00/` — initial port.
- `builds/versions/v10_2026-09-08_dtcomp-0p010us/` — `MCCONF_FOC_DT_US` default
  0.010 µs baked (was mcconf_default's 0.12, a silicon-FET number; the GaN stage
  slews the SW node during its ~42 ns dead time so the effective compensation
  needed is small — bench-chosen value).
- `builds/versions/v9_2026-09-08_erpm-300k-600k/` — default speed limit raised to
  ±300k eRPM (`MCCONF_L_RPM_MAX/MIN`; stock default is 100k) and `HW_LIM_ERPM`
  to ±600k so the tool accepts up to 600k. CAUTION: 600k eRPM = 85,700 mech RPM
  at 7 pole pairs, and sensorless tracking realistically tops out ~100–150k eRPM
  (~10 observer samples per electrical cycle at a 25 kHz loop) — the raised limit
  is headroom, and the speed limit no longer protects the prop/rotor mechanically. 240 kHz still requires the LispBM
procedure (tool UI caps f_zv at 150k): `(conf-set 'foc-f-zv 240000)` + `(conf-store)`.

## Not yet done on 7.00

- No bench validation at all (6.05 v1–v6 findings were bench-driven; v7/v8 are not).
- Pair with VESC Tool 7.00+ (older tools connect in limited mode).
- HFI still requires a salient motor (test motor measured ld_lq_diff = 0).

v11 (2026-09-08): FW3_12 "Bench Test_003" sensorless profile baked into defaults —
same change as 6.05-experimental v11 (see that tree's changelog for details).
Snapshot: builds/versions/v11_2026-09-08_fw3-12-sensorless/.
