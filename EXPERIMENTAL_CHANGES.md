# Experimental Firmware Changes — High-Frequency PWM (FW 6.05)

Changes live in `hwconf/intrusive_robotics/hw_vesc_gan_core.h` plus one small runtime
mechanism in `motor/mcpwm_foc.c` (detection-safe divider, described below). Goal: run the GaN power stage (LMG2100R026) at far higher PWM frequency than
stock VESC while keeping the FOC control loop inside the F405's CPU budget, using
`FOC_CONTROL_LOOP_FREQ_DIVIDER` (originally added by Marcos Chaparro for the Axiom,
paltatech/bldc commit 9652231, for "PWM running at 100+kHz").

Last updated 2026-08-31.

## Version history & build snapshots

Each change now gets a versioned snapshot under `builds/versions/vN_<date>_<tag>/`
containing `vesc_gan.bin` + `.elf` for every variant. The four top-level `builds/`
folders always hold the LATEST full artifact set (bin/elf/hex/dmp/list).

| Ver | Date | Change |
|---|---|---|
| v1 | 2026-08-25 | Fixed-divider builds (IR_Exp_60k div 3, IR_Exp_120k div 5) |
| v2 | 2026-08-25 | 150k/240k builds; adaptive divider; ARR-based transition fix; 30k stock region |
| v3 | 2026-08-27 | Voltage divider fix (VIN_R1 68.1k→43.2k); baked sensorless startup defaults |
| v4 | 2026-08-31 | `_2` variants (VIN_R1=68.1k) for the 68.1k-divider board population |
| v5 | 2026-08-31 | HFI + silent HFI divider compatibility fix (first versioned snapshot) |
| v6 | 2026-08-31 | Shunt-selection margin scaled with ARR (`mcpwm_foc.c` LONGEST_ZERO branch): upstream's fixed 500-tick "low modulation" threshold is ~9% of the period at stock 30k but ~45% at 150k, so duty swings crossed it and the shunt-pair selection flipped constantly — step discontinuities visible as spikes in the MC-total current. Now `ARR/11` (509 at stock = unchanged; 101 at 150k). Diagnosed from bench capture: clean phase currents but spiky "Total current filtered by MC" at 150k/12% duty. |

## Frequency semantics (source of endless confusion — see vedderb/bldc issue #212)

- `foc_f_zv` = **zero-vector rate**. Each FET/phase node switches at **f_zv / 2**;
  motor current ripple is at f_zv; the ADC ISR fires at f_zv.
- The FOC loop runs every `FOC_CONTROL_LOOP_FREQ_DIVIDER`-th ADC ISR:
  loop rate = f_zv / divider (V0_V7 sampling) or f_zv / (2·divider) (V0-only, odd divider).
- F405 loop-rate wall: RTOS crashes near a 38 kHz loop (26 µs ISR, Axiom data);
  we target 20–27 kHz.

## Changes vs. stock hwconf

| Define | Stock / previous | Now | Why |
|---|---|---|---|
| `HW_NAME` | stock Delta name | `Exper240k` | Distinguish experimental build in VESC Tool |
| `HW_HAS_PHASE_SHUNTS` | absent | defined | Shunts are physically on phase outputs (INA241, −5…110 V CMR); unlocks V0_V7 sampling where current is valid in both zero vectors |
| `MCCONF_FOC_CONTROL_SAMPLE_MODE` | (dead `MCCONF_FOC_SAMPLE_V0_V7 false` — ignored by 6.05) | `FOC_CONTROL_SAMPLE_MODE_V0_V7` | Every ADC ISR is a loop candidate → loop = f_zv/divider with exact dt |
| `MCCONF_FOC_F_ZV` | 30000 | 150000 / 240000 (per build) | 75 / 120 kHz per-FET switching |
| `FOC_LOOP_RATE_STOCK_MAX_HZ` | — (new) | 31000 | Undivided-loop rates up to this run with divider 1 (stock: f_zv ≤ 30k in V0_V7, incl. all detections) |
| `FOC_LOOP_RATE_TARGET_HZ` | — (new) | 25500 | **Adaptive divider** ceiling above the stock region: smallest odd divider keeping the loop ≤ 25.5 kHz (see below) |
| `HW_LIM_FOC_CTRL_LOOP_FREQ` | 3000–30000 (hw.h default) | 5000–150000 / 240000 (per build) | The clamp in `commands_apply_mcconf_hw_limits()` knows nothing about the divider; it must admit the raw f_zv |
| `VIN_R1` | 68100 | **43200** or **68100** (per variant) | Two board populations exist with different divider top legs (R3/R4/R7/R15). The `_2` builds use 68.1k (ratio 31.95, full scale 105.4 V); the plain builds use 43.2k (ratio 20.64, full scale 68.1 V). Mismatching firmware to board scales every voltage reading by 1.55x. **Measure your board before flashing.** |
| `MCCONF_FOC_OPENLOOP_RPM` etc. | FW defaults | bench-tuned | Baked sensorless startup defaults 2026-08-27: openloop 1800 ERPM, rpm-low 0.1, lock 0.1 s, openloop time 0.1 s, boost 10 A, start-curr-dec 0.05. Temp comp deliberately baked FALSE (motor temp sensor is disabled on this board). |
| (cleanup) | doubled `/*` in header | fixed | clang warning |

## Build lineage

| Build | Default f_zv | Per-FET | Divider | Loop | HW_NAME |
|---|---|---|---|---|---|
| `builds/IR_Exp_60k` | 60 k | 30 k | fixed 3 | 20 kHz (V0_V7) | stock |
| `builds/IR_Exp_120k` | 120 k | 60 k | fixed 5 | 24 kHz | stock |
| `builds/IR_Exp_150k` | 150 k | 75 k | **adaptive** | ≤24 kHz | **Exper150k** |
| `builds/IR_Exp_240k` | 240 k | 120 k | **adaptive** | ≤24 kHz | **Exper240k** |
| `builds/IR_Exp_150K_2` | 150 k | 75 k | **adaptive** | ≤24 kHz | **IR_Exp_150K_2** (VIN_R1=68.1k variant) |
| `builds/IR_Exp_240K_2` | 240 k | 120 k | **adaptive** | ≤24 kHz | **IR_Exp_240K_2** (VIN_R1=68.1k variant) |

The `_2` variants (2026-08-31) differ from the base builds ONLY in `VIN_R1` = 68.1k
(divider ratio 31.95, full-scale 105.4 V) — for boards whose divider top legs are
fitted with 68.1k. The source tree itself keeps the schematic-confirmed 43.2k.

Since 2026-08-25 the 150k/240k builds use the **adaptive divider** (`motor/mcpwm_foc.c`,
ADC ISR). The undivided loop rate is derived from the **live timer period (TIM1->ARR)**,
never from the configured f_zv. Deriving it from the timer makes switching-frequency
changes safe by construction: the ARR the ISR reads is always the period it is actually
running at, so there is no window in which the loop rate and the hardware disagree.

*Why it is done this way:* on a switching-frequency change the config updates before the
timer does. A pre-release revision that computed the rate from the config could briefly
run the full FOC loop at the old, higher ISR rate during that window. The ARR-based
computation removes the race structurally rather than papering over it, and the
transition has been clean since.

Divider selection: undivided-loop rates ≤ `FOC_LOOP_RATE_STOCK_MAX_HZ` (31 kHz) run with
**divider 1** — stock behavior for f_zv ≤ 30k in V0_V7, which covers every detection
routine (confirmed working on the bench) and the stock 30 kHz default. Above that, the
smallest ODD divider keeping the loop ≤ `FOC_LOOP_RATE_TARGET_HZ` (25.5 kHz). Resulting
dividers (V0_V7): 30k→1 (30 kHz loop), 60k→3, 120k→5, 150k→7, 240k→11; V0-only modes get
the additional /2 accounted for. Any f_zv set in the tool is safe — no per-frequency
builds needed anymore. The two remaining builds differ only in HW_NAME, default f_zv,
and the HW f_zv limit: `IR_Exp_150k` matches VESC Tool's 150 kHz UI maximum (fully
tool-native), while `IR_Exp_240k` admits 240 kHz, reachable only via the LispBM
procedure below. The dt passed to observers/controllers scales with the live divider in
the same ISR pass.

Each folder has a README with its exact config and bench checklist.

## Hard limits at 240 kHz (why this is the ceiling)

- **ADC:** trigger period 4.17 µs vs 3.86 µs for the 6-rank regular sequence
  (6 × 27 cycles at 42 MHz ADCCLK, `ADC_Prescaler_Div2`). 310 ns deterministic margin.
  Adding any ADC conversion or lengthening any sample time overruns the trigger.
- **Timer:** ARR = 700 counts → ~0.14% duty resolution; 40 ns dead time = 7 ticks.
- **CPU:** skip-only ISR entries at 240 kHz cost ≈7–12%; loop ≈40–55%.
- **VESC Tool:** UI caps f_zv at 150 kHz in every tool version. 240 kHz can only be set via
  LispBM: `(conf-set 'foc-f-zv 240000)` + `(conf-store)` (this path does apply the HW-limit
  clamp — verified — and passes with the raised limit). **Any config write from the tool UI
  clamps f_zv back to ≤150 kHz**; re-apply and check with `(conf-get 'foc-f-zv)`.

## Known pitfalls (verified in source, `motor/mcpwm_foc.c`)

1. **Even dividers are broken in V0-only sampling.** The skip counter (line 2817) runs
   before the v0/v7 early-return; an even divider locks onto one parity — either dt is 2×
   wrong, or the loop never runs and the THREAD_MCPWM watchdog faults. Odd dividers are
   safe in every mode. V0_V7 sampling accepts any divider (kept odd anyway).
2. **dt scaling** (`dt *= FOC_CONTROL_LOOP_FREQ_DIVIDER`, line 3089) exists in 6.05 but is
   marked "TODO: Test this" upstream. The original Paltatech commit lacked it — that is
   what "motor malfunction with the divider" in issue #212 was about.
3. **HFI divider incompatibility — FIXED in v5 (2026-08-31).** All HFI variants
   (classic HFI/HFI_START and silent HFI V2–V5) assumed the di sample interval was
   1/f_zv. With the loop divided, samples are `divider` zero vectors apart. Fix in
   `motor/mcpwm_foc.c`: the ISR publishes its live divider (`m_foc_loop_div_now`),
   and four sites now scale by it — the V4/V5 and V2/V3 `foc_hfi_adjust_angle`
   conversions, the classic-HFI FFT buffer fill, and the `dt_sw` phase-lag
   compensation in `hfi_update`. `foc_math.c` needed no changes (pure inductance
   terms; its dt argument was already divider-scaled). At f_zv ≤ 30k (divider 1)
   all HFI behavior is bit-identical to stock. NOT yet bench-validated, and the
   saliency prerequisite stands: the current test motor measured ld_lq_diff = 0,
   so validate HFI on a salient motor. Injection tuning note: with divider N the
   flux excursion per injection half-cycle is N× larger for the same hfi_voltage —
   expect to LOWER the HFI voltages at 150k/240k relative to 30k values.
4. **Detection wizards** internally force f_zv to 10–30 kHz, which under a fixed divider
   meant a sub-kHz loop and failing detections. FIXED 2026-08-25 by the **adaptive
   divider** (`motor/mcpwm_foc.c`, ISR top + dt scaling): divider 1 below 35 kHz gives
   detections exactly stock behavior; above that the divider is computed from f_zv and
   the sample mode to hold the loop at ≤24 kHz. The skip compare also changed `==` →
   `>=` so a runtime divider decrease can't strand the counter and stall the loop.
   Applies to the current IR_Exp_150k / IR_Exp_240k binaries (60k/120k predate it).
5. **Config persistence:** the mcconf signature doesn't change between these builds, so the
   stored config survives reflash — new `MCCONF_*` defaults do NOT auto-apply. Set values
   explicitly (or load firmware defaults) after flashing.
6. **Cosmetic divider blind spots:** VESC Tool sampled-data timebase and the
   sampling-frequency getters ignore the divider (11× off); tone/beep pitch may be off.
7. **Scope aliasing:** at low sample rates a 60 kHz node reads as 40 kHz (100 kS/s alias).
   Always measure edge-to-edge at ≥10 MS/s. Expect 8.33 µs per phase at f_zv=240k.

## Bench essentials

1. Flash `builds/IR_Exp_240k/vesc_gan.bin` to 0x08000000; set f_zv via the Lisp snippet.
2. Watch `mcpwm_foc_get_last_adc_isr_duration` (RT panel): loop period is 45.8 µs,
   stay under ~35 µs. Over that → f_zv 200k or divider 13, rebuild.
3. Thermal: per-FET switching losses at 120 kHz ≈ 4× the 30 kHz baseline — first real
   test of the GaN thermal design.
