# Intrusive Robotics Delta — Experimental Firmware

Experimental firmware builds for the **Intrusive Robotics Delta**, a compact
high-power GaN motor controller that is **compatible with VESC® software**.

Fork of the VESC firmware project ([`vedderb/bldc`](https://github.com/vedderb/bldc)),
licensed under the **GNU General Public License v3.0** (see [`LICENSE`](LICENSE)).

> **Not affiliated with or endorsed by Benjamin Vedder or the VESC project.**
> "VESC" is a trademark of Benjamin Vedder. This product is *compatible with*
> VESC software; it is not a VESC-branded product.

---

## ⚠️ These builds are experimental

They run the GaN power stage far outside stock VESC parameters and are **not
validated for production use**. They can damage hardware. Do not run one on a
motor or battery you are not prepared to lose, and scope the switch nodes before
committing to any new configuration.

For the stable, released Delta firmware, see the
[Delta repository](https://github.com/IntrusiveRobotics/Delta).

---

## What is experimental here

Stock VESC runs the FOC control loop once per ADC interrupt, which caps the PWM
frequency at whatever the F405 can service. These builds raise the zero-vector rate
(`MCCONF_FOC_F_ZV`) well beyond stock and decouple the control loop from it using
`FOC_CONTROL_LOOP_FREQ_DIVIDER`, so the power stage switches fast while the loop
stays inside the CPU budget.

Full technical detail, including the adaptive-divider scheme and the frequency
semantics that cause most of the confusion in this area, is in
[`EXPERIMENTAL_CHANGES.md`](EXPERIMENTAL_CHANGES.md).

---

## ⚠️ Voltage divider — check your board first

Two board populations exist with different voltage-divider top legs
(R3/R4/R7/R15), and the firmware must match:

| Variant | `VIN_R1` | Ratio | Full scale |
|---|---|---|---|
| plain (e.g. `IR_Exp_150k`) | 43.2 kΩ | 20.64 | 68.1 V |
| `_2` (e.g. `IR_Exp_150K_2`) | 68.1 kΩ | 31.95 | 105.4 V |

The divider is shared by the VBAT sense **and** all three phase-voltage senses, so
a mismatch scales every voltage reading by 1.55× in one direction or the other.
**Measure the top-leg resistors on your actual board before flashing, and verify
against a bench supply afterwards.**

---

## Building

Requires the `arm-none-eabi-gcc` toolchain (GCC 7.x, matching upstream VESC) and
GNU Make:

```bash
make fw_vesc_gan
```

Artifacts land in `build/vesc_gan/`.

> The Makefile does not tolerate spaces in the source path. If your checkout path
> contains spaces, build from a path without them.

Variants differ only in `hwconf/intrusive_robotics/hw_vesc_gan_core.h` — edit
`HW_NAME` and `VIN_R1`, then rebuild.

---

## Flashing

Flash over SWD with an ST-Link and [`stlink`](https://github.com/stlink-org/stlink):

```bash
st-flash --reset write vesc_gan.bin 0x08000000
```

On a fresh chip, flash the generic VESC bootloader to `0x080E0000` first.
