/*
     Copyright 2016 Benjamin Vedder    benjamin@vedder.se

     This file is part of the VESC firmware.

     The VESC firmware is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation, either version 3 of the License, or
     (at your option) any later version.

     The VESC firmware is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.

	Hardware configuration for the Intrusive Robotics VESC GaN ESC:
	  - STM32F405RGT6, 8 MHz HSE
	  - 3x LMG2100R026 GaN half-bridges (TTL PWM, integrated gate driver)
	  - 3x INA241A2 external current-sense amps, 0.5 mOhm shunts, 20 V/V
	  - Three-shunt sensing on PC0/PC1/PC2 (ADC1_IN10 / ADC2_IN11 / ADC3_IN12)
	  - 6S-12S LiPo (22-50.4 V), 60 A continuous target
	  - Sensorless FOC, no Hall sensors, no IMU, no SPI gate driver

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef HW_VESC_GAN_CORE_H_
#define HW_VESC_GAN_CORE_H_

#define HW_NAME					"IR_Exp_150K_2"

// HW properties
#define HW_HAS_3_SHUNTS
// Shunts sit on the phase outputs (SW node -> motor pad), not the low-side legs,
// so current samples are valid in both zero vectors — required for the V0_V7
// control sample modes.
#define HW_HAS_PHASE_SHUNTS
// INA241 with REF1=VS, REF2=GND outputs (VS/2) + (gain * Rshunt * I), so positive
// motor current produces a rising ADC reading. Do NOT define INVERTED_SHUNT_POLARITY.

// PD2 drives the analog switches on the CURRENT-sense RC filters — matching the
// official VESC 6 reference, which also puts CURRENT_FILTER on PD2. This was
// previously mislabeled as phase-voltage filters (HW_HAS_PHASE_FILTERS): the
// board has no switchable phase filters, so that define is gone and the
// phase-filter observer math is disabled. High = switches closed = filter in.
// mcpwm_foc drives this automatically: OFF while HFI runs (it needs the raw
// current derivative), ON in all other running states.
#define CURRENT_FILTER_GPIO		GPIOD
#define CURRENT_FILTER_PIN		2
#define CURRENT_FILTER_ON()		palSetPad(CURRENT_FILTER_GPIO, CURRENT_FILTER_PIN)
#define CURRENT_FILTER_OFF()	palClearPad(CURRENT_FILTER_GPIO, CURRENT_FILTER_PIN)

// LEDs -- TODO: confirm against schematic
#define LED_GREEN_GPIO			GPIOB
#define LED_GREEN_PIN			1
#define LED_RED_GPIO			GPIOB
#define LED_RED_PIN				0

// LEDs are active-LOW: anode to 3_3VCC, cathode returns to the GPIO through
// R9/R10, so the pin must sink current (drive LOW) to light the LED.
#define LED_GREEN_ON()			palClearPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_GREEN_OFF()			palSetPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_RED_ON()			palClearPad(LED_RED_GPIO, LED_RED_PIN)
#define LED_RED_OFF()			palSetPad(LED_RED_GPIO, LED_RED_PIN)

/*
 * ADC Vector
 *
 * 0  (1):	IN10	CURR1     (CS_U, PC0)
 * 1  (2):	IN11	CURR2     (CS_V, PC1)
 * 2  (3):	IN12	CURR3     (CS_W, PC2)
 * 3  (1):	IN0		SENS1     (phase U voltage, PA0)
 * 4  (2):	IN1		SENS2     (phase V voltage, PA1)
 * 5  (3):	IN2		SENS3     (phase W voltage, PA2)
 * 6  (1):	IN5		ADC_EXT1  (PA5 -- aux/throttle)
 * 7  (2):	IN6		ADC_EXT2  (PA6 -- aux)
 * 8  (3):	IN3		TEMP_MOS  (board NTC, PA3)
 * 9  (1):	IN14	TEMP_MOTOR (PC4)
 * 10 (2):	IN15	UNUSED    (PC5)
 * 11 (3):	IN13	VIN_SENS  (VBAT divider, PC3)
 * 12 (1):	Vrefint
 * 13 (2):	IN0		(PA0 dup)
 * 14 (3):	IN1		(PA1 dup)
 * 15 (1):	IN8		(PB0)
 * 16 (2):	IN9		(PB1)
 * 17 (3):	IN2		(PA2 dup)
 */

#define HW_ADC_CHANNELS			18
#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			6

// ADC Indexes
#define ADC_IND_SENS1			3
#define ADC_IND_SENS2			4
#define ADC_IND_SENS3			5
#define ADC_IND_CURR1			0
#define ADC_IND_CURR2			1
#define ADC_IND_CURR3			2
#define ADC_IND_VIN_SENS		11
#define ADC_IND_EXT				6
#define ADC_IND_EXT2			7
#define ADC_IND_TEMP_MOS		8
#define ADC_IND_TEMP_MOTOR		9
#define ADC_IND_VREFINT			12

// Component parameters
#ifndef V_REG
#define V_REG					3.3
#endif
// Divider ratio for BOTH the VBAT sense and the three phase-voltage senses -- the
// board fits four identical dividers (R3/R4/R7/R15 top, R2/R5/R8/R16 bottom), and
// ADC_VOLTS_PH_FACTOR is left at its 1.0 default, so mcpwm_foc scales phase voltage
// with these same constants. Changing VIN_R1 moves both readings together.
// THIS IS THE 68.1k VARIANT. Two board populations exist and they are NOT
// interchangeable:
//
//   VIN_R1 = 43200.0  ->  ratio 20.64, full scale 68.1 V
//   VIN_R1 = 68100.0  ->  ratio 31.95, full scale 105.4 V   <-- this build
//
// Flashing the wrong one scales every voltage reading (VBAT and all three phases)
// by 1.55x in one direction or the other. Measure the top-leg resistors
// (R3/R4/R7/R15) on your actual board before flashing, and verify against a bench
// supply afterwards. HW_LIM_VIN's 93 V is a separate limit and is not the
// measurement ceiling.
#ifndef VIN_R1
#define VIN_R1					68100.0
#endif
#ifndef VIN_R2
#define VIN_R2					2200.0
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		20.0	// INA241A2
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		0.0005	// 0.5 mOhm
#endif

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// NTC -- 10k @ 25C, beta = 3380 K (board thermistor). This is the DATASHEET beta of
// the intended part (NTCG164BH103FT1S); the schematic's NTCG103JF103FT1 is 3435 K,
// a 1.4 C difference at 115 C -- immaterial either way.
//
// HISTORY / OPEN ITEM (2_12): this was 4067 K from 2_11 back to the 2026-06-17 edit.
// That 4067 came from a bench fit, not a datasheet: at beta=3380 a reference "true
// 85 C" read ~100 C, and solving back gives ~4067 K. Reverted to 3380 deliberately.
// Note the fit's sign is NOT explained by an NTC-to-FET thermal gradient -- the NTC
// is downstream of the FETs and would read LOW, not high -- so the real part may
// genuinely be ~4067 K. If so, 3380 OVER-reports (~+15 C at 85 C) and the
// MCCONF_L_LIM_TEMP_FET_START/END limits below trip early (95 C fires at ~81 C
// actual, 115 C at ~96 C). Fail-safe, but it costs thermal headroom.
// RESOLVE with a two-point soak measured AT THE NTC (not the FET): ambient plus one
// hot point, solve beta directly, then handle any real gradient by adjusting the
// FET temp limits rather than by bending beta.
// Board NTC is wired HIGH-side: 3.3V -> NTC -> ADC node (PA3) -> 10k -> GND.
// Heating drops R_ntc, which RAISES the ADC count, so resistance recovery is
// R = 10k * (4095/adc - 1)  (note '*', not '/'). The low-side form read
// temperature backwards (reported temp fell as the board heated).
#define NTC_RES(adc_val)		(10000.0 * ((4095.0 / (float)adc_val) - 1.0))
#define NTC_TEMP(adc_ind)		(1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)

#define NTC_RES_MOTOR(adc_val)	(10000.0 / ((4095.0 / (float)adc_val) - 1.0))
#define NTC_TEMP_MOTOR(beta)	(1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)

// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4095.0 * V_REG)

// COMM-port ADC GPIOs
#define HW_ADC_EXT_GPIO			GPIOA
#define HW_ADC_EXT_PIN			5
#define HW_ADC_EXT2_GPIO		GPIOA
#define HW_ADC_EXT2_PIN			6

// UART -- TODO: confirm UART3 PB10/PB11 against MCU.kicad_sch
#define HW_UART_DEV				SD3
#define HW_UART_GPIO_AF			GPIO_AF_USART3
#define HW_UART_TX_PORT			GPIOB
#define HW_UART_TX_PIN			10
#define HW_UART_RX_PORT			GPIOB
#define HW_UART_RX_PIN			11

// ICU (servo / PPM input) -- TODO: confirm pin
#define HW_USE_SERVO_TIM4
#define HW_ICU_TIMER			TIM4
#define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV				ICUD4
#define HW_ICU_CHANNEL			ICU_CHANNEL_1
#define HW_ICU_GPIO_AF			GPIO_AF_TIM4
#define HW_ICU_GPIO				GPIOB
#define HW_ICU_PIN				6

// I2C (defined for completeness; design has no I2C peripherals)
#define HW_I2C_DEV				I2CD2
#define HW_I2C_GPIO_AF			GPIO_AF_I2C2
#define HW_I2C_SCL_PORT			GPIOB
#define HW_I2C_SCL_PIN			10
#define HW_I2C_SDA_PORT			GPIOB
#define HW_I2C_SDA_PIN			11

// Hall / encoder pins (sensorless FOC, but firmware references them)
#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		8
#define HW_ENC_TIM				TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_CH			EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC		EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler

// SPI pins (unused; defined so SPI macros in hw.h resolve)
#define HW_SPI_DEV				SPID1
#define HW_SPI_GPIO_AF			GPIO_AF_SPI1
#define HW_SPI_PORT_NSS			GPIOB
#define HW_SPI_PIN_NSS			11
#define HW_SPI_PORT_SCK			GPIOA
#define HW_SPI_PIN_SCK			5
#define HW_SPI_PORT_MOSI		GPIOB
#define HW_SPI_PIN_MOSI			2
#define HW_SPI_PORT_MISO		GPIOA
#define HW_SPI_PIN_MISO			6

// Phase voltage measurement
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// GaN dead time. LMG2100R026 has no body diode, so any dead-time conduction
// flows through the GaN reverse-channel at ~2-3 V drop -- expensive at 60 A.
// Bring up at 100 ns, scope switch nodes for shoot-through, then ratchet down
// toward 40 ns (per CLAUDE.md target).
#define HW_DEAD_TIME_NSEC		40.0

// Default mc_configuration overrides for 6S-12S / 60 A target
#ifndef MCCONF_L_MIN_VOLTAGE
#define MCCONF_L_MIN_VOLTAGE			9.0
#endif
#ifndef MCCONF_L_MAX_VOLTAGE
#define MCCONF_L_MAX_VOLTAGE			80.0	// 12S charged = 50.4 V
#endif
#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC
#endif
// 150 kHz zero-vector rate: motor sees 150 kHz ripple, each FET PWMs at 75 kHz.
// Chosen as the highest f_zv that VESC Tool's UI supports natively (its cap is
// 150 kHz), so the value displays and edits normally — no LispBM workaround.
// With FOC_CONTROL_LOOP_FREQ_DIVIDER 7 and V0_V7 sampling the FOC loop runs at 21.4 kHz.
// ARR = 168 MHz / 150 kHz = 1120 counts (~0.09% duty resolution, dead time = 7 ticks).
// ADC: trigger period 6.67 us vs 3.86 us for the 6-rank regular sequence at 42 MHz
// ADCCLK (Div2) — comfortable margin (240 kHz, with 310 ns margin, is the hard ceiling).
#ifndef MCCONF_FOC_F_ZV
#define MCCONF_FOC_F_ZV					150000.0
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		80.0
#endif
// Sample in both V0 and V7 (phase shunts make both valid). Every ADC ISR entry is
// then a control-loop candidate, so any FOC_CONTROL_LOOP_FREQ_DIVIDER value gives
// loop rate = f_zv / divider with correct dt. (In V0-only mode the divider must be
// ODD — the skip counter in mcpwm_foc_adc_int_handler runs before the v0/v7 check,
// and an even divider locks onto one parity: dt 2x wrong, or loop never runs.)
#ifndef MCCONF_FOC_CONTROL_SAMPLE_MODE
#define MCCONF_FOC_CONTROL_SAMPLE_MODE	FOC_CONTROL_SAMPLE_MODE_V0_V7
#endif
// Sensorless / open-loop start settings from a bench-proven profile (2026-09-08)
// for a smooth default start. This REMOVES the earlier aggressive-start bakes
// (openloop 1800 ERPM, 10 A boost, start-current-decrease 0.05, lock 0.1 s) —
// starting is smoother on stock values for those. Only deviations from
// mcconf_default.h are listed.
#ifndef MCCONF_FOC_SL_ERPM
#define MCCONF_FOC_SL_ERPM				4000.0	// Later handoff to the observer
#endif
#ifndef MCCONF_FOC_SL_OPENLOOP_TIME
#define MCCONF_FOC_SL_OPENLOOP_TIME		0.1
#endif
#ifndef MCCONF_FOC_SL_OPENLOOP_T_RAMP
#define MCCONF_FOC_SL_OPENLOOP_T_RAMP	0.0
#endif
// MXV observer with lambda compensation
#ifndef MCCONF_FOC_OBSERVER_TYPE
#define MCCONF_FOC_OBSERVER_TYPE		FOC_OBSERVER_MXV_LAMBDA_COMP
#endif
// Bench motor detection results + tuning for ONE SPECIFIC test motor (14-pole,
// ~370 Kv drone motor). These let a fresh config spin that bench motor without a
// wizard run. THEY ARE NOT GENERIC: on any other motor, run FOC detection in VESC
// Tool before applying torque. The current limits below are that motor's detected
// values, not a board rating (the board's absolute ceiling is
// MCCONF_L_MAX_ABS_CURRENT).
#ifndef MCCONF_FOC_MOTOR_R
#define MCCONF_FOC_MOTOR_R				0.0491708
#endif
#ifndef MCCONF_FOC_MOTOR_L
#define MCCONF_FOC_MOTOR_L				2.33379e-05
#endif
#ifndef MCCONF_FOC_MOTOR_LD_LQ_DIFF
#define MCCONF_FOC_MOTOR_LD_LQ_DIFF		6.78116e-06
#endif
#ifndef MCCONF_FOC_MOTOR_FLUX_LINKAGE
#define MCCONF_FOC_MOTOR_FLUX_LINKAGE	0.00215029
#endif
#ifndef MCCONF_FOC_OBSERVER_GAIN
#define MCCONF_FOC_OBSERVER_GAIN		2.16274e+08
#endif
#ifndef MCCONF_FOC_CURRENT_KP
#define MCCONF_FOC_CURRENT_KP			0.0233379
#endif
#ifndef MCCONF_FOC_CURRENT_KI
#define MCCONF_FOC_CURRENT_KI			49.1707
#endif
#ifndef MCCONF_L_CURRENT_MAX
#define MCCONF_L_CURRENT_MAX			26.0367
#endif
#ifndef MCCONF_L_CURRENT_MIN
#define MCCONF_L_CURRENT_MIN			-26.0367
#endif
// Battery-cut defaults for the 6S BENCH PACK (3.40 / 3.00 V per cell). Set your
// own cell count and cutoffs in VESC Tool — on a higher-cell pack these defaults
// sit far below the real pack voltage, so undervoltage protection will not engage.
#ifndef MCCONF_SI_BATTERY_CELLS
#define MCCONF_SI_BATTERY_CELLS			6
#endif
#ifndef MCCONF_L_BATTERY_CUT_START
#define MCCONF_L_BATTERY_CUT_START		20.4
#endif
#ifndef MCCONF_L_BATTERY_CUT_END
#define MCCONF_L_BATTERY_CUT_END		18.0
#endif
// Temp comp deliberately NOT baked as true: the board defaults motor temp sensing
// to TEMP_SENSOR_DISABLED (below), so resistance temp compensation would track a
// dead input.
#ifndef MCCONF_FOC_TEMP_COMP
#define MCCONF_FOC_TEMP_COMP			false
#endif
// Board has NO switchable phase filters (PD2 is the current filter — see above).
// Force the flag false so the tool shows the truth; the phase-filter observer
// math is compiled out anyway (no HW_HAS_PHASE_FILTERS).
#ifndef MCCONF_FOC_PHASE_FILTER_ENABLE
#define MCCONF_FOC_PHASE_FILTER_ENABLE	false
#endif
// If phase filters are enabled, only use them below this ERPM (default 4000).
#ifndef MCCONF_FOC_PHASE_FILTER_MAX_ERPM
#define MCCONF_FOC_PHASE_FILTER_MAX_ERPM	500.0
#endif
// No motor thermistor fitted by default — disable motor temp sensing so an open/
// floating TEMP_MOTOR input can't fake a hot reading and derate/shut down the motor.
// Overrides mcconf_default.h's TEMP_SENSOR_NTC_10K_25C. Re-selectable in VESC Tool.
#ifndef MCCONF_M_MOTOR_TEMP_SENS_TYPE
#define MCCONF_M_MOTOR_TEMP_SENS_TYPE	TEMP_SENSOR_DISABLED
#endif
// FET thermal derating: begin limiting at 95 C, full shutdown at 115 C (was 85/100).
// GaN LMG2100R026 is rated to 150 C junction; 115 C board-NTC cutoff keeps margin.
#ifndef MCCONF_L_LIM_TEMP_FET_START
#define MCCONF_L_LIM_TEMP_FET_START		95.0
#endif
#ifndef MCCONF_L_LIM_TEMP_FET_END
#define MCCONF_L_LIM_TEMP_FET_END		115.0
#endif
#ifndef MCCONF_L_IN_CURRENT_MAX
#define MCCONF_L_IN_CURRENT_MAX			60.0
#endif
#ifndef MCCONF_L_IN_CURRENT_MIN
#define MCCONF_L_IN_CURRENT_MIN			-60.0
#endif

// Hardware limits (upper clamps that the user cannot exceed in VESC Tool)
// LMG2100R026 is 53 A continuous / 93 V continuous. Keep healthy margin.
#define HW_LIM_CURRENT			-105.0, 105.0
#define HW_LIM_CURRENT_IN		-105.0, 105.0
#define HW_LIM_CURRENT_ABS		0.0, 165.0
#define HW_LIM_VIN				8.0, 93.0
#define HW_LIM_ERPM				-200e3, 200e3
#define HW_LIM_DUTY_MIN			0.0, 0.05
#define HW_LIM_DUTY_MAX			0.0, 0.99
#define HW_LIM_TEMP_FET			-40.0, 160.0

// ADAPTIVE loop divider (runtime, in mcpwm_foc_adc_int_handler), computed from
// the LIVE timer period so switching-frequency transitions can never briefly run
// the full loop at a stale high ISR rate (that race starved the RTOS and
// watchdog-reset the VESC when lowering f_zv 150k -> 30k).
// Undivided-loop rates up to FOC_LOOP_RATE_STOCK_MAX_HZ run with divider 1 —
// stock behavior: f_zv <= 30k in V0_V7 (including every detection routine, and
// f_zv 30k itself -> 30 kHz loop, Luna-class). Above that, the smallest ODD
// divider keeping the loop at or below FOC_LOOP_RATE_TARGET_HZ. V0_V7 sampling:
//   f_zv 30k -> 1 (30.0 kHz loop)      f_zv 150k -> 7  (21.4 kHz loop)
//   f_zv 60k -> 3 (20.0 kHz loop)      f_zv 240k -> 11 (21.8 kHz loop)
//   f_zv 120k -> 5 (24.0 kHz loop)
// (V0-only modes get the extra /2 accounted for.) Odd dividers are enforced
// because in v0-only sampling an even divider locks the skip counter onto one
// v0/v7 parity (dt 2x wrong, or loop never runs -> WDT fault).
#define FOC_LOOP_RATE_STOCK_MAX_HZ				31000.0
#define FOC_LOOP_RATE_TARGET_HZ					25500.0

// commands_apply_mcconf_hw_limits() clamps f_zv from this limit with NO knowledge
// of FOC_CONTROL_LOOP_FREQ_DIVIDER — it thinks the loop runs at f_zv (V0_V7) or
// f_zv/2 (V0). Upper bound 150k matches VESC Tool's own UI cap, so the full tool
// range is usable. Worst reachable case: V0_V7 at 150k -> 21.4 kHz loop. All
// combinations stay at or below that.
#define HW_LIM_FOC_CTRL_LOOP_FREQ	5000.0, 150000.0

#endif /* HW_VESC_GAN_CORE_H_ */
