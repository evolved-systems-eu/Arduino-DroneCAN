ICEMonitor – Fuel Injection Signal Monitor
==========================================

Monitors a fuel injection signal from a Delphi MT05 ECU, extracts RPM and
injection duty cycle, and broadcasts the results over DroneCAN as
uavcan.equipment.ice.reciprocating.Status.

Additional inputs for throttle position and coolant temperature are optionally
broadcast in the same message. Throttle and coolant temperature each support
three source modes: disabled, ADC pin, or INA228 voltage sensor over I2C.
Two independent analog temperature sensors broadcast via
uavcan.equipment.device.Temperature.


CONFIGURATION REQUIRES REBOOT
------------------------------

Any change to a signal source (THR_SRC, CLT_SRC), pin assignment
(FUELINJ_PIN, THR_PIN, CLT_PIN, TMP0_PIN, TMP1_PIN), INA228 I2C address
(THR_I2C_ADDR, CLT_I2C_ADDR), RPM_MAX, or RPM_K_WIN takes effect only after
a reboot. All pin, source, and bounds initialisation is performed once at
startup.

The following parameters are updated live at 1 Hz without reboot:
  BCAST_MS, RPM_ALPHA, RPM_A1, RPM_A0, RPM_HOLD_MS, RPM_STOP_MS,
  RPM_ACCEL, RPM_K_ACC, DUTY_ALPHA, DUTY_A1, DUTY_A0,
  THR_ALPHA, THR_A1, THR_A0, CLT_ALPHA, CLT_A1, CLT_A0,
  TMP0_ALPHA, TMP0_A1, TMP0_A0, TMP0_BCAST,
  TMP1_ALPHA, TMP1_A1, TMP1_A0, TMP1_BCAST.

All other parameters (sources, pins, addresses, NODEID, BAUD) require reboot.


BROADCAST CONDITION
-------------------

The ICE Status message is broadcast when at least one of the following is
configured (all checked at boot):

  - FUELINJ_PIN >= 0
  - THR_SRC = 2 and THR_I2C_ADDR > 0
  - THR_SRC = 1 and THR_PIN >= 0
  - CLT_SRC = 2 and CLT_I2C_ADDR > 0
  - CLT_SRC = 1 and CLT_PIN >= 0

With factory defaults (CLT_SRC = 2, CLT_I2C_ADDR = 0x40) the message is
broadcast even without FUELINJ_PIN or THR configured.


INPUT SIGNAL (INJECTION)
------------------------

Source : Delphi MT05 ECU fuel injector output
Logic  : Active HIGH = fuel is being injected
           Rising edge  -> injection starts
           Falling edge -> injection ends
Engine : Single cylinder, 4-stroke
           -> 1 injection per 2 crankshaft revolutions

Signal characteristics:
  - Idle RPM range  : ~1000-3000 RPM -> period 40-120 ms
  - Max RPM range   : up to RPM_MAX (default 8000 RPM -> period 15 ms; capped)
  - Pulse width     : typically 2-20 ms depending on load
  - Logic level     : 3.3 V max (see PIN TABLE)


RPM CALCULATION
---------------

The ISR timestamps each rising edge. The period between two consecutive
rising edges equals one full injection cycle = 2 crankshaft revolutions.

    period_us = time between two consecutive rising edges [us]

    RPM = 2 revolutions / period_seconds * 60 s/min
        = 120 / period_seconds
        = 120,000,000 / period_us

Example: period_us = 20,000 us -> RPM = 120,000,000 / 20,000 = 6000 RPM

Only periods within [120,000,000/RPM_MAX us, 400,000 us] are accepted.
With the default RPM_MAX = 8000: [15,000 us, 400,000 us] = 300-8,000 RPM.
Periods outside this range are silently discarded to reject noise glitches.
Signals faster than RPM_MAX are rejected; the EMA holds its last value and
the output is capped at RPM_MAX. RPM_MAX requires reboot to change.

All complete injection cycles committed during the broadcast window are
averaged before being fed into the plausibility filter and EMA:

    mean_period = sum(period_us) / count
    mean_high   = sum(high_us)   / count

Only matched pairs (both rising and falling edge captured for the same cycle)
are included. The accumulators (period_sum, high_sum, count) are reset at each
broadcast. The ISR staging variables (last rising edge timestamp, pending pulse
width) are NOT reset, so a cycle that spans two broadcast windows is still
committed correctly at the next rising edge.


FUEL CUT-OFF AND SIGNAL RELIABILITY
-------------------------------------

The injection signal is absent during fuel cut-off (overspeed protection,
deceleration fuel cut) even though the engine is still spinning. A single
stale timeout would incorrectly report STATE_STOPPED in this situation.

Two-tier stale detection (RPM_HOLD_MS / RPM_STOP_MS):

  No signal for < RPM_HOLD_MS   : STATE_RUNNING, EMA updated normally.
  No signal for < RPM_STOP_MS   : STATE_RUNNING, last RPM held, EMA frozen,
                                   FLAG_CRANKSHAFT_SENSOR_ERROR set in flags.
  No signal for >= RPM_STOP_MS  : STATE_STOPPED, RPM zeroed, filter reset.

RPM_HOLD_MS must be less than RPM_STOP_MS for the hold tier to be active.

Plausibility filter (RPM_ACCEL / RPM_K_WIN / RPM_K_ACC):

Some ECUs implement overspeed protection by firing the injector on every other
cycle (injection skipping). This makes the measured period appear doubled and
RPM appear halved, with no way to detect it from the period alone. The
plausibility filter addresses this:

  Each sample's implied rate of RPM change [RPM/s] is computed against the
  last accepted sample:

      accel = |rpm_new - rpm_last_accepted| / dt_seconds

  If accel > RPM_ACCEL the sample is individually rejected. Results are kept
  in a sliding window of RPM_K_WIN entries. The EMA is updated only when at
  least RPM_K_ACC of the last RPM_K_WIN samples pass. When fewer pass, the
  EMA is frozen and FLAG_CRANKSHAFT_SENSOR_ERROR is set.

  Why this works for injection skipping: at 8000 RPM, a sudden halving to
  4000 RPM implies ~133,000 RPM/s -- 26x above the default RPM_ACCEL of
  5000 RPM/s -- so every skip-injection sample is rejected. Normal injection
  resuming after the cut passes the check (the elapsed time since the last
  accepted sample is large, making any plausible deceleration acceptable).

  The sliding window (rather than requiring all-consecutive passes) tolerates
  a single spurious rejection without stalling the filter.

FLAG_CRANKSHAFT_SENSOR_ERROR is also set during the hold tier (RPM_HOLD_MS to
RPM_STOP_MS), so consumers always have an unambiguous signal when the RPM
value may be stale.

Note on BCAST_MS vs signal period: the broadcast interval does not need to
exceed the signal period. Cycles committed in a previous window carry over
via the preserved staging variables. However, a shorter broadcast interval
increases the proportion of empty windows and slows EMA convergence. A
BCAST_MS of 100-500 ms is recommended for idle engine speeds.

Note on BCAST_MS and plausibility filter effectiveness: the plausibility
check operates on the mean period of all cycles within one broadcast window.
If BCAST_MS is large enough that a normal cycle and an anomalous cycle (skip-
injection or spurious edge) both land in the same window, their periods are
averaged before the check runs, diluting the anomaly and potentially reducing
its apparent RPM/s change. At the default BCAST_MS=10 ms and RPM_MAX=8000
(injection period 15 ms), at most one cycle lands per window, so dilution
does not occur in practice. Users increasing BCAST_MS substantially above
the injection period should be aware of this effect.


DUTY CYCLE CALCULATION
----------------------

    high_us   = time from rising edge to falling edge [us]
    period_us = time between two consecutive rising edges [us]

    duty_pct = 100 x high_us / period_us   [%]

This represents the injection pulse width as a fraction of the total cycle.
It is placed in fuel_consumption_rate_cm3pm (repurposed field; see message
mapping below).


THROTTLE POSITION (THR_SRC)
----------------------------

THR_SRC selects the source for throttle position measurement:

  THR_SRC = 0  Disabled. throttle_position_percent is set to 0x7F (127),
               which is out of the valid 0-100 % range and used as an
               "invalid / not available" sentinel.

  THR_SRC = 1  ADC pin. THR_PIN selects the analog input pin (see PIN TABLE).
               Voltage is read, filtered, and scaled via THR_A1/THR_A0.

  THR_SRC = 2  INA228 over I2C. THR_I2C_ADDR sets the 7-bit I2C address.
               THR_I2C_ADDR = 0 disables the sensor (treated as not configured).
               The INA228 is configured for continuous bus voltage measurement
               with 16-sample averaging at 1052 us conversion time.

THR_SRC, THR_PIN, and THR_I2C_ADDR all require reboot to take effect.


COOLANT TEMPERATURE (CLT_SRC)
------------------------------

CLT_SRC selects the source for coolant temperature measurement:

  CLT_SRC = 0  Disabled. coolant_temperature is set to -273.15 (absolute zero)
               as an "invalid / not available" sentinel.

  CLT_SRC = 1  ADC pin. CLT_PIN selects the analog input pin (see PIN TABLE).
               Voltage is read, filtered, and scaled via CLT_A1/CLT_A0.

  CLT_SRC = 2  INA228 over I2C. CLT_I2C_ADDR sets the 7-bit I2C address
               (default 0x40). CLT_I2C_ADDR = 0 disables the sensor.
               Configuration same as THR INA228 above.

Default: CLT_SRC = 2, CLT_I2C_ADDR = 0x40.

CLT_SRC, CLT_PIN, and CLT_I2C_ADDR all require reboot to take effect.


INA228 SENSOR (I2C)
-------------------

When THR_SRC = 2 or CLT_SRC = 2, an Adafruit INA228 is used to measure a
bus voltage, which is then scaled via A1/A0 to the desired output unit.

Startup behaviour:
  - Wire.begin() is called once if either INA228 is configured.
  - Each sensor is probed via begin(addr). If not found, a debug message
    "THR INA228 not found" / "CLT INA228 not found" is broadcast and the
    channel reports the invalid sentinel value.

Address conflict:
  - If both THR_SRC = 2 and CLT_SRC = 2 are configured with the same
    I2C address, both sensors are disabled for the session and a debug
    message "THR/CLT INA228 addr conflict" is broadcast. Set different
    addresses and reboot to resolve.

Runtime health monitoring (1 Hz):
  - If a sensor was not found at boot: retried every second. On recovery,
    "THR/CLT INA228 found" is broadcast.
  - If a sensor was healthy: conversionReady() is called to verify the CNVR
    bit in DIAG_ALRT. The INA228 sets CNVR on every completed conversion
    (~17 ms cycle in the configured mode). Reading DIAG_ALRT clears CNVR;
    reading the VBUS register does not. Therefore, after one second of normal
    operation, CNVR is guaranteed to be set. If conversionReady() returns
    false, the I2C transaction returned a NACK, indicating the sensor has
    been disconnected. "THR/CLT INA228 lost" is broadcast and the EMA seed
    is cleared. Detection latency is up to ~1 s.

INA228 address hardware configuration (A0/A1 pins on the INA228 module):

  A1 | A0 | Address
  ---|----|--------
   0 |  0 | 0x40  (default)
   0 |  1 | 0x41
   0 | VS | 0x42
   0 | SCL| 0x43
   1 |  0 | 0x44
   1 |  1 | 0x45
   (see INA228 datasheet Table 7 for full list)


ANALOG INPUTS (THROTTLE ADC, COOLANT TEMPERATURE ADC, TEMPERATURE SENSORS)
---------------------------------------------------------------------------

All ADC-based inputs share the same signal processing chain:

  1. ADC read        : raw = analogRead(pin)       [0 - 4095, 12-bit]
  2. Voltage         : V = raw x 3.3 / 4095        [V]
  3. EMA filter      : filtered = alpha x V + (1 - alpha) x filtered_prev
  4. Linear transform: output = A1 x filtered + A0

The A1 and A0 coefficients convert voltage to the desired unit or percent.
Defaults (A1=1.0, A0=0.0) pass voltage through unchanged (output in volts).

Example - throttle 0-100 % from a 0-3.3 V sensor:
    A1 = 100 / 3.3 ~= 30.3,  A0 = 0.0

The ADC is configured for 12-bit resolution (analogReadResolution(12) in
setup), giving raw values 0-4095 for 0-3.3 V.

Maximum ADC input voltage: 3.3 V. Only ADC-capable pins may be used for
analog inputs (see PIN TABLE).


EMA FILTER
----------

An exponential moving average (EMA) is applied independently to each
measurement channel:

    filtered[n] = alpha x raw[n] + (1 - alpha) x filtered[n-1]

    alpha = 1.0  -> no filtering, raw value passes through (default)
    alpha -> 0.0 -> heavy smoothing, slow to respond
    Typical value: 0.1 - 0.5 for a smooth but responsive signal

The filter is seeded with the first valid sample to avoid a slow ramp up
from zero on startup or after a pin/sensor change.

Each channel has its own alpha parameter (RPM_ALPHA, DUTY_ALPHA, THR_ALPHA,
CLT_ALPHA, TMP0_ALPHA, TMP1_ALPHA).


PIN TABLE
---------

All PIN parameters share the same encoding: port x 100 + pin_number
(port A = 0, port B = 1). A negative value disables the input.
An unrecognised positive value, a capability mismatch (e.g. non-ADC pin used
as analog input, non-interrupt pin used as FUELINJ_PIN), or a duplicate
assignment (two inputs sharing one pin) is written back to -1 automatically.

IMPORTANT: Maximum voltage on all pins is 3.3 V. No pins on this board
are 5 V tolerant. Do not apply voltages above 3.3 V.

  Value | Label | STM32 | ADC ch | PWM timer | INT  | ADC? | FUELINJ?
  ------|-------|-------|--------|-----------|------|------|----------
    0   | A0    | PA0   | Ch.5   | TIM2_CH1  | Yes  | Yes  | No
    1   | A1    | PA1   | Ch.6   | TIM2_CH2  | Yes  | Yes  | No  (loopback output in test)
    4   | A4    | PA4   | Ch.9   | --        | Yes  | Yes  | No
    8   | A8    | PA8   | --     | TIM1_CH1  | Yes  | No   | Yes  (default FUELINJ_PIN)
    9   | A9    | PA9   | --     | TIM1_CH2  | Yes  | No   | Yes
   10   | A10   | PA10  | --     | TIM1_CH3  | Yes  | No   | Yes
   11   | A11   | PA11  | --     | TIM1_CH4  | Yes  | No   | Yes
  100   | B0    | PB0   | Ch.15  | --        | Yes  | Yes  | Yes  (BATT_VOLT in AP_Periph *)
  101   | B1    | PB1   | Ch.16  | --        | Yes  | Yes  | Yes  (BATT_CURR in AP_Periph *)

(*) PB0 and PB1 are used for battery voltage/current sensing when running
    AP_Periph firmware. Avoid using them for other functions in that context.

FUELINJ_PIN: use only pins with FUELINJ = Yes (values 8, 9, 10, 11, 100, 101).
             Assigning a non-interrupt-capable pin (0, 1, 4) is rejected and
             written back to -1.

THR_PIN, CLT_PIN, TMP0_PIN, TMP1_PIN: use only pins with ADC = Yes
             (values 0, 1, 4, 100, 101). Assigning a non-ADC pin is rejected
             and written back to -1.


PARAMETER TABLE
---------------

All parameters marked (*) require reboot to take effect.
All other parameters are updated live at 1 Hz.

ICE Status (uavcan.equipment.ice.reciprocating.Status)

  Name          | Type | Default | Min       | Max      | Description
  --------------|------|---------|-----------|----------|----------------------------------
  NODEID        | INT  | 5       | 0         | 127      | DroneCAN node ID (*)
  BAUD          | INT  | 500     | 500       | 1000     | CAN baudrate in kbps (*)
  FUELINJ_PIN   | INT  | 8       | -1        | 101      | Injection input pin; <0=disabled (*)
  BCAST_MS      | INT  | 10      | 10        | 5000     | ICE Status broadcast interval [ms]
  RPM_MAX       | INT  | 8000    | 100       | 20000    | Max measurable RPM; faster periods rejected (*)
  RPM_ALPHA     | REAL | 0.2     | 0.01      | 1.0      | EMA alpha for RPM
  RPM_A1        | REAL | 1.0     | -10000.0  | 10000.0  | RPM linear scale
  RPM_A0        | REAL | 0.0     | -100000.0 | 100000.0 | RPM linear offset
  RPM_HOLD_MS   | INT  | 500     | 100       | 5000     | ms no signal -> hold RPM + CRANKSHAFT_SENSOR_ERROR
  RPM_STOP_MS   | INT  | 3000    | 500       | 30000    | ms no signal -> zero RPM, STATE_STOPPED
  RPM_ACCEL     | REAL | 5000.0  | 100.0     | 100000.0 | Max plausible RPM/s; plausibility filter threshold
  RPM_K_WIN     | INT  | 5       | 2         | 10       | Plausibility sliding window size (*)
  RPM_K_ACC     | INT  | 3       | 1         | 10       | Passes required in window to update EMA
  DUTY_ALPHA    | REAL | 0.3     | 0.01      | 1.0      | EMA alpha for duty cycle
  DUTY_A1       | REAL | 1.0     | -10000.0  | 10000.0  | Duty cycle linear scale
  DUTY_A0       | REAL | 0.0     | -100000.0 | 100000.0 | Duty cycle linear offset
  THR_SRC       | INT  | 1       | 0         | 2        | Throttle source: 0=off 1=ADC 2=INA228 (*)
  THR_I2C_ADDR  | INT  | 0x40    | 0         | 127      | INA228 address when THR_SRC=2; 0=disabled (*)
  THR_PIN       | INT  | -1      | -1        | 101      | ADC pin when THR_SRC=1; <0=disabled (*)
  THR_ALPHA     | REAL | 1.0     | 0.01      | 1.0      | EMA alpha for throttle
  THR_A1        | REAL | 1.0     | -10000.0  | 10000.0  | Throttle linear scale
  THR_A0        | REAL | 0.0     | -100000.0 | 100000.0 | Throttle linear offset
  CLT_SRC       | INT  | 2       | 0         | 2        | Coolant source: 0=off 1=ADC 2=INA228 (*)
  CLT_I2C_ADDR  | INT  | 0x40    | 0         | 127      | INA228 address when CLT_SRC=2; 0=disabled (*)
  CLT_PIN       | INT  | -1      | -1        | 101      | ADC pin when CLT_SRC=1; <0=disabled (*)
  CLT_ALPHA     | REAL | 1.0     | 0.01      | 1.0      | EMA alpha for coolant temperature
  CLT_A1        | REAL | 1.0     | -10000.0  | 10000.0  | Coolant temperature linear scale
  CLT_A0        | REAL | 0.0     | -100000.0 | 100000.0 | Coolant temperature linear offset

Temperature sensors (uavcan.equipment.device.Temperature)

  Name          | Type | Default | Min       | Max      | Description
  --------------|------|---------|-----------|----------|----------------------------------
  TMP0_PIN      | INT  | -1      | -1        | 101      | Sensor 0 analog pin; <0=disabled (*)
  TMP0_ID       | INT  | 0       | 0         | 65535    | device_id in Temperature message (*)
  TMP0_BCAST    | INT  | 1000    | 10        | 5000     | Sensor 0 broadcast interval [ms]
  TMP0_ALPHA    | REAL | 1.0     | 0.01      | 1.0      | EMA alpha for sensor 0
  TMP0_A1       | REAL | 1.0     | -10000.0  | 10000.0  | Sensor 0 linear scale
  TMP0_A0       | REAL | 0.0     | -100000.0 | 100000.0 | Sensor 0 linear offset
  TMP1_PIN      | INT  | -1      | -1        | 101      | Sensor 1 analog pin; <0=disabled (*)
  TMP1_ID       | INT  | 1       | 0         | 65535    | device_id in Temperature message (*)
  TMP1_BCAST    | INT  | 1000    | 10        | 5000     | Sensor 1 broadcast interval [ms]
  TMP1_ALPHA    | REAL | 1.0     | 0.01      | 1.0      | EMA alpha for sensor 1
  TMP1_A1       | REAL | 1.0     | -10000.0  | 10000.0  | Sensor 1 linear scale
  TMP1_A0       | REAL | 0.0     | -100000.0 | 100000.0 | Sensor 1 linear offset

(*) Requires reboot to take effect.


DRONECAN MESSAGE MAPPING
------------------------

Message: uavcan.equipment.ice.reciprocating.Status  (ID 1120)
Broadcast at BCAST_MS interval when at least one source is configured
(see BROADCAST CONDITION above).

  Field                       | Source
  ----------------------------|----------------------------------------------
  state                       | STATE_RUNNING when no_signal < RPM_STOP_MS
                              |   (includes the hold tier; engine still spinning)
                              | STATE_STOPPED when no_signal >= RPM_STOP_MS
                              |   or FUELINJ_PIN disabled
  flags                       | FLAG_CRANKSHAFT_SENSOR_ERROR_SUPPORTED always set
                              |   when FUELINJ_PIN is enabled.
                              | FLAG_CRANKSHAFT_SENSOR_ERROR additionally set when:
                              |   - hold tier active (RPM_HOLD_MS..RPM_STOP_MS), or
                              |   - plausibility filter is rejecting samples
                              |   (RPM value is frozen at last accepted reading)
  engine_load_percent         | 0
  engine_speed_rpm            | RPM after EMA + linear transform; 0 if
                              |   FUELINJ_PIN disabled or truly stopped
  fuel_consumption_rate_cm3pm | Injection duty cycle [%] after EMA + transform
                              |   (repurposed field -- see note below)
  coolant_temperature         | CLT: A1 x V + A0 after EMA when valid;
                              |   -273.15 if CLT disabled or sensor not ready
  throttle_position_percent   | THR: round(A1 x V + A0) constrained 0-100;
                              |   0x7F (127) if THR disabled or sensor not ready
                              |   (127 is out of the valid 0-100 % range and
                              |    used as an "invalid" sentinel)
  all other fields            | 0 / default

Note: fuel_consumption_rate_cm3pm is repurposed to carry injection duty cycle
in percent. Configure the receiving autopilot driver accordingly.

Note: engine_speed_rpm is a 17-bit field in the wire format (max 131071 RPM).

---

Message: uavcan.equipment.device.Temperature  (ID 1110)
One message per enabled sensor (TMP0, TMP1), each on its own broadcast timer.

  Field       | Source
  ------------|------------------------------------------------
  device_id   | TMP0_ID or TMP1_ID
  temperature | A1 x EMA(voltage) + A0
  error_flags | 0


CAN BAUDRATE (BAUD parameter)
------------------------------

Supported values: 500 (kbps) and 1000 (kbps).

The BAUD parameter is read from flash at boot by libArduinoDroneCAN
(v2.0.1-es or later) before CAN initialisation. Changing BAUD and rebooting
is sufficient to switch baudrate; no firmware reflash is required.

Changing BAUD always requires a reboot to take effect.


TEST FEATURE (LOOPBACK_TEST)
-----------------------------

Define LOOPBACK_TEST to generate a PWM signal on PA1 (TIM2_CH2) and loop it
back to the injection input pin for self-test without real ECU hardware.

When enabled, three extra DroneCAN parameters appear:

  LB_EN    [INT, 0-1]       : 0 = PWM output off, 1 = PWM output on (default 1)
  LB_DUTY  [INT, 0-100]     : loopback signal duty cycle [%]  (default 50)
  LB_FREQ  [INT, 1-20000]   : loopback signal frequency [Hz]  (default 60)

These parameters take effect at boot (reboot required to change them).
Wire: PA1 -> FUELINJ_PIN (default PA8).

When LB_EN = 1, any input pin (THR_PIN, CLT_PIN, TMP0_PIN, TMP1_PIN,
FUELINJ_PIN) assigned to PA1 (value = 1) is automatically disabled and its
parameter written back to -1 to prevent output/input conflicts.

PA1 is chosen because:
  - It is a dedicated PWM output pin on the CAN Node header (TIM2_CH2)
  - It does not conflict with CAN (PA11/PA12), Serial (PA2/PA3), or any
    default measurement pin
  - PA0 was avoided due to a reported connectivity issue on some carrier boards

Remove the #define LOOPBACK_TEST and the LB_* parameters when deploying in
production.
