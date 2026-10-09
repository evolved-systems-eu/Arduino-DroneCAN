ICEMonitor – Fuel Injection Signal Monitor
==========================================

Monitors a fuel injection signal from a Delphi MT05 ECU, extracts RPM and
injection duty cycle, and broadcasts the results over DroneCAN as
uavcan.equipment.ice.reciprocating.Status.

Additional analog inputs for throttle position and coolant temperature are
optionally broadcast in the same message. Two independent analog temperature
sensors broadcast via uavcan.equipment.device.Temperature.

All analog and digital inputs are independently enabled or disabled by setting
their pin parameter. A negative value disables the input; a valid pin number
(see PIN TABLE) enables it. If no ICE inputs are active (FUELINJ_PIN, THR_PIN
and CLT_PIN all negative) the ICE Status message is not broadcast.


INPUT SIGNAL (INJECTION)
------------------------

Source : Delphi MT05 ECU fuel injector output
Logic  : Active HIGH = fuel is being injected
           Rising edge  → injection starts
           Falling edge → injection ends
Engine : Single cylinder, 4-stroke
           → 1 injection per 2 crankshaft revolutions

Signal characteristics:
  - Idle RPM range  : ~1000–3000 RPM → period 40–120 ms
  - Max RPM range   : up to ~10000 RPM → period ~12 ms
  - Pulse width     : typically 2–20 ms depending on load
  - Logic level     : 3.3 V max (see PIN TABLE)


RPM CALCULATION
---------------

The ISR timestamps each rising edge. The period between two consecutive
rising edges equals one full injection cycle = 2 crankshaft revolutions.

    period_us = time between two consecutive rising edges [µs]

    RPM = 2 revolutions / period_seconds * 60 s/min
        = 120 / period_seconds
        = 120,000,000 / period_us

Example: period_us = 20,000 µs → RPM = 120,000,000 / 20,000 = 6000 RPM

All complete injection cycles committed during the broadcast window are averaged
before being fed into the EMA filter:

    mean_period = sum(period_us) / count
    mean_high   = sum(high_us)   / count

Only matched pairs (both rising and falling edge captured for the same cycle)
are included. The accumulators (period_sum, high_sum, count) are reset at each
broadcast. The ISR staging variables (last rising edge timestamp, pending pulse
width) are NOT reset, so a cycle that spans two broadcast windows is still
committed correctly at the next rising edge.

When no new complete cycle falls within a broadcast window but the signal is
still active (an edge was seen within the last 2000 ms), the previous EMA
values are preserved and STATE_RUNNING is reported. RPM and duty cycle are
only zeroed when the signal goes truly stale (no edge for 2000 ms), at which
point STATE_STOPPED is reported.

Signal stale timeout: 2000 ms (covers down to ~60 RPM engine speed, at which
point period = 2,000,000 µs = 2 s).

Note on BCAST_MS vs signal period: the broadcast interval does not need to
exceed the signal period. Cycles committed in a previous window carry over
via the preserved staging variables. However, a shorter broadcast interval
increases the proportion of empty windows and slows EMA convergence. A
BCAST_MS of 100–500 ms is recommended for idle engine speeds.


DUTY CYCLE CALCULATION
----------------------

    high_us   = time from rising edge to falling edge [µs]
    period_us = time between two consecutive rising edges [µs]

    duty_pct = 100 × high_us / period_us   [%]

This represents the injection pulse width as a fraction of the total cycle.
It is placed in fuel_consumption_rate_cm3pm (repurposed field; see message
mapping below).


ANALOG INPUTS (THROTTLE, COOLANT TEMPERATURE, TEMPERATURE SENSORS)
-------------------------------------------------------------------

All analog inputs share the same signal processing chain:

  1. ADC read       : raw = analogRead(pin)       [0 – 4095, 12-bit]
  2. Voltage        : V = raw × 3.3 / 4095        [V]
  3. EMA filter     : filtered = alpha × V + (1 − alpha) × filtered_prev
  4. Linear transform: output = A1 × filtered + A0

The A1 and A0 coefficients convert voltage to the desired unit or percent.
Defaults (A1=1.0, A0=0.0) pass voltage through unchanged (output in volts).

Example – throttle 0–100 % from a 0–3.3 V sensor:
    A1 = 100 / 3.3 ≈ 30.3,  A0 = 0.0

The ADC is configured for 12-bit resolution (analogReadResolution(12) in
setup), giving raw values 0–4095 for 0–3.3 V.

Maximum ADC input voltage: 3.3 V. Only ADC-capable pins may be used for
analog inputs (see PIN TABLE).


EMA FILTER
----------

An exponential moving average (EMA) is applied independently to each
measurement channel:

    filtered[n] = alpha × raw[n] + (1 − alpha) × filtered[n−1]

    alpha = 1.0  → no filtering, raw value passes through (default)
    alpha → 0.0  → heavy smoothing, slow to respond
    Typical value: 0.1 – 0.5 for a smooth but responsive signal

Each channel has its own alpha parameter (RPM_ALPHA, DUTY_ALPHA, THR_ALPHA,
CLT_ALPHA, TMP0_ALPHA, TMP1_ALPHA).


PIN TABLE
---------

All PIN parameters share the same encoding: port × 100 + pin_number
(port A = 0, port B = 1). A negative value disables the input.
An unrecognised positive value, a non-ADC pin used as analog input, or a
duplicate assignment (two inputs sharing one pin) is written back to -1
automatically (see resolve_pin() and refresh_pin_params() in main.cpp).

IMPORTANT: Maximum voltage on all pins is 3.3 V. No pins on this board
are 5 V tolerant. Do not apply voltages above 3.3 V.

  Value | Label | STM32 | ADC ch | PWM timer    | INT | Use as analog input?
  ------|-------|-------|--------|--------------|-----|---------------------
    0   | A0    | PA0   |  Ch.5  | TIM2_CH1     |  Y  | Yes
    1   | A1    | PA1   |  Ch.6  | TIM2_CH2     |  Y  | Yes  (loopback output in test)
    4   | A4    | PA4   |  Ch.9  | —            |  Y  | Yes
    8   | A8    | PA8   |  —     | TIM1_CH1     |  Y  | No  (no ADC channel)
    9   | A9    | PA9   |  —     | TIM1_CH2     |  Y  | No  (no ADC channel)
   10   | A10   | PA10  |  —     | TIM1_CH3     |  Y  | No  (no ADC channel)
   11   | A11   | PA11  |  —     | TIM1_CH4     |  Y  | No  (no ADC channel; also USB_DM)
  100   | B0    | PB0   | Ch.15  | —            |  Y  | Yes  (BATT_VOLT in AP_Periph *)
  101   | B1    | PB1   | Ch.16  | —            |  Y  | Yes  (BATT_CURR in AP_Periph *)

(*) PB0 and PB1 are used for battery voltage/current sensing when running
    AP_Periph firmware. Avoid using them for other functions in that context.

Use any pin (INT = Y) for FUELINJ_PIN.
Use only ADC-capable pins (ADC ch ≠ —) for THR_PIN, CLT_PIN, TMP0_PIN,
TMP1_PIN. Assigning a non-ADC pin to an analog input is rejected and written
back to -1.


PARAMETER TABLE
---------------

ICE Status (uavcan.equipment.ice.reciprocating.Status)

  Name        | Type | Default | Min       | Max      | Description
  ------------|------|---------|-----------|----------|----------------------------------
  NODEID      | INT  | 5       | 0         | 127      | DroneCAN node ID
  BAUD        | INT  | 500     | 500       | 1000     | CAN baudrate in kbps (see note)
  FUELINJ_PIN | INT  | 8       | -1        | 101      | Injection input pin; <0 = disabled
  BCAST_MS    | INT  | 10      | 10        | 5000     | ICE Status broadcast interval [ms]
  RPM_ALPHA   | REAL | 0.2     | 0.01      | 1.0      | EMA alpha for RPM
  RPM_A1      | REAL | 1.0     | -10000.0  | 10000.0  | RPM linear scale
  RPM_A0      | REAL | 0.0     | -100000.0 | 100000.0 | RPM linear offset
  DUTY_ALPHA  | REAL | 0.3     | 0.01      | 1.0      | EMA alpha for duty cycle
  DUTY_A1     | REAL | 1.0     | -10000.0  | 10000.0  | Duty cycle linear scale
  DUTY_A0     | REAL | 0.0     | -100000.0 | 100000.0 | Duty cycle linear offset
  THR_PIN     | INT  | -1      | -1        | 101      | Throttle analog pin; <0 = disabled
  THR_ALPHA   | REAL | 1.0     | 0.01      | 1.0      | EMA alpha for throttle
  THR_A1      | REAL | 1.0     | -10000.0  | 10000.0  | Throttle linear scale
  THR_A0      | REAL | 0.0     | -100000.0 | 100000.0 | Throttle linear offset
  CLT_PIN     | INT  | -1      | -1        | 101      | Coolant temp analog pin; <0 = disabled
  CLT_ALPHA   | REAL | 1.0     | 0.01      | 1.0      | EMA alpha for coolant temperature
  CLT_A1      | REAL | 1.0     | -10000.0  | 10000.0  | Coolant temperature linear scale
  CLT_A0      | REAL | 0.0     | -100000.0 | 100000.0 | Coolant temperature linear offset

Temperature sensors (uavcan.equipment.device.Temperature)

  Name        | Type | Default | Min       | Max      | Description
  ------------|------|---------|-----------|----------|----------------------------------
  TMP0_PIN    | INT  | -1      | -1        | 101      | Sensor 0 analog pin; <0 = disabled
  TMP0_ID     | INT  | 0       | 0         | 65535    | device_id in Temperature message
  TMP0_BCAST  | INT  | 1000    | 10        | 5000     | Sensor 0 broadcast interval [ms]
  TMP0_ALPHA  | REAL | 1.0     | 0.01      | 1.0      | EMA alpha for sensor 0
  TMP0_A1     | REAL | 1.0     | -10000.0  | 10000.0  | Sensor 0 linear scale
  TMP0_A0     | REAL | 0.0     | -100000.0 | 100000.0 | Sensor 0 linear offset
  TMP1_PIN    | INT  | -1      | -1        | 101      | Sensor 1 analog pin; <0 = disabled
  TMP1_ID     | INT  | 1       | 0         | 65535    | device_id in Temperature message
  TMP1_BCAST  | INT  | 1000    | 10        | 5000     | Sensor 1 broadcast interval [ms]
  TMP1_ALPHA  | REAL | 1.0     | 0.01      | 1.0      | EMA alpha for sensor 1
  TMP1_A1     | REAL | 1.0     | -10000.0  | 10000.0  | Sensor 1 linear scale
  TMP1_A0     | REAL | 0.0     | -100000.0 | 100000.0 | Sensor 1 linear offset

BAUD note: see CAN BAUDRATE section below.


DRONECAN MESSAGE MAPPING
------------------------

Message: uavcan.equipment.ice.reciprocating.Status  (ID 1120)
Broadcast when at least one of FUELINJ_PIN, THR_PIN, CLT_PIN is enabled.

  Field                          | Source
  -------------------------------|----------------------------------------------
  state                          | STATE_RUNNING when signal not stale
                                 |   (any edge within last 2000 ms)
                                 | STATE_STOPPED when stale or FUELINJ_PIN disabled
  flags                          | 0
  engine_load_percent            | 0
  engine_speed_rpm               | FUELINJ_PIN: RPM after EMA + linear transform
                                 | 0 if FUELINJ_PIN disabled or signal stale
  fuel_consumption_rate_cm3pm    | FUELINJ_PIN: duty cycle [%] after EMA + transform
                                 | (repurposed field — see note)
  coolant_temperature            | CLT_PIN: A1×V+A0 after EMA; 0 if disabled
  throttle_position_percent      | THR_PIN: round(A1×V+A0) constrained 0–100;
                                 | 0 if disabled
  all other fields               | 0 / default

Note: fuel_consumption_rate_cm3pm is repurposed to carry injection duty cycle
in percent. Configure the receiving autopilot driver accordingly.

Note: engine_speed_rpm is a 17-bit field in the wire format (max 131071 RPM).

---

Message: uavcan.equipment.device.Temperature  (ID 1110)
One message per enabled sensor (TMP0, TMP1), each on its own broadcast timer.

  Field       | Source
  ------------|------------------------------------------------
  device_id   | TMP0_ID or TMP1_ID
  temperature | A1 × EMA(voltage) + A0
  error_flags | 0


CAN BAUDRATE (BAUD parameter)
------------------------------

The library (libArduinoDroneCAN v2.0.0) hardcodes 1 Mbps in dronecan.cpp:

    return CANInit(CAN_1000KBPS, 2);  // dronecan.cpp

The underlying canL431.h defines CAN_500KBPS and CAN_1000KBPS as valid enum
values, so the hardware supports 500 kbps. Two options to implement runtime
selection:

  Option A (modify library): change dronecan.cpp to read the BAUD parameter
    from flash before calling CANInit, and pass CAN_500KBPS or CAN_1000KBPS
    accordingly. Requires forking libArduinoDroneCAN.

  Option B (pre-init workaround): call CANInit(CAN_500KBPS, 2) directly in
    the application before dronecan.init(). The library will then re-call
    CANInit inside init() overwriting it with 1 Mbps — this does NOT work
    without Option A.

Status: BAUD parameter is included in the parameter table and stored in flash,
but runtime selection is not implemented in v1. The node always runs at 1 Mbps.
Implement Option A when 500 kbps support is required.

Changing BAUD always requires a node restart to take effect regardless of the
implementation chosen.


TEST FEATURE (LOOPBACK_TEST)
-----------------------------

Define LOOPBACK_TEST to generate a PWM signal on PA1 (TIM2_CH2) and loop it
back to the injection input pin for self-test without real ECU hardware.

When enabled, three extra DroneCAN parameters appear:

  LB_EN    [INT, 0–1]       : 0 = PWM output off, 1 = PWM output on (default 1)
  LB_DUTY  [INT, 0–100]     : loopback signal duty cycle [%]  (default 50)
  LB_FREQ  [INT, 1–20000]   : loopback signal frequency [Hz]  (default 60)

All three can be changed live from the DroneCAN GUI without reflashing.
LB_EN = 0 leaves PA1 as high-impedance input (does not drive the pin).
LB_DUTY and LB_FREQ are updated at 1 Hz but only take effect when LB_EN = 1.
Wire: PA1 → FUELINJ_PIN (default PA8).

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
