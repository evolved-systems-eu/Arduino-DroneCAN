# PWM Duty Cycle Measurement Example

Measures the duty cycle of a PWM signal on a selectable GPIO pin and broadcasts
the result as a DroneCAN `uavcan.equipment.device.Temperature` message at 10 Hz.

## How it works

An external interrupt fires on every edge of the input signal.  The ISR records:

- **High-pulse duration** (`high_us`) — time between the rising and falling edge
- **Period** (`period_us`) — time between two consecutive rising edges

The duty cycle is computed as `100 × high_us / period_us` [%] and placed in the
`temperature` field of the message.  If no edge is seen for 500 ms the signal is
considered absent and 0 % is reported.

> **Note:** the `temperature` field carries duty cycle in percent, not degrees
> Kelvin.  Configure the receiving autopilot driver (e.g. ArduPilot
> `TEMP_SENSx_TYPE`) to interpret `device_id` accordingly.

## Parameters

| Parameter  | Type | Default | Range   | Description                                    |
|------------|------|---------|---------|------------------------------------------------|
| NODEID     | INT  | 100     | 0–127   | DroneCAN node ID                               |
| DEVICE_ID  | INT  | 0       | 0–127   | `device_id` field in the Temperature message   |
| PWM_PIN    | INT  | 8       | 8–101   | Pin encoded as port×100+pin (see below)        |

`PWM_PIN` is hot-swappable: changing it over DroneCAN reconfigures the interrupt
pin within one second without requiring a reboot. Invalid values fall back to PA8.

## Pin table

The parameter value encodes the pin directly as `port × 100 + pin_number`
(port A = 0, port B = 1), so the number shown in the DroneCAN GUI identifies
the pin without needing a separate lookup table.

| PWM_PIN value | STM32 pin |
|---------------|-----------|
| 8             | PA8       |
| 9             | PA9       |
| 10            | PA10      |
| 100           | PB0       |
| 101           | PB1       |

All listed pins support external interrupts on the Micro Node / CAN Node
(STM32L431) and the CAN Node Plus (STM32H723).

## Dependencies

No extra libraries required.  Uses the Arduino `attachInterrupt` / `micros` API
available on all supported boards.

## Serial output

At 115200 baud the node prints the current duty cycle once per 100 ms:

```
PWM duty: 47.3 %
PWM duty: 47.4 %
```
