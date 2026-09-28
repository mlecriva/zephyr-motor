# Motor driver sample

Drives the motors of the board open-loop from the shell and reads their
feedback (shaft angle, speed, phase currents) as sensors, using the motor
driver directly with no motor control subsystem. The application holds no
motor code: each board overlay describes the power stage and picks the
motor driver.

## Nucleo-G431RB + X-NUCLEO-IHM16M1

The overlay runs a 6-step motor with Hall sensors and 3-shunt current
sensing on the STSPIN830 shield, stacked on the morpho connectors.

### Shield setup

The factory shield drives each phase with one input and one enable. This
sample drives each switch from a complementary timer output, which needs
the STSPIN830 in MODE=H: remove R12 and replace R11 (39 kΩ) with 0 Ω.

Keep the factory jumpers, which select 3-shunt sensing: JP4 and JP7 open,
J5 and J6 closed, J2 on 2-3, J3 on 1-2. The STSPIN830 current limiter is
off in this configuration, so nothing limits the phase current in
hardware.

Connect the motor phases to CN1 pins 3 to 5, the Hall sensors to J1, and a
7 to 45 V supply to CN1 pins 1-2 or to J4. With push-pull Hall sensors,
remove the pull-ups R20 to R22.

### Signals

| Signal | Pins | Peripheral |
| ------ | ---- | ---------- |
| Phase U, V, W high side | PA8, PA9, PA10 | TIM1 CH1, CH2, CH3 |
| Phase U, V, W low side | PB13, PB14, PB15 | TIM1 CH1N, CH2N, CH3N |
| EN/FAULT | PB12 | TIM1 BKIN (not armed) |
| Hall 1, 2, 3 | PA15, PB3, PB10 | TIM2 CH1, CH2, CH3 |
| Phase U, V, W current | PA1, PB1, PB0 | ADC1 IN2, IN12, IN15 |

The current values come from the shield: 330 mΩ shunts and a 1.53 gain.
The pole pairs are those of the motor and must be adjusted. The Nucleo
SPI1, SPI2, SPI3, USART1 and TIM2 PWM LED are disabled, as they share pins
with the shield.

## Building and running

```console
make
make flash
```

On the console (ST-Link virtual COM port, 115200 baud):

```console
uart:~$ motor voltage 200
uart:~$ motor start
uart:~$ sensor get feedback
uart:~$ motor stop
```

The voltage is in per-mille of the supply, its sign sets the direction.

## References

- [UM2415](https://www.st.com/resource/en/user_manual/um2415-getting-started-with-the-xnucleoihm16m1-threephase-brushless-motor-driver-board-based-on-stspin830-for-stm32-nucleo-stmicroelectronics.pdf):
  X-NUCLEO-IHM16M1 user manual
