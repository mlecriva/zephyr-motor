# Zephyr Motor

Zephyr [module][modules] and [workspace application][workspace_app] with
motor drivers, a motor control subsystem, devicetree bindings and
shell-driven samples.

[modules]: https://docs.zephyrproject.org/4.4.2/develop/modules.html
[workspace_app]: https://docs.zephyrproject.org/4.4.2/develop/application/index.html#zephyr-workspace-app

## Objective

This repository demonstrates an approach to motor drivers in Zephyr. Vendor
and technology specific drivers sit behind a generic motor driver class, which
exposes one common API regardless of the motor type and of how it is driven at
low level.

A motor control subsystem can then be layered on top of it to add closed-loop
control.

The example implements a sensored 6-step drive of a BLDC motor, with a PID
speed loop on top. The split between drivers and subsystem is designed so that
other drive modes can be added later, such as a brushed DC motor or
field-oriented control (FOC).

> Note  
> Current control loops are out of scope for now. Where they belong still needs
> some thought. The current lead is the driver side, since such a loop has to
> run at high frequency with hard real-time constraints. Going through the
> generic layers on every tick would add latency and jitter, so the loop would
> run in the driver, triggered by hardware, and only a slow setpoint would
> cross the API.

## Demo hardware

The samples are run on the following hardware:

- **Board**: [NUCLEO-G431RB][nucleo_g431rb] (STM32G431RB) stacked with the
  [X-NUCLEO-IHM16M1][ihm16m1] three-phase driver shield (STSPIN830).
- **Motor**: [HM4820H][motor] three-phase brushless motor with Hall sensors 14 poles (7 pole pairs).

The board and the shield are also sold together as the [P-NUCLEO-IHM03][kit]
motor control kit.

[kit]: https://estore.st.com/en/p-nucleo-ihm03-cpn.html
[nucleo_g431rb]: https://www.st.com/en/evaluation-tools/nucleo-g431rb.html
[ihm16m1]: https://www.st.com/en/ecosystems/x-nucleo-ihm16m1.html
[motor]: https://fr.aliexpress.com/item/1005004516876627.html

## Getting started

### Initialization

Initialize the workspace folder (`motor-workspace`) where `zephyr-motor` and its
Zephyr modules will be cloned:

```shell
west init -m https://github.com/mlecriva/zephyr-motor --mr main motor-workspace
cd motor-workspace
```

### Create Python virtual environment

Use `apt` to install Python venv package:

```shell
sudo apt install python3-venv
```

Create a new virtual environment:

```shell
python3 -m venv .venv
```

Activate the virtual environment. This should be done before running any west command.

```shell
source .venv/bin/activate
```

### Install west in virtual env

```shell
pip install west
```

### Get the Zephyr source code from the manifest

```shell
west update --narrow
```

Export a Zephyr CMake package. This allows CMake to automatically load boilerplate code required for building Zephyr applications:

```shell
west zephyr-export
```

### Install Python dependencies in virtual env

Install the Python dependencies required by Zephyr in virtual env using `west packages`:

```shell
pip install -r zephyr/scripts/requirements.txt
```

> Note  
> This could downgrade or upgrade west itself.

### Install the Zephyr SDK

Install the `Zephyr SDK` version required by the current Zephyr version using `west sdk`, and to
avoid to install the toolchains of every architecture supported by the SDK (which represents several gigabytes),
use `--toolchains`/`-t` to install only the ones required by the target boards, for instance `arm-zephyr-eabi`
for the Cortex-M boards of this template:

```shell
west sdk install --toolchains arm-zephyr-eabi
```

> Note  
> Several toolchains can be given at once (`--toolchains arm-zephyr-eabi riscv64-zephyr-elf`), and
> `west sdk install --interactive` lets you pick them from a prompt instead.

The SDK is installed in `${HOME}/zephyr-sdk-<version>`, use `--install-base` to choose another parent directory. Run
`west sdk install --help` for the remaining options.

### Building and running

Two samples are provided:

| Sample | Description |
| --- | --- |
| [samples/driver/motor](samples/driver/motor/README.md) | Drives the motor open-loop from the shell and reads its feedback (angle, speed, phase currents). Board wiring and shell usage. |
| [samples/subsys/motor_control](samples/subsys/motor_control/README.md) | Runs a closed speed loop on the motor and tunes it from the shell. |

Each sample ships a `Makefile` wrapping `west build`/`west flash` with sysbuild. Run it from the sample directory:

```shell
cd zephyr-motor/samples/driver/motor
make
make flash
```

Use `samples/subsys/motor_control` instead for the closed speed loop.

`BOARD` defaults to `nucleo_g431rb`; override it (and `BUILD_DIR`) on the
command line, e.g. `make BOARD=<board>`. `make clean` removes the build
directory.

The closed-loop sample reuses the board setup of the driver sample, so start
with the latter for the wiring.
