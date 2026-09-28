# Zephyr Motor

Zephyr [module][modules] and [workspace application][workspace_app] with
motor drivers, devicetree bindings and a shell-driven sample.

[modules]: https://docs.zephyrproject.org/4.4.2/develop/modules.html
[workspace_app]: https://docs.zephyrproject.org/4.4.2/develop/application/index.html#zephyr-workspace-app

## Getting started

Before getting started, make sure you have a proper Zephyr development
environment. Follow the official
[Zephyr Getting Started Guide](https://docs.zephyrproject.org/4.4.2/develop/getting_started/index.html).

### Initialization

Initialize the workspace folder (`motor-workspace`) where `zephyr-motor` and its
Zephyr modules will be cloned:

```shell
west init -m https://github.com/mlecriva/zephyr-motor --mr main motor-workspace
cd motor-workspace
west update --narrow
```

### Building and running

Each sample ships a `Makefile` wrapping `west build`/`west flash` with
sysbuild. Run it from the sample directory:

```shell
cd zephyr-motor/samples/driver/motor
make
make flash
```

`BOARD` defaults to `nucleo_g431rb`; override it (and `BUILD_DIR`) on the
command line, e.g. `make BOARD=<board>`. `make clean` removes the build
directory.

See [samples/driver/motor](samples/driver/motor/README.md) for the board
wiring and shell usage.
