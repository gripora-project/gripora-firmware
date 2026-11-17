<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/gripora-project/gripora-website/master/branding/gripora_logo_white.svg">
    <img src="https://raw.githubusercontent.com/gripora-project/gripora-website/master/branding/gripora_logo_dark.svg" alt="Gripora" width="420">
  </picture>
</p>

# Gripora Firmware

Gripora is open-source stability-control firmware for RC cars: a gyro that keeps the car pointing
where you steer it, with the braking and traction aids of a full-size car's chassis electronics.

**[Website](https://gripora.org)** ·
**[Documentation](https://doc.gripora.org)** ·
**[Releases](https://github.com/gripora-project/gripora-firmware/releases)**

> Gripora has just started. It is a fork of [WingFlight](https://github.com/WingFlight/wingflight-firmware),
> and this repository still contains WingFlight's fixed-wing flight code while the car conversion
> is under way. It is not ready to drive a car yet.


## Goals

* **Steering stabilization**: yaw-rate gyro control that catches slides and corrects for
  surface changes, with gain from the transmitter
* **ABS**: modulates braking to keep the wheels turning, using motor RPM from ESC telemetry
* **Traction control**: limits wheelspin on throttle by comparing wheel speed with the car's
  measured motion
* **Launch control** and throttle/brake shaping
* Setup and tuning from a desktop/web **Configurator** and from **Lua scripts** on the transmitter
  (FrSky Ethos, EdgeTX)
* High-rate **Blackbox** logging to tune and diagnose all of the above

Inherited from WingFlight / Rotorflight / Betaflight, and kept:

* Receivers: CRSF/ELRS, S.BUS, F.Port, FBUS, SRXL2, Spektrum DSM, IBUS, SUMD, XBUS, Jeti EX Bus, GHOST
* Telemetry back to the transmitter, and ESC telemetry from most common ESCs
* Motor protocols: PWM, OneShot, Multishot, DShot, Castle Link and SRXL2
* Gyro filtering with dynamic notch and RPM filters
* Battery voltage, current and BEC sensing, GPS and RGB LEDs


## The Gripora project

| | |
| --- | --- |
| **Firmware** | This repository |
| [Configurator](https://github.com/gripora-project/gripora-configurator) | Flashes and configures the controller, on the desktop or in a browser |
| [Blackbox Explorer](https://github.com/gripora-project/gripora-blackbox) | Analyses logs |
| [Lua suite for Ethos](https://github.com/gripora-project/gripora-lua-ethos-suite) | Configuration and dashboard on FrSky Ethos radios |
| [Lua scripts for EdgeTX](https://github.com/gripora-project/gripora-lua-edgetx) | Configuration on EdgeTX radios |
| [Targets](https://github.com/gripora-project/gripora-targets) | Board configurations |
| [Documentation](https://github.com/gripora-project/gripora-docs) | Source of [doc.gripora.org](https://doc.gripora.org) |
| [Website](https://github.com/gripora-project/gripora-website) | Source of [gripora.org](https://gripora.org) |


## Hardware

Gripora builds one firmware per MCU family: STM32F405, STM32F411, STM32F7X2, STM32F745, STM32G47X
and STM32H743. The board itself (pins and defaults) is a configuration applied on top, kept in
[gripora-targets](https://github.com/gripora-project/gripora-targets).


## Building

```
make arm_sdk_install      # once: installs the ARM toolchain under tools/
make TARGET=STM32F7X2     # one target; `make unified` builds all
make test                 # unit tests
```

See [AGENTS.md](AGENTS.md) for the layout of the code and the rules for changing it.


## Contributing

Bug reports, ideas and pull requests are welcome in the
[issue tracker](https://github.com/gripora-project/gripora-firmware/issues).


## Origins

Gripora is open source and free of charge, with no warranty. It is licensed under the GPL v3
(see [LICENSE](LICENSE)).

It is a fork of [WingFlight](https://github.com/WingFlight), which is a fork of
[Rotorflight](https://github.com/rotorflight), which is a fork of
[Betaflight](https://github.com/betaflight) 4.3, which is a fork of
[Cleanflight](https://github.com/cleanflight). Thank you to everyone who contributed along the way.
