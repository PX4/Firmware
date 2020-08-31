# PX4 Drone Autopilot

[![Releases](https://img.shields.io/github/release/PX4/Firmware.svg)](https://github.com/PX4/Firmware/releases) [![DOI](https://zenodo.org/badge/22634/PX4/Firmware.svg)](https://zenodo.org/badge/latestdoi/22634/PX4/Firmware)

[![Nuttx Targets](https://github.com/PX4/Firmware/workflows/Nuttx%20Targets/badge.svg)](https://github.com/PX4/Firmware/actions?query=workflow%3A%22Nuttx+Targets%22?branch=master) [![SITL Tests](https://github.com/PX4/Firmware/workflows/SITL%20Tests/badge.svg?branch=master)](https://github.com/PX4/Firmware/actions?query=workflow%3A%22SITL+Tests%22)

[![Slack](https://px4-slack.herokuapp.com/badge.svg)](http://slack.px4.io)

This repository holds the [PX4](http://px4.io) flight control solution for drones, with the main applications located in the [src/modules](https://github.com/PX4/Firmware/tree/master/src/modules) directory. It also contains the PX4 Drone Middleware Platform, which provides drivers and middleware to run drones.

* Official Website: http://px4.io (License: BSD 3-clause, [LICENSE](https://github.com/PX4/Firmware/blob/master/LICENSE))
* [Supported airframes](https://docs.px4.io/master/en/airframes/airframe_reference.html) ([portfolio](http://px4.io/#airframes)):
  * [Multicopters](https://docs.px4.io/master/en/airframes/airframe_reference.html#copter)
  * [Fixed wing](https://docs.px4.io/master/en/airframes/airframe_reference.html#plane)
  * [VTOL](https://docs.px4.io/master/en/airframes/airframe_reference.html#vtol)
  * many more experimental types (Rovers, Blimps, Boats, Submarines, Autogyros, etc)
* Releases: [Downloads](https://github.com/PX4/Firmware/releases)


## PX4 Users

The [PX4 User Guide](https://docs.px4.io/master/en/) explains how to assemble [supported vehicles](https://docs.px4.io/master/en/airframes/airframe_reference.html) and fly drones with PX4.
See the [forum and chat](https://docs.px4.io/master/en/#support) if you need help!


## PX4 Developers

This [Developer Guide](https://dev.px4.io/) is for software developers who want to modify the flight stack and middleware (e.g. to add new flight modes), hardware integrators who want to support new flight controller boards and peripherals, and anyone who wants to get PX4 working on a new (unsupported) airframe/vehicle.

Developers should read the [Guide for Contributions](https://dev.px4.io/master/en/contribute/).
See the [forum and chat](https://dev.px4.io/master/en/#support) if you need help!


### Weekly Dev Call

The PX4 Dev Team syncs up on a [weekly dev call](https://dev.px4.io/master/en/contribute/#dev_call).

> **Note** The dev call is open to all interested developers (not just the core dev team). This is a great opportunity to meet the team and contribute to the ongoing development of the platform. It includes a QA session for newcomers. All regular calls are listed in the [Dronecode calendar](https://www.dronecode.org/calendar/).


## Maintenance Team

  * Project: Founder - [Lorenz Meier](https://github.com/LorenzMeier), Architecture: [Daniel Agar](https://github.com/dagar)
    * [Dev Call](https://github.com/PX4/Firmware/labels/devcall) - [Ramon Roche](https://github.com/mrpollo)
  * Communication Architecture
    * [Beat Kueng](https://github.com/bkueng)
    * [Julian Oes](https://github.com/JulianOes)
  * UI in QGroundControl
    * [Gus Grubba](https://github.com/dogmaphobic)
  * [Multicopter Flight Control](https://github.com/PX4/Firmware/labels/multicopter)
    * [Mathieu Bresciani](https://github.com/bresch)
  * [Multicopter Software Architecture](https://github.com/PX4/Firmware/labels/multicopter)
    * [Matthias Grob](https://github.com/MaEtUgR)
  * [VTOL Flight Control](https://github.com/PX4/Firmware/labels/vtol)
    * [Roman Bapst](https://github.com/RomanBapst)
  * [Fixed Wing Flight Control](https://github.com/PX4/Firmware/labels/fixedwing)
    * [Roman Bapst](https://github.com/RomanBapst)
  * OS / NuttX [David Sidrane](https://github.com/davids5)
  * Driver Architecture [Daniel Agar](https://github.com/dagar)
  * Commander Architecture [Julian Oes](https://github.com/julianoes)
  * [UAVCAN](https://github.com/PX4/Firmware/labels/uavcan) [Daniel Agar](https://github.com/dagar)
  * [State Estimation](https://github.com/PX4/Firmware/issues?q=is%3Aopen+is%3Aissue+label%3A%22state+estimation%22) - [Paul Riseborough](https://github.com/priseborough)
  * Vision based navigation
    * [Julian Kent](https://github.com/jkflying)
  * Obstacle Avoidance - [Martina Rivizzigno](https://github.com/mrivi)
  * RTPS/ROS2 Interface - [Nuno Marques](https://github.com/TSC21)

See also [maintainers list](https://px4.io/community/maintainers/) (px4.io) and the [contributors list](https://github.com/PX4/Firmware/graphs/contributors) (Github).

## Supported Hardware

This repository contains code supporting these boards:
  * FMUv2
    * [Pixhawk](https://docs.px4.io/master/en/flight_controller/pixhawk.html)
    * [Pixfalcon](https://docs.px4.io/master/en/flight_controller/pixfalcon.html)
  * FMUv3
    * [Pixhawk 2](https://docs.px4.io/master/en/flight_controller/pixhawk-2.html)
    * [Pixhawk Mini](https://docs.px4.io/master/en/flight_controller/pixhawk_mini.html)
    * [CUAV Pixhack v3](https://docs.px4.io/master/en/flight_controller/pixhack_v3.html)
  * FMUv4
    * [Pixracer](https://docs.px4.io/master/en/flight_controller/pixracer.html)
    * [Pixhawk 3 Pro](https://docs.px4.io/master/en/flight_controller/pixhawk3_pro.html)
  * FMUv5 (ARM Cortex M7)
    * [Pixhawk 4](https://docs.px4.io/master/en/flight_controller/pixhawk4.html)
    * [Pixhawk 4 mini](https://docs.px4.io/master/en/flight_controller/pixhawk4_mini.html)
    * [CUAV V5+](https://docs.px4.io/master/en/flight_controller/cuav_v5_plus.html)
    * [CUAV V5 nano](https://docs.px4.io/master/en/flight_controller/cuav_v5_nano.html)
  * [Airmind MindPX V2.8](http://www.mindpx.net/assets/accessories/UserGuide_MindPX.pdf)
  * [Airmind MindRacer V1.2](http://mindpx.net/assets/accessories/mindracer_user_guide_v1.2.pdf)
  * [Bitcraze Crazyflie 2.0](https://docs.px4.io/master/en/flight_controller/crazyflie2.html)
  * [Omnibus F4 SD](https://docs.px4.io/master/en/flight_controller/omnibus_f4_sd.html)
  * [Holybro Durandal](https://docs.px4.io/master/en/flight_controller/durandal.html)
  * [Holybro Kakute F7](https://docs.px4.io/master/en/flight_controller/kakutef7.html)
  * [Raspberry PI with Navio 2](https://docs.px4.io/master/en/flight_controller/raspberry_pi_navio2.html)

Additional information about supported hardware can be found in [PX4 user Guide > Autopilot Hardware](https://docs.px4.io/master/en/flight_controller/).

## Project Roadmap

A high level project roadmap is available [here](https://www.dronecode.org/roadmap/).
