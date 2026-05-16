# Klevebrand Drone Position Library

Open-source C++ library for accurate drone velocity and position estimation using Kalman-predicted sensor fusion.

The library is intended to combine data from:

- `u-blox NEO-M9N` GNSS for global position and ground-speed data
- `BMP280` barometric pressure sensor for altitude estimation
- Gyroscope and acceleration input data for short-term motion tracking and prediction

The goal is to provide a practical position-estimation layer for drone projects that need smoother, more accurate motion data than raw sensor readings alone can provide.

## What it does

- Implements a drone position module in modern C++
- Uses barometric pressure data to estimate altitude
- Designed for Kalman-based prediction and correction of velocity and position
- Built to work alongside inertial and GNSS input streams
- Optimized for embedded Arduino and AVR-based environments

## Hardware focus

- `NEO-M9N`
- `BMP280`
- Gyroscope and accelerometer/IMU input
- AVR targets such as `ATmega2560`

## Project structure

This repository contains the drone-position implementation layer and builds on top of:

https://github.com/LordMax2/klevebrand-all-drone-controller

It is suited for projects that want a reusable library for sensor-driven drone position and velocity tracking rather than a full flight-controller application on its own.

## License

MIT License. See `LICENSE`.
