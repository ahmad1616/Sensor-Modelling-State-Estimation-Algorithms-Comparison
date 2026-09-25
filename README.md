# DBF Sensor Modeling and Estimation Pipeline

This document describes the end-to-end workflow for generating flight data from FlightGear, simulating realistic sensor measurements, calibrating them, and testing attitude estimation algorithms.

## Pipeline Overview

```
FlightGear ──► flightData.csv ──► uniformDataFile ──► simulatedSensorsData ──► calibratedSensorsData ──► Estimation Algorithms

```

## Workflow Steps

### 1. Configure FlightGear to Stream Flight States

1. Locate your FlightGear data directory. For example:
   `C:\Users\[Your User Name]\FlightGear\Downloads\fgdata_2024_1`

2. Inside that directory, open the `Protocol` folder and add a file named `flight_states`. This file defines which states FlightGear's telnet/generic server will stream.

3. In FlightGear's launcher, under **Settings → Additional Settings**, add the following command:

   ```
   --generic=socket,out,100,localhost,5500,udp,flight_states
   
   ```

   This configures FlightGear to:

   * Use the socket protocol streaming outwards.

   * Stream at a rate of **100 Hz**.

   * Send data to `localhost:5500` over **UDP**.

   * Use the `flight_states` protocol definition file.

### 2. Capture Flight Data

1. Launch FlightGear with the configuration above.

2. Run `getProperties.py` to capture the streamed states and write them to a CSV file named `flightData.csv`.

### 3. Generate Uniformly Sampled Data

Run `createUniformData` to interpolate `flightData.csv` onto a uniform time grid, producing `uniformDataFile`.

> ⚠️ **Known Issue: Angle Wraparound**
>
> Interpolating across the $0^\circ / 360^\circ$ wraparound boundary can produce artificial intermediate angle values that never occurred in the actual trajectory (e.g., interpolating between $359^\circ$ and $1^\circ$ may momentarily pass through $\approx 180^\circ$).
>
> **Resolution:** Correct this prior to interpolation by unwrapping the angular data (`unwrap`) or by interpolating the sine and cosine components instead of raw angles.

### 4. Simulate Sensor Measurements

Run `simulatedSensors` to inject realistic calibration errors and noise into each sensor channel, producing `simulatedSensorsData`.

* **IMU (Accelerometer / Gyroscope):** Derived from `flightData`'s true kinematic states, with scale-factor, misalignment, bias, and white-noise errors applied.

* **Magnetometer:** Simulated by projecting a known Earth magnetic field vector into the body frame using true Euler angles from `flightData`, then applying soft-iron, hard-iron, and measurement noise errors.

* **Barometer:** Simulated by converting true altitude from `flightData` into pressure via the ISA barometric formula, then applying static bias and noise errors.

### 5. Calibrate Sensors

Run `calibrateSensors` to invert the known deterministic error models (scale factor, misalignment, and static bias) and recover corrected measurements, producing `calibratedSensorsData`. This file serves as the primary input for all downstream state-estimation work.

### 6. Test Estimation Algorithms

Using `calibratedSensorsData`, evaluate and compare attitude and state estimation approaches:

* **Accelerometer-only** attitude estimation

* **Gyroscope-only** attitude estimation (dead reckoning)

* **Complementary filter** (gyroscope + accelerometer)

* **Kalman filter / Extended Kalman filter (EKF)** (gyroscope + accelerometer, optionally + magnetometer)

Each algorithm's output should be validated against the ground-truth Euler angles from `flightData` to quantify estimation error metrics.

## File Summary

| File | Produced By | Description | 
 | ----- | ----- | ----- | 
| **`flight_states`** | Manual setup | Defines the property layout for streamed FlightGear states. | 
| **`flightData.csv`** | `getProperties.py` | Raw, non-uniformly sampled flight states streamed from FlightGear. | 
| **`uniformDataFile`** | `createUniformData` | Uniformly resampled flight states (with angle-wrap corrections applied). | 
| **`simulatedSensorsData`** | `simulatedSensors` | Raw synthetic sensor measurements with injected errors and noise. | 
| **`calibratedSensorsData`** | `calibrateSensors` | Corrected sensor measurements ready for estimation algorithms. | 
| **`estimateAlgorithms`** | `estimateAlgorithms` | Implements and evaluates state/attitude estimation filters against ground truth. | 
