# Hydrobotics ROV: sensor fusion and control

Orientation estimation and pilot control software for a 6-DOF underwater ROV, written for Team Bath Hydrobotics.

The core of the repo is an Error-State Kalman Filter that fuses two IMUs with different characteristics: a BNO085 providing an absolute fused quaternion at 50 Hz, and an ICM-20948 providing raw gyro at 200 Hz. The filter estimates ICM gyro bias online rather than assuming it constant, so the high-rate output stays locked to the absolute reference instead of drifting between updates.

![IMU fusion output](docs/imu_fusion.png)

## Why this architecture

The BNO085 alone caps the update rate at 50 Hz and gives no visibility into estimate confidence. Running from raw sensors alone means rebuilding magnetometer handling that the BNO085 already does well. Cascading the two gets high-rate output while keeping an absolute reference, and makes gyro bias observable.

Sensor selection followed the same logic. A second BNO-family device would share magnetic interference and drift characteristics with the BNO085: both headings wrong by a similar amount at the same time is not redundancy. A BMI088 offers good low-drift raw output but carries no magnetometer, which would cost absolute yaw reference. The ICM-20948 gives raw 9-axis output, complementing the BNO085 rather than duplicating it. The filter currently uses its gyro and accelerometer; its magnetometer is available for an independent yaw measurement but is not yet used.

```
ICM-20948 gyro  ──200 Hz──▶  PREDICT  ─┐
                                        ├──▶  state: quaternion + gyro bias  ──▶  6-DOF state vector
BNO085 quaternion ──50 Hz──▶  CORRECT  ─┘
                                             bias fed back into predict
```

The accelerometer bypasses the filter entirely, since the BNO085 already fuses it for levelling. It is gravity-compensated, rotated to world frame using the fused attitude, and integrated separately for linear velocity, with a leak term and deadband to bound drift. Horizontal velocity still drifts without an external reference such as a DVL.

A Bar10 pressure sensor feeds a small 1-D Kalman filter for depth and vertical velocity, which is blended into the state vector when readings are fresh.

## Running it

No hardware required: mock sensor models with a known injected gyro bias let the full pipeline run anywhere.

```bash
pip install -r requirements.txt

# simulated sensors, 60 seconds, generates a plot
python3 imu_fusion.py --mock --duration 60 --plot

# real sensors over I2C
python3 imu_fusion.py --rate 50 --icm-rate 200 --plot

# sensors bridged through an STM32 over UART
python3 imu_fusion.py --stm32 /dev/ttyUSB0 --plot
```

Every run writes a CSV log and can render a diagnostic plot covering orientation, per-axis error, bias estimates, magnetometer calibration state, covariance trace and the divergence watchdog.

## Validation

Against a simulated IMU with an injected gyro bias of `[0.8, -0.5, 0.4]` deg/s unknown to the filter, the estimate converges to within 0.08 deg/s on all three axes in about 9 seconds from a cold start and stays there for a 300 s run. The ESKF tracks the BNO085 reference to 0.011° RMS on each axis, compared with 0.46–0.84° for the complementary filter baseline.

```bash
python3 imu_fusion.py --mock --duration 300 --plot --no-bias-load --no-bias-save
```

## A bug in the simulator, not the filter

An earlier version showed the bias estimate oscillating by about ±0.2 deg/s at the motion frequency. I initially attributed this to the first-order quaternion integration. A controlled comparison ruled that out: with the simulated vehicle held level the oscillation vanished, and under full motion it only appeared when the mock gyro output Euler-angle rates. A real gyro measures body rates, which differ from Euler-angle rates whenever roll or pitch is non-zero. The BNO085 reference stayed kinematically consistent, so the bias state absorbed the mismatch. The mock now converts Euler-angle rates to body rates, and the oscillation is gone.

## Known limitations

Validation is against simulated sensors only. The hardware arrived after the filter was written, so bench testing against the real ICM-20948 and BNO085 is the next step. The noise parameters (`Q_GYRO`, `Q_BIAS`, `R_MEAS`) are set for the simulator and will need retuning on real data.

## Design notes

**Magnetometer trust timeout.** Yaw is normally excluded from the correction until the BNO085 reports adequate magnetometer calibration. But an ROV sitting near thrusters and a steel frame can stay uncalibrated indefinitely, and yaw gyro bias is only observable through that correction. Past a timeout the filter trusts yaw anyway with inflated measurement noise, rather than leaving yaw unobserved forever.

**Complementary filter baseline.** A simple complementary filter runs alongside the ESKF and is logged for comparison. It exists to check the added complexity is justified rather than assumed. After the first 20 seconds, the ESKF's RMS attitude error against the simulated truth is 0.011° on each axis, against 0.84° (roll), 0.62° (pitch) and 0.46° (yaw) for the complementary filter. Most of that gap is bias: the complementary filter cannot estimate it, so it carries a steady offset of roughly bias times its time constant. Note its `ALPHA` is applied per update and is not timestep-aware, so it is only meaningful at a fixed rate.

**Joseph-form covariance update** for numerical stability, and a watchdog that flags sustained divergence between the fused estimate and the BNO085 reference.

## Repository layout

```
imu_fusion.py                ESKF, depth filter, state vector, logging and plotting
controller_server_updated.py Pilot input → 6-DOF velocity vector → TCP server
example_client.py            Minimal client for testing the control link
requirements.txt
docs/                        Diagnostic plots and diagrams
```

## Pilot control

Controller axes map to a 6-DOF body-frame velocity vector `[X, Y, Z, RX, RY, RZ]`, with deadzone filtering and per-axis inversion, packed as length-prefixed float32 and streamed over TCP at 50 Hz. Axis indices vary by controller and operating system, so the mapping constants at the top of the file may need adjusting; the startup printout lists available axes and buttons.

## Hardware

| Component | Role |
|---|---|
| BNO085 | Absolute orientation, internally fused, 50 Hz |
| ICM-20948 | Raw gyro and accelerometer, 200 Hz |
| Bar10 (MS5837) | Pressure and depth, 20 Hz |
| Jetson Orin Nano | Onboard compute |
| STM32 | Optional sensor bridge over UART |

## Licence

MIT.
