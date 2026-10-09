# indoor_nav: EKF3 ExternalNav cold-start patch (SITL only)

## What changed

- `world_estimator.py`: calibration, health and ODOMETRY no longer depend on `/mavros/local_position/pose`.
  - Independent FC attitude input: `/mavros/imu/data` (ROS ENU/FLU), even when local EKF XY has not initialized.
  - UWB supplies measured XY; orientation is forwarded from FC IMU strictly for ROS/MAVLink message consistency (NOT an independent yaw sensor).
  - Z comes from the range sensor (plus explicitly configured fixed `nav_z_offset_m`), NOT from FC EKF local Z. Keep `EK3_SRC1_POSZ=1` (Baro) for the first test.
  - Physical operator condition remains: nose must align with `world_lab +X` when calibrating. World-to-nav yaw is locked at calibration. No flight-time re-calibration.
  - Local FC pose subscriber remains only for optional diagnostics.
- `localization.yaml`: new `imu_attitude_topic`, `imu_attitude_timeout_s`, `world/nav_z_offset_m` options.
- `ardupilot_expected.yaml`: test-only expected values POSXY=6, VELXY=0, POSZ=1, VISO_TYPE=3. This does **not** write flight-controller params.
- `tools/set_ekf_origin_once.py`: **optional explicit** geographic EKF origin request with coordinates supplied by the operator. Never called automatically.
- `tests/test_cold_start_static.py`: offline sanity checks.

## Install patch

From the project root (`~/github/indoor_nav_ws`):

```bash
# Inspect diff before copying if desired; zip contains only changed/new files.
unzip -o /path/to/indoor_nav_cold_start_patch.zip -d ~/github/indoor_nav_ws
cd ~/github/indoor_nav_ws
python3 -m unittest discover -s tests -v
catkin_make
```

Keep previous Git commit as rollback point. Restart all SITL processes with the existing scripts after changing source/config.

## Step A: transport / cold-start isolation (no arm)

1. **Start with SITL GPS available**, but after tests pass explicitly change primary EKF source to ExternalNav.
2. Verify `/mavros/imu/data`, `/indoor_nav/sitl/mock_range` and mock UWB are publishing, and MAVROS has subscribed `/mavros/odometry/out`.
3. Check parameters in QGC/MAVROS (`rosrun mavros mavparam get ...` if available) and set only parameters that exist in the specific firmware. For ArduCopter ExternalNav test:

   - `AHRS_EKF_TYPE=3`
   - `EK3_ENABLE=1`
   - `VISO_TYPE=3` (ArduPilot external vision backend; verify availability)
   - `EK3_SRC1_POSXY=6`, `EK3_SRC1_VELXY=0`
   - `EK3_SRC1_POSZ=1`, `EK3_SRC1_VELZ=0`, `EK3_SRC1_YAW=1`

   Reboot the SITL flight controller after changing EKF source or VISO parameters. DO NOT ARM.
4. After reboot, **before calling calibration**, check whether local EKF position is absent. Calibration should now succeed regardless:

```bash
rostopic hz /mavros/imu/data
rostopic hz /indoor_nav/sitl/mock_range
rosservice call /indoor_nav/calibrate
rostopic echo -n1 /indoor_nav/localization_status
rostopic hz /mavros/odometry/out
rostopic echo -n1 /mavros/odometry/out
```

`tools/sitl.env` already includes `EXTERNAL_NAV_ENABLED=true` in the provided snapshot.

**Do not mistake `/mavros/odometry/out` 30Hz for EKF fusion success.** Independent evidence is required: FC DataFlash visual-odometry / visual-position messages, EKF innovations or consistent FC local position motion correlated with UWB after source selection.

## Step B: EKF origin without GPS

ArduPilot requires a geographic EKF origin for pure non-GPS startup. The geodetic origin is NOT a GPS position measurement. While connected and disarmed, if `/mavros/global_position/gp_origin` shows no valid origin, set it with **an operator-chosen, known reference**:

```bash
source /opt/ros/noetic/setup.bash
source ~/github/indoor_nav_ws/devel/setup.bash
python3 tools/set_ekf_origin_once.py --lat YOUR_LAT --lon YOUR_LON --alt YOUR_AMSL_ALT
rostopic echo -n1 /mavros/global_position/gp_origin
```

The sample SITL location often appears near -35.363...,149.165... but **do not blindly copy it or invent altitude**; use known SITL home reference, or a surveyed physical origin for real flight. A sent MAVLink message is not proof ArduPilot accepted the origin.

## Step C: final GPS-disabled simulation

Only after Step A and B have passing logs, disable SIM GPS with settings **supported by the exact SITL firmware** (inspect available SIM_GPS*/GPS* parameters first). Cold-reboot SITL with ExternalNav already selected. Check that local position appears after IMU+UWB+range -> calibration -> odometry -> EKF. Compare in a stationary/disarmed, manually moved Gazebo model against `/gazebo/model_states` and mock UWB; do not ARM for these tests.

## Safety and frame limitations

- `fixed_start_pose` aligns body nose to +X of the world at calibration. This is an operator physical assumption. Heading comes from FC IMU, so the code does NOT infer a survey-quality world heading itself.
- Gyro/attitude consistency and ENU/NED transforms require bench verification with +X/+Y translation and yaw before any flight.
- Optical flow is NOT simulated or enabled by this patch. No speed is inferred; twist covariances remain large.
- ROS covariance values are NOT a reliable guarantee that ArduPilot will ignore ODOMETRY Z or yaw. Validate FC actual fusion source / logs.
- The safety supervisor may not indicate READY until FC local pose exists; that is expected during EKF cold start.
- Ground tag T1 calibration, anchor map verification, range and timeouts still gate publication. Calibrate after restarting the node, before flying.
- No automatic source change, EKF origin setting, takeoff, arming, or geofence weakening is included.
