# 外部接口核对与注意事项

- Nooploop ROS1 nlink_parser: https://github.com/nooploop-dev/nlink_parser
- `LinktrackNodeframe2.msg`: https://github.com/nooploop-dev/nlink_parser/blob/master/msg/LinktrackNodeframe2.msg
- `LinktrackNode2.msg`: https://github.com/nooploop-dev/nlink_parser/blob/master/msg/LinktrackNode2.msg
- `LinktrackAnchorframe0.msg`: https://github.com/nooploop-dev/nlink_parser/blob/master/msg/LinktrackAnchorframe0.msg
- `LinktrackTag.msg`: https://github.com/nooploop-dev/nlink_parser/blob/master/msg/LinktrackTag.msg
- MTF-01P official manual: https://micoair.cn/zh/docs/sensors/sensors/mtf-01p-sensors
- ArduPilot MicoAir MTF guide (4.7+ parameters differ): https://ardupilot.org/copter/docs/common-mtf-01.html
- ArduPilot EKF source selections: https://ardupilot.org/copter/docs/common-ekf-sources.html
- ArduPilot ExternalNav: https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html
- ArduPilot Guided local frame: https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
- MAVROS odometry ROS frames: https://github.com/mavlink/mavros/blob/ros2/docs/plugins/extras/odom.md (check ROS1 version too)
- Caution: ArduPilot issue reporting ODOMETRY Z influencing EKF even with RangeFinder POSZ: https://github.com/ArduPilot/ardupilot/issues/32506

文档中的 `RNGFND1_TYPE=10`、`FLOW_TYPE=5`、`EK3_SRC1_POSZ=2` 等应按实际固件重新确认；Nooploop P-A/P-B/P-C 子型号尚未确认。
