# V0-Research 安全同步、增量部署与回滚

以下为**用户未来人工执行**的命令，本次没有连接机载电脑或真实飞控。
所有操作先在拆桨、DISARM、导航未控制且飞手知情时进行。不要在飞行中部署/重启。
不执行git pull/reset/clean，不删除未跟踪文件，不覆盖声学采集Launch、自动rosbag或驱动。

## 1. 台式机审查与GitHub同步

本地起点 `feat/arducopter-v0-relative-waypoints@dcf3919`，本次未改变分支或提交。
原有未跟踪文件不属于本次修改，不能git add全部。实际新增测试/文档与逐文件清单见 `V0_RESEARCH_UPGRADE.md`。
在.git可写且人工审查工作树之后，才执行例如：

```bash
cd /path/to/indoor_nav_ws
git status --short
git diff --check
git switch -c feat/v0-research-simplification
# 只暂存明确的运行文件/文档白名单，未改文件不会产生提交差异。
while IFS= read -r path; do git add -- "$path"; done < docs/v0/V0_RESEARCH_DEPLOY_FILES.txt
git add -- tests/test_v0_research.py tests/test_v0_research_ros.py tests/test_v0_streams_ros.py
git add -- tools/v0_sitl_test.py tools/v0_sitl_suite.py
# 证据目录若要归档，先检查大小/敏感信息，再显式选择；不要提交历史未跟踪目录。
git diff --cached --stat
git diff --cached
# 人工批准后才提交；本次助手没有执行：
git commit -m 'feat(v0): research policies and readonly onboard USB integration'
# 再次人工确认远程/分支后才同步，不是本次自动动作：
git push -u origin feat/v0-research-simplification
```

GitHub仅作同步备份。机载现场存在未提交修复时，不能直接checkout/pull覆盖；优先下面的带哈希增量包。
未来若采用Git同步，也须先备份/diff，fetch后审阅提交并逐文件合并，不把开发机旧采集代码带上机。

## 2. 台式机制作白名单包

仍在本地仓库根目录，审核最新测试报告及白名单：

```bash
WS="$PWD"
STAMP="$(date +%Y%m%dT%H%M%S)"
OUT="/tmp/v0-research-$STAMP"
mkdir -p "$OUT"
while IFS= read -r path; do sha256sum "$path"; done \
  < docs/v0/V0_RESEARCH_DEPLOY_FILES.txt > "$OUT/SHA256SUMS"
tar -czf "$OUT/v0-research.tgz" -T docs/v0/V0_RESEARCH_DEPLOY_FILES.txt
sha256sum "$OUT/v0-research.tgz" > "$OUT/BUNDLE_SHA256"
printf 'Bundle: %s\n' "$OUT"
# 人工设置真实主机名，执行前再核对目的地址；本次没有执行此传输。
ONBOARD_HOST=replace-with-reviewed-hostname
scp "$OUT/v0-research.tgz" "$OUT/SHA256SUMS" "$OUT/BUNDLE_SHA256" \
  "sunrise@$ONBOARD_HOST:~/"
```

不打包build/devel/install、SITL脚本/参数、EEPROM、旧证据、大bag或声学系统。主机名不得照抄占位符。

## 3. 机载先暂存、备份、比较（不覆盖运行文件）

以下切换到机载终端；工作区可通过WS覆盖，不依赖固定账号。

```bash
WS="${WS:-$HOME/ros_indoor_nav}"
RUN="$HOME/v0-upgrade-$(date +%Y%m%dT%H%M%S)"
INCOMING="$RUN/incoming"
REVIEW="$RUN/reviewed"
mkdir -p "$INCOMING" "$REVIEW"
read -r EXPECTED_HASH HASH_PATH < "$HOME/BUNDLE_SHA256"
printf '%s  %s\n' "$EXPECTED_HASH" "$HOME/v0-research.tgz" | sha256sum -c - || exit 1
# 再与台式机显示的包SHA256人工比较，确认传输来源后再解包。
tar -xzf "$HOME/v0-research.tgz" -C "$INCOMING"
cp "$HOME/SHA256SUMS" "$RUN/SHA256SUMS"
(cd "$INCOMING" && sha256sum -c "$RUN/SHA256SUMS") || exit 1
LIST="$INCOMING/docs/v0/V0_RESEARCH_DEPLOY_FILES.txt"
test -d "$WS/src/indoor_nav" || exit 1
cd "$WS"
git status --short > "$RUN/git-status-before.txt" 2>&1 || true
git rev-parse HEAD > "$RUN/git-head-before.txt" 2>&1 || true
: > "$RUN/existing.txt"
: > "$RUN/absent.txt"
while IFS= read -r path; do
  if [ -e "$WS/$path" ]; then
    test -f "$WS/$path" && test ! -L "$WS/$path" || exit 1
    printf '%s\n' "$path" >> "$RUN/existing.txt"
  else
    printf '%s\n' "$path" >> "$RUN/absent.txt"
  fi
done < "$LIST"
tar -czf "$RUN/before-existing.tgz" -C "$WS" -T "$RUN/existing.txt"
while IFS= read -r path; do sha256sum "$WS/$path"; done \
  < "$RUN/existing.txt" > "$RUN/before-existing.sha256"
cp -a "$INCOMING/." "$REVIEW/"
while IFS= read -r path; do
  printf '\n===== %s =====\n' "$path"
  if [ -f "$WS/$path" ]; then diff -u "$WS/$path" "$INCOMING/$path" || true; fi
done < "$LIST" > "$RUN/merge-review.diff"
less "$RUN/merge-review.diff"
printf 'Retain this RUN path for rollback: %s\n' "$RUN"
```

若遇目录、符号链接或非普通文件，先停止人工检查，不跟随链接覆盖其它路径。
保护清单只包含导航文件/明确文档，未列出的机载现场文件完全不动。

## 4. 人工合并闸门

**停在此处逐文件审核，不可直接把INCOMING覆盖到WS。** 在REVIEW内完成合并：

- 保留机载真实USB URL、已有mission_file透传、respawn=false、两把只读开关与五组stream请求修复。
- 已有MAVROS时启动命令用start_mavros=false；不得新增第二FCU连接。
- `mission_v0.yaml`、`mission_v0_level.yaml` 默认保留现场原件供兼容加载，不为匹配历史hash重建。
- 必须确认左/右航点方向。交接中的负右向YAML与历史负ENU y输出不一致，详见升级说明；不要改坐标变换来凑数。
- 新research YAML可独立添加，逐字段核对飞行净空、相对高度、围栏与策略后才允许未来控制试验。
- CMake/package.xml只合入导航增量，保留机载可能已有其它安装项/依赖。
- 不编辑采集系统、自动rosbag、传感器驱动或failsafe。所有实际合并后的文件必须留存以便复现。

默认保留两个现场任务（只修改REVIEW，不改WS）：

```bash
for name in mission_v0.yaml mission_v0_level.yaml; do
  path="src/indoor_nav/config/$name"
  if [ -f "$WS/$path" ]; then cp -p "$WS/$path" "$REVIEW/$path"; fi
done
# 使用编辑器逐一合并其它差异到REVIEW，并复核解析/方向；不要只覆盖旧Launch。
# 全部清单条目审核后才建立批准清单；未批准条目应从该清单移除。
cp "$LIST" "$RUN/approved-files.txt"
```

若无法证明现场差异已合并，停止部署。机载历史level原件的真实hash应另存，不采用开发机重建值冒充。

## 5. 批准后的增量部署、权限与构建

先停止**经确认属于导航的两个节点**，保持DISARM。不要停止/重启声学采集或已有MAVROS。
不要使用killall、广泛pkill或删除现场文件。构建时确保没有其它catkin构建在运行。

```bash
read -r -p 'All diffs merged, DISARMED, navigation stopped? type DEPLOY_REVIEWED_V0: ' CONFIRM
test "$CONFIRM" = DEPLOY_REVIEWED_V0 || exit 1
cd "$WS"
sha256sum -c "$RUN/before-existing.sha256" || exit 1
while IFS= read -r path; do
  grep -Fxq -- "$path" "$LIST" || exit 1
  test -f "$REVIEW/$path" && test ! -L "$REVIEW/$path" || exit 1
done < "$RUN/approved-files.txt"
while IFS= read -r path; do
  mkdir -p "$(dirname "$WS/$path")"
  cp -p "$REVIEW/$path" "$WS/$path" || exit 1
done < "$RUN/approved-files.txt"
chmod +x src/indoor_nav/scripts/relative_mission_manager.py \
  src/indoor_nav/scripts/v0_nav_health.py src/indoor_nav/scripts/v0_request_streams.py
while IFS= read -r path; do sha256sum "$WS/$path"; done \
  < "$RUN/approved-files.txt" > "$RUN/deployed.sha256"
source /opt/ros/noetic/setup.bash
catkin_make -j2 || exit 1
source devel/setup.bash
python3 - <<'PY'
from pathlib import Path
import rospkg
from indoor_nav.v0 import load_config, relative_target
root = Path(rospkg.RosPack().get_path('indoor_nav'))
for name in ('mission_v0.yaml', 'mission_v0_level.yaml', 'mission_v0_research.yaml'):
    mission = load_config((root / 'config' / name).read_text())
    print(name, mission['start_policy'], mission['motion'], mission['arrival'])
    print([relative_target((0., 0., 1.), 0., waypoint) for waypoint in mission['waypoints']])
PY
```

这里使用rospack定位，不依赖台式机路径或tools是否部署。解析/构建失败先回滚，不为验证而ARM。

## 6. 一键只读验证

下列基础命令用于尚无MAVROS实例的情况；若已有all_sensors或其它采集入口提供MAVROS，
**必须追加 `start_mavros:=false`**。先确认ROS_MASTER_URI为现场Master，不继承SITL环境；use_sim_time不得为true。

```bash
cd ~/ros_indoor_nav
source /opt/ros/noetic/setup.bash
source devel/setup.bash
printf '%s\n' "$ROS_MASTER_URI"
rosnode list
rosparam get /use_sim_time
# 默认严格level，DISARM下只读，不调用start、不切模式。
PYTHONNOUSERSITE=1 roslaunch indoor_nav indoor_nav_real_v0.launch \
  enable_control:=false dry_run:=true
```

若WS不是上述默认路径，使用 `cd "$WS"`。已有MAVROS的科研配置只读示例（替代上面的launch，不叠加）：

```bash
PYTHONNOUSERSITE=1 roslaunch indoor_nav indoor_nav_real_v0.launch \
  start_mavros:=false \
  mission_file:="$(rospack find indoor_nav)/config/mission_v0_research.yaml" \
  enable_control:=false dry_run:=true
```

同一现场Master的另一终端：

```bash
source /opt/ros/noetic/setup.bash
WS="${WS:-$HOME/ros_indoor_nav}"
source "$WS/devel/setup.bash"
rosparam get /v0_nav_manager/mission_file
rosparam get /v0_nav_watchdog/mission_file
rosparam get /v0_nav_manager/enable_control
rosparam get /v0_nav_watchdog/dry_run
rostopic echo -n 1 /v0_nav/status
rostopic echo -n 1 /v0_nav/watchdog
timeout 35s rostopic hz /mavros/global_position/raw/fix
rostopic echo -n 1 /mavros/local_position/pose
rostopic echo -n 1 /mavros/local_position/velocity_local
timeout 15s rostopic hz /mavros/local_position/pose
timeout 15s rostopic hz /mavros/local_position/velocity_local
timeout 15s rostopic hz /mavros/imu/data
rostopic info /mavros/setpoint_position/local
rosservice list | grep '^/v0_nav/'
rosparam get /mavros/fcu_url
lsof /dev/ttyACM0
```

核对两节点同一配置、control_enabled=false、固件4.5.7、EKF健康及参数快照、持续新鲜定位。
DISARMED在只读检查中是正确拒绝原因，不应为消除它而ARM。
只读模式Watchdog不创建FC目标publisher；topic可能不存在，或只有其它系统订阅者，不能有导航控制发布。
GPS一帧有效不代表长期可靠；测量最大更新间隔，若超过gps_timeout_s应诊断通信，不伪造时间戳/连续旧位置。
服务完成不等于请求频率达标；若不更新，保留日志并停止升级。timeout返回124仅表示观测窗口到期。

## 7. 回滚（保留新增文件，不删除）

先确认DISARM、停止本次导航入口、没有活动控制输出。使用前面记录的绝对RUN路径。
若外层入口负责MAVROS，停止入口也会断开该MAVROS，必须先安排采集/飞手确认；已有采集MAVROS不得被误停。

```bash
WS="${WS:-$HOME/ros_indoor_nav}"
RUN=/absolute/path/to/the-reviewed-v0-upgrade-directory
test -f "$RUN/before-existing.tgz" && test -f "$RUN/approved-files.txt" || exit 1
mkdir -p "$RUN/pre-rollback" "$RUN/added-files-retained"
while IFS= read -r path; do
  grep -Fxq -- "$path" "$RUN/incoming/docs/v0/V0_RESEARCH_DEPLOY_FILES.txt" || exit 1
  if [ -f "$WS/$path" ]; then
    mkdir -p "$(dirname "$RUN/pre-rollback/$path")"
    cp -p "$WS/$path" "$RUN/pre-rollback/$path"
  fi
done < "$RUN/approved-files.txt"
while IFS= read -r path; do
  if grep -Fxq -- "$path" "$RUN/approved-files.txt" && [ -f "$WS/$path" ]; then
    mkdir -p "$(dirname "$RUN/added-files-retained/$path")"
    mv "$WS/$path" "$RUN/added-files-retained/$path"
  fi
done < "$RUN/absent.txt"
tar -xzf "$RUN/before-existing.tgz" -C "$WS"
sha256sum -c "$RUN/before-existing.sha256" || exit 1
cd "$WS"
source /opt/ros/noetic/setup.bash
catkin_make -j2 || exit 1
source devel/setup.bash
# 仅用已恢复且现场审核的旧入口进行只读验证，不自动ARM/起飞。
```

新文件移动到归档而非删除；当前部署版另存pre-rollback。若备份后现场又有其它修改，回滚前先逐文件复核，
不要把后来用户修改覆盖掉。Git分支回退不代替文件备份；不使用reset --hard或clean。
