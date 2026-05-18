# robot voice task for ROS 2

这个包是 ROS1 `robot` 语音抓取任务包的 ROS 2 迁移版本，用在 Fetch stockroom Ignition Gazebo 仿真中。

## 功能

- 启动 stockroom 仿真、Nav2、MoveIt2、RViz 和语音任务节点。
- 提供 `/run_voice_task` 服务，输入音频文件路径后解析任务。
- 默认测试语句为：`从一号桌抓取胶水放到二号桌`。
- 机器人先导航到源位置，按 ROS1 逻辑把物体世界坐标转换到 `base_link` 后交给 MoveIt2 抓取，再导航到目标位置并放置物体。
- 任务启动时向 MoveIt2 planning scene 发布桌面和货架碰撞体，抓取目标物体本身不加入碰撞体。
- Fetch 的 Gazebo 初始关节值和 MoveIt2 `stow/home/ready` 组状态与 ROS1 pick/place 场景一致。
- 场景启动时直接加载 12 个任务物体：胶水、书本、小球、纸巾、零食、水杯、快递盒、剪刀、铅笔、外套、鼠标、充电器。

## 构建

```bash
cd /home/slz/ROS2/fetch_ros2
source /opt/ros/humble/setup.zsh
colcon build --symlink-install
source install/setup.zsh
```

## 运行

ROS1 原命令：

```bash
roslaunch robot voice_fetch_task.launch light_mode:=false launch_rviz:=true
rosservice call /run_voice_task "file_path: '/workspace/src/robot/test.wav'"
```

ROS2 对应命令：

```bash
cd /home/slz/ROS2/fetch_ros2
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch robot voice_fetch_task.launch.py
```

另开终端调用服务：

```bash
cd /home/slz/ROS2/fetch_ros2
source /opt/ros/humble/setup.zsh
source install/setup.zsh
FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/install/robot/share/robot/config/fastdds_no_shm.xml \
ros2 service call /run_voice_task robot/srv/SpeechNLUSrv "{file_path: '/workspace/src/robot/test.wav'}"
```

成功响应示例：

```text
success=True
raw_text='从一号桌抓取胶水放到二号桌'
target_object='glue'
source_location='table_1'
target_location='table_2'
message='task finished'
```

## 配置文件

- `config/object_instances.yaml`：物体名称、显示名、Gazebo model 名和初始桌位。
- `config/location_map.yaml`：Nav2 导航接近点。ROS2 中 `table_2` 接近点已按当前地图调整到可达位置。
- `config/table_surface_map.yaml`：物体在 Gazebo 世界中的桌面放置坐标。
- `config/fastdds_no_shm.xml`：禁用 FastDDS shared memory，避免 `/dev/shm/fastrtps_port*` 残留导致服务发现异常。

## 节点和服务

- `/speech_service`：ASR 服务，类型 `robot/srv/SpeechSrv`。
- `/speech_nlu_service`：语义解析服务，类型 `robot/srv/SpeechNLUSrv`。
- `/run_voice_task`：任务入口服务，类型 `robot/srv/SpeechNLUSrv`。
- `task_dispatcher.py`：串联 NLU、Nav2、MoveIt2 机械臂规划、夹爪控制和物体放置。

## 验证结果

已验证：

- Gazebo world 启动后直接存在 Fetch、12 个货架 bin、12 个 tag 和 12 个 `obj_*` 任务物体。
- 调用 `/run_voice_task` 后，机器人到达一号桌，执行抓取胶水动作，到达二号桌接近点并放置胶水。
- 服务返回 `success=True` 和 `message='task finished'`。

如果启动时出现 FastDDS shared-memory 端口错误，可先清理旧端口再启动：

```bash
find /dev/shm -maxdepth 1 \( -name 'fastrtps_*' -o -name 'fastrtps_port*' \) -user "$USER" -delete
ros2 daemon stop
ros2 daemon start
```
