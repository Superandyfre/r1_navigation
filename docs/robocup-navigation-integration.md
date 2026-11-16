# RoboCup 机器人导航对接说明

本文面向任务规划、视觉、机械臂、底盘和测试同学，介绍导航的调用方式、代码入口及故障排查。核对日期：2026-11-16。

## 一、版本与分工

当前导航按钮指向机器人上的发布包：

```text
/home/rpp/neupan_failstop_ws/robustness/bundle-motion-c0913d8
```

该发布包的源码版本为 `c0913d8`。机器人源码仓库为 `/home/rpp/r1_navigation`，其 `main` 上后续提交 `bc9b179` 尚未部署到当前发布包。**改动源码或编译源码不会自动改变按钮启动的版本。** 下文描述的是当前发布包，不表示导航此刻正在运行。

导航负责接收目标、利用地图与定位生成路径、通过 NeuPAN 计算局部轨迹并输出底盘运动指令。任务模块负责选择目标、管理任务顺序和处理结果；机械臂和底盘模块负责各自设备与状态接口。导航不会自行决定比赛任务目标。

## 二、启动和运行前准备

1. 按现场流程准备底盘、定位、前后雷达、机械臂驱动及地图，确认机器人周围环境允许运动。
2. 使用机器人操作界面的导航按钮启动。入口会检查发布包和是否已有冲突的导航进程；不要同时启动第二套导航。
3. 定位重启后按现场流程设置初始位姿，在地图中核对机器人位置和朝向。进程启动成功不代表定位已经可用。
4. 确认 `/navigate_to_pose` action 可用，再下发经过现场确认的目标。

机器人终端中可执行以下只读检查：

```bash
ros2 action list -t
ros2 topic list -t
ros2 topic info -v /odom
ros2 topic info -v /r1_localization_health
ros2 topic echo /r1_localization_health --once
ros2 topic echo /r1_collision_guard/status --once
ros2 topic echo /neupan_controller_status --once
```

状态话题可能因节点未启动或未发布而暂时没有输出，`echo --once` 会等待；终端中可用 Ctrl+C 结束等待。单个状态话题正常不能证明整个导航链路已经就绪。

## 三、任务模块如何下发目标

对接接口是 ROS 2 action `/navigate_to_pose`，类型 `nav2_msgs/action/NavigateToPose`。目标为 `PoseStamped`：

- 坐标系通常填写 `map`；`x`、`y` 使用当前地图坐标，单位为米。
- 平面朝向 `yaw` 单位为弧度，对应四元数 `z = sin(yaw/2)`、`w = cos(yaw/2)`；`x`、`y` 分量为零。
- 填写当前时间戳，确保目标坐标系和当前定位一致。

下例会提交真实运动目标，只用于说明调用格式；使用前必须替换为现场确认的坐标：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}' \
  --feedback
```

程序应等待 action server、判断目标是否被接受，并处理反馈和最终结果。状态码 4 表示成功、5 表示取消、6 表示失败。**速度为零不等于到达。** 当前配置的 Nav2 目标容差为位置 0.10 m、朝向 0.05 rad；需要更精确对准时，由上层另行设计对准流程。

多个目标由任务模块管理顺序和失败策略。执行中需要更换目标时，调用方应明确处理旧目标的取消或终止结果，不要假定并发目标会自动排队。

### Python 调用示例

将以下代码保存为 `send_nav_goal.py`，在已配置机器人 ROS 2 环境的终端执行 `python3 send_nav_goal.py`。代码会发送真实目标，运行前应先修改坐标。比赛程序还应加入符合任务需求的反馈、取消和超时处理。

```python
import math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose


def main():
    rclpy.init()
    node = Node('robocup_navigation_client')
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    try:
        if not client.wait_for_server(timeout_sec=5.0):
            node.get_logger().error('导航 action 不可用')
            return
        x, y, yaw = 1.0, 2.0, 0.0  # 修改为现场目标：米、弧度
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, sent)
        handle = sent.result()
        if handle is None or not handle.accepted:
            node.get_logger().error('目标未被接受')
            return
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result)
        node.get_logger().info(f'导航结束，状态码: {result.result().status}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 四、各模块对接约定

| 模块 | 对接内容 | 需要注意 |
| --- | --- | --- |
| 任务规划 | `/navigate_to_pose` action | 保存目标和任务编号，处理接受、反馈、成功、取消与失败。 |
| 视觉 | 提供目标位姿、坐标系和时间戳 | 将目标转换到 `map`；区分物体位置与机器人应到达的位置和朝向。 |
| 机械臂 | 驱动提供左右臂关节状态及 TF | 当前导航入口默认使用机械臂 3D 回波过滤；姿态、遮挡和工具会影响结果，未配置的工具不能假设已被覆盖。 |
| 底盘 | 提供 `/odom` 并执行速度命令 | 当前导航链路最终通常输出 `/cmd_vel`；遥控与导航必须协调控制权，不能同时争抢速度指令。 |
| 测试 | 地图、定位、导航状态和雷达数据 | 记录目标、时间、机器人和机械臂姿态、现场障碍及 action 结果。 |

当前发布包默认使用前方 Livox 和后方 2D 雷达。传感器有遮挡和盲区，不能假定各个方向和高度的障碍物都能被观察到。配置中的前进速度上限为 0.7 m/s、倒车速度上限为 0.3 m/s，实际速度由规划和控制情况决定。

`/neupan_path`、`/neupan_goal`、`/neupan_planner_result`、`/neupan_execution_context` 等用于导航内部协作或诊断。上层应通过 action 下发任务，不要向这些内部话题或 `/cmd_vel` 直接发布消息来替代导航任务接口。

## 五、代码与启动脚本

以下路径均为机器人本地路径，命令均在机器人本地终端执行：

| 位置 | 用途 |
| --- | --- |
| `/home/rpp/r1_navigation` | 主源码仓库。查询版本用 `git status -sb` 和 `git log -1 --oneline`。 |
| 发布包 `tools/starterbutton/start_navigation_high_sampling_neupan.sh` | 具体导航启动脚本：检查环境、包和进程，再调用 ROS launch。 |
| 发布包 `install/navigation_2d/share/navigation_2d/launch/local_and_nav_high_sampling_controller.launch.py` | 启动脚本调用的定位与导航组合入口。 |
| 源码 `packages/navigation_neupan_bridge/launch/r1_neupan_hw1_motion.launch.py` | 硬件导航、雷达处理和机械臂相关节点配置。 |
| 源码 `packages/navigation_neupan_bridge/config/planner_r1_hw1.yaml` | NeuPAN 参数；实际运行值请读取当前发布包 `install/navigation_neupan_bridge/share/navigation_neupan_bridge/config/` 下同名文件。 |
| 源码 `packages/navigation_neupan_bridge/config/nav2_params_r1_neupan_hw1.yaml` | Nav2 和目标容差配置；实际运行值以发布包安装目录中同名文件为准。 |
| 源码 `packages/r1_neupan_integration/r1_neupan_integration/n1_16_node.py`、`n1_15_whole_core.py` | 导航桥接和独立规划进程；发布包的对应 Python 代码在 `install/r1_neupan_integration/lib/python3.10/site-packages/r1_neupan_integration/`。 |
| 发布包 `third_party/NeuPAN/neupan/` | 当前发布版使用的 NeuPAN 代码。源码仓库下同名目录可能属于不同版本。 |
| 源码 `packages/navigation_neupan_bridge/src/r1_arm_self_filter_3d_shadow.cpp` | 3D 机械臂回波过滤实现；发布包运行对应的已编译程序。 |

推荐通过操作界面启动。以下命令是在机器人终端直接启动**固定发布包**的方法，会启动真实导航进程；该方式绕过按钮入口的版本指针和清单校验，不能替代日常入口。启动前确认没有冲突进程：

```bash
bash /home/rpp/neupan_failstop_ws/robustness/bundle-motion-c0913d8/tools/starterbutton/start_navigation_high_sampling_neupan.sh
```

此脚本默认启用双雷达及 3D 机械臂过滤，默认不启动机械臂驱动，使用 `/home/rpp/rpp_ws/config/r1_hw1_calibration.json`。脚本保持前台运行，并在退出时清理其启动的进程组；结束导航与遥控接管按现场流程协调。

查看源码和发布包参数：

```bash
cd /home/rpp/r1_navigation
git status -sb
git log -1 --oneline
cat /home/rpp/neupan_failstop_ws/robustness/bundle-motion-c0913d8/install/navigation_neupan_bridge/share/navigation_neupan_bridge/config/planner_r1_hw1.yaml
```

源码修改、构建、生成发布包和切换运行版本由导航负责人统一管理；不要直接修改现有发布包。

## 六、故障反馈

- **目标后不动**：先检查 action 是否接受、反馈及结果，再看定位、目标坐标、导航节点和控制器状态；不要只反复重发目标。
- **倒车接近障碍**：检查后雷达数据、安装遮挡及盲区，结合现场障碍位置和扫描画面记录。停止或遥控接管遵循现场控制权流程。
- **空旷区域有障碍点**：对照原始和过滤后点云以及机械臂实际姿态，记录时间；不要直接认定是回波。
- **遥控脱困后仍卡住**：确认遥控已释放控制权，记录旧任务和新任务的状态以及定位信息，由导航负责人排查。
- **启动报冲突或校验失败**：保存完整终端输出，交导航负责人处理，不要绕过检查。

反馈时请附：发生时间、目标坐标和朝向、action 状态、机器人与机械臂姿态、障碍物实际位置、是否遥控过、启动日志及可用的录包或截图。本文不是实机运动验证报告；切换发布包后需重新核对。
