# UI 导航7源码同步（2026-10-09）

本次将机器人上正在用于 UI「7.双雷达NeuPAN导航」的源码同步进版本控制，基于原 main 提交 29200aa0aceacf1d722795f73b8d3577d0529fdc。

## 实际启动链路

- UI 配置：`~/rpp_tools/starterbutton/.starterButton/config/config.json`，导航7注入 `RPP_NEUPAN_SCAN_SOURCE=dual RPP_NEUPAN_USE_MESH_SELF_FILTER=true`。
- 入口：`~/rpp_tools/starterbutton/.bin/start_navigation_high_sampling_neupan.sh`。
- 脚本加载 `~/.rpprc`，使用 `/opt/ros/humble` 和 `~/rpp_ws/install/local_setup.bash`。
- 启动 `navigation_2d/local_and_nav_high_sampling_controller.launch.py`，选择 `local_controller_mode:=neupan`，进入 `local_and_nav_high_sampling_neupan.launch.py`。
- 导航源：`~/rpp_ws/src/application/navigation_2d`；NeuPAN、桥接、双雷达和保护包源：`~/rpp_ws/src/application/neupan_migration`。
- `tools/starterbutton/` 保存导航7入口、两个辅助文件及按钮配置摘录。仅作为入口源码记录；本次没有执行启动脚本。

`docs/navigation7_source_manifest.json` 记录选定导航包、NeuPAN 内核和三个入口文件的来源与 SHA-256。本次同步后逐文件匹配；仓库既有但源目录不包含的辅助文件保留，清单没有宣称完整 install 二进制身份。

## 相对旧仓库的主要变化

- 桥接器在新路径起点距旧路线超过 0.15 m 时重新发布路径；每次发布同步推进路径与目标版本。它缓解了偏离旧路线后无法重新接入的问题。
- Python 侧增加路径权威目标模式及目标/路径消息先后顺序回归，减少上下文版本不同步。
- 增加最新输入调度、双向命令稳定和 gear 切换处理、路径校验。
- 扩展机械臂几何投影、自身雷达回波过滤、动态 footprint 同步与通道净空诊断。
- 更新定位健康监测、保护后的恢复状态和局部/全局代价地图配置。
- 自动检查真实双臂状态流，再进入导航；新增相应脚本与测试。

这是机器人当前工程版本的归档，不是新科研算法，也不代表已完成科研强基线验收。

## 科研复用仍需注意

1. 路径更新判据仍主要检查新起点离旧路线的距离，不比较未来路线的实质变化。同目标、起点仍在旧路线附近时的左右换路仍需专门验证。
2. 当前是 Ubuntu 22.04/Humble 工程。Ubuntu 24.04/Jazzy 需要隔离迁移，不能把本次源码同步当成 Jazzy 构建成功。
3. 保留现有 R1 真机参数与部署约束；科研配置另设固定公开几何和统一运动/评价协议。集成 DUNE 权重与 Limo 权重相同的问题仍需几何验证。
4. UI 入口依赖厂商机器人描述、机械校准、雷达/底盘驱动、外部定位和地图等设备资料；这些运行资料没有因本次同步自动变成可移植实验依赖。
5. 命令稳定、保护和恢复机制会改变执行行为，研究对比要公平配置并记录介入情况。

## 本次验证

- `git diff --check` 通过。
- 120 个 Python 源文件语法解析及两个 shell 入口文件 `bash -n` 通过。
- 使用 Humble 和现有 Python 环境，在同步后的仓库源码上运行六项离线检查，通过：latest_input_scheduler、command_stabilizer、path_validation、n1_gear_context、n2_scan_self_filter、nav2_goal_order。
- 未启动导航、未发布运动或关节命令；真机闭环效果没有在本次重新验收。
- Humble 隔离构建四个 C++ 包通过：navigation_2d、navigation_neupan_bridge、r1_lidar_preprocess、r1_collision_guard。独立 build/install 位于 `/tmp/r1-navigation-sync-validation/`，没有替换正在部署的工作区。
- 新构建的桥接与保护测试共 10 个 gtest 用例全部通过；Python 包运行依赖仍使用机器现有环境。
- 提交前复核清单中的 364 个源码/入口文件，仓库快照和源文件哈希均匹配。

运行日志、抓取结果、Python runtime 报告和本机校准文件没有加入本次提交。
