# UI7 导航鲁棒性：轮廓同步恢复进度

本次仅改变实时轮廓和 costmap 均存在且新鲜、但两者不一致时的处理：
先进入零速 HOLD，允许最多 1 秒的连续/间歇同步窗口；恢复后仍须经过原有 3 秒稳定窗口。
缺失或过期输入、定位跳变及其他异常仍按原故障策略处理。Guard 和 Controller
在 READY 后要求新输入和新结果，Guard 还要求两条速度流先各自观察到零速。

验证：独立构建 navigation_neupan_bridge、r1_collision_guard；3 个 C++ 测试组通过；
Guard 17 项隔离 ROS 测试通过；真实 ControllerServer、Controller 插件、速度平滑器、
Guard 的隔离链 HOLD 与恢复测试通过。后者使用模拟 Worker/传感器，不代表实机或真实
NeuPAN 求解的端到端验收。另用真实定位监视器及模拟 TF、里程计、轮廓和 costmap
验证短时不同步恢复与持续不同步锁存，尚未与真实 Worker 串成完整链。

构建输出及测试日志：`/home/rpp/neupan_failstop_ws/robustness/`。ROS 测试只用 188/192
隔离域和测试速度话题。此次未切换 UI7 发布指针或启动底盘；现有在役发布包不含本次改动。
完整鲁棒性方案其余项目仍待实施和验证。原主工作树未提交的 Clarabel 实验与事故日志
保存在带说明的 Git stash 中，不包含于 main；不能把它当成已验证的求解器修复。
