# HW1 collision check 调整收尾记录（2026-10-29）

代码提交：`0e4b48e56909bfad23cf606b631278f498ef69c5`。
用户最新要求：先 Git 提交和收尾，暂不实机验收、暂不切换生产 UI7；CogNav 不修改。

HW1/UI7 候选路径移除额外的障碍碰撞检测：Guard 不再构造碰撞几何逻辑，
不订阅自身 scan、动态 footprint 或 mesh；三个 Launch 入口和 UI7 脚本只允许
collision_check_enabled=false。最终同名节点保留 A+ 授权、Worker/源/命令超时
停车、有限值/速度限制和里程计/定位检查。NeuPAN 算法和自身避障输入未改。

已完成必要验证：独立 Guard 构建、授权回归 15/15、入口/UI7 4/4；
整套 8 包 BUILD_TESTING=OFF 构建成功，candidate07 清单 492 文件；
bundle08 清单 651 文件，实际安装链 3/3（无额外几何输入可运行、FATAL 停车、
Worker 扫描丢失停车），包内 Worker/solver 来源核对无错误。
未重复全量回归。旧 Guard 关闭开关仍受几何数据超时阻断的基线失败保留。

整套候选：`/var/tmp/r1-neupan-candidate07-20261029`。
固定目录：`/home/rpp/neupan_failstop_ws/release_bundles/20261029-a-plus-08`。
固定目录 payload 与通过上述 3 项测试的 bundle 文件哈希一致，重定位兼容检查、
UI7 输入追踪、实际 Controller/core/plugin/Guard maps 核对通过。
构建源和工具源均为上述代码提交；本记录是后续文档提交，不修改该发布包。

此前按现场急停确认执行了无目标、未 arm、最终速度输出隔离的启动观察：
491 个隔离输出样本全零，观察时真实 /cmd_vel 无发布者。
首次终端缺少图形环境导致 RViz 失败；使用 UI 的 DISPLAY/XAUTHORITY 后正常打开。
定位仍等待 map→base；未给初始位姿、未测试导航运动。
实际启动的定位缺失、costmap inflation 提示及停止阶段异常均保留在原日志，
不得以局部链路通过关闭这些问题或声称整套实机可用。

当前 NOT_DEPLOYABLE：所有发布批准和实机门禁保持 false，生产入口未切换。
按用户“先不实机、先收尾”要求停止本轮禁运动实例，保留原有底盘驱动。
后续现场任务：确认定位与原故障恢复流程、必要的发布/回滚核对及运动验收；
历史 DDS/健康间歇问题仍待关闭。NeuPAN 避障不能用作零风险验收结论。

原方案已在原附件路径修订。详细报告和所有原始失败/通过日志位于：
`/home/rpp/neupan_failstop_ws/reports/neupan_failstop_20261027/wp7-collisionoff-review.md`。
