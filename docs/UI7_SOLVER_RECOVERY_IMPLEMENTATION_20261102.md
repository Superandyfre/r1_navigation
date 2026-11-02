# UI7 求解失败恢复实现与验证记录

日期：2026-11-02。对应方案：[UI7 求解失败恢复方案](UI7_SOLVER_FAILURE_RECOVERY_PLAN_20261030.md)。

## 1. 交付状态与适用边界

本轮实现了明确的数值失败分类、有界冷重试、新目标状态重置、跨组件撤权、一次 BT 重规划、受控运行重建、实际底盘无响应判定和 UI7 固定发布入口。没有修改 CogNav 生产代码、UI8、底盘及机械臂驱动。正常 UI7 仍按原来的真实双臂状态流程自动启动，不增加正常导航的手动 arm 步骤。最终重复碰撞检查保持关闭。

**尚未进行实机导航验收。** 固定现场锥规划回归和有限正常对照通过，不能据此声明所有 ECOS 输入均已修复。原现场完整有状态输入缺失，真实路径质量、长时间运行及现场脱困效果仍待验证。

额外的自主后退脱困没有启用：`reverse_escape_enabled=false`，NeuPAN 新增 `bypass_allow_backoff=false`。绕障候选如需负向运动会被拒绝，诊断为 `reverse_disabled`；候选和引导重建耗尽后任务有界退出。常规路径的倒车能力与额外自主后退脱困分别处理。尚未证明后向有效感知、完整机械臂轮廓扫掠和真实制动条件，不将“有限重规划通过”写成“完整运动脱困通过”。

## 2. 数值修复

现场文件：`/home/rpp/neupan_failstop_ws/reports/ui7-restart-stuck-20261029/ecos-failure-evidence/ecos-failure-592246-5.npz`。校验值、固定验收阈值和每次解的原尺度残差保存在 `reports/solver-recovery-20261102/cone-scaling.json`，该路径相对于 `/home/rpp/neupan_failstop_ws`。

现场锥规划的目标系数范围为 0.05–2000。NRMP 将整个目标函数统一乘以正数 0.1，全部约束、各项相对权重、几何净空及执行速度上限保持原值。该变换保持最优解集合，不提高迭代上限，不接受负返回码，不引入备用求解器。

| 对照 | 结果 |
| --- | --- |
| 现场原始锥规划，原目标尺度 | 10 次均在 100 次迭代处失败，`exitFlag=-1` |
| 同一锥规划，目标统一乘 0.1 | 10 次均 `exitFlag=0`，29 次迭代 |
| 缩放解，恢复原尺度检查 | 等式最大残差约 `8.88e-15`；原始锥约束最大违反约 `1.14e-9`；相对对偶残差约 `8.07e-15`；相对间隙约 `3.02e-11` |
| 两类正常输入，各 10 周期 | 两版本均无 stop；最大首指令差约 `3.94e-11` |
| 同一正常对照周期耗时 p50 / p95 / 最大 | 原版 24.6 / 48.3 / 50.0 ms；缩放后 21.0 / 40.0 / 40.4 ms |

这不是吞吐量基准；正常对照仅覆盖两个确定性场景。固定锥规划通过不替代现场完整路径、热启动、障碍映射及阶段切换验证。完整有状态故障上下文和更广的正常样本仍是 P3 的剩余验收内容。

重放入口：

```bash
/home/rpp/venvs/neupan_ros_humble/bin/python tools/ecos_diagnostics/validate_scaling.py \
  /absolute/ecos-failure.npz --output /absolute/cone-scaling.json
/home/rpp/venvs/neupan_ros_humble/bin/python tools/ecos_diagnostics/compare_normal_scaling.py \
  --output /absolute/normal-scaling.json
```

## 3. 失败与任务恢复契约

Worker 在进程内观察真实 `ecos.solve` 返回值。只有原始 `exitFlag=-1`、残差有限且对应明确 `diffcp.SolverError` 的迭代上限失败才进入 `recoverable_solver_failure`。没有对应底层证据、未知异常、IPC/身份冲突、无效数值和进程失效继续采用致命故障处理。绕障候选仅能局部拒绝明确不可行状态，不能吞掉其他求解异常。

失败响应携带请求/扫描/路径/目标/Worker 代次及原始求解信息，数值输出必须为空，`finite=false`。Bridge 检查这些身份和序列后抛出独立的可恢复异常，不隔离 Worker，也不把失败记成成功。

默认边界：

- 每个 FollowPath 执行最多一次冷重试；重试只用新输入，清理轨迹、绕障与求解缓存。
- 该执行累计恢复等待上限为 1 秒；成功停止恢复计时，但不补充已消耗的次数或时间预算。
- 预算耗尽后报告 `exhausted`，Controller 结束 FollowPath。现有 BT 最多重规划一次。
- BT 保存父请求预算，同一父请求最多两次失败执行；首次失败后，进入替代执行前还有 4 秒截止时间。正常路径修订不建立新的父任务。
- 新目标以新执行身份重置求解状态；重复终点坐标不等于新任务。旧结果不能跨身份生效。
- 同一执行的路径修订不会跳过已发生的求解失败计数；替换为新任务后迟到的失败则不会污染新任务。

Worker 健康协议升级为 `r1_neupan_worker_health_v2`，新增必需字段 `planner_state` 和 `planner_failed_request_id`。状态分别为 `ready`、`retry`、`exhausted`。Guard 收到失败后即使旧结果仍在时效内也不允许沿用；随后必须取得请求号高于失败请求的新有效来源。健康心跳不能代替规划结果或刷新旧结果时间。

Worker READY/响应同时校验 `r1_solver_recovery_v1`。Guard 状态声明健康协议，Controller 拒绝不匹配的 Guard。发布协议集为 `A+context1-source2-grant1-health2-owner1`，不得混用旧版组件。

## 4. 致命故障的受控运行重建

UI7 正常入口启动 `ui7_runtime_supervisor`，监督器从出生开始拥有独立导航进程组。这个组包括 Nav2、Worker、Guard 及其导航感知/轮廓辅助节点；底盘和机械臂驱动、地图服务与主定位在组外。不按进程名发现和杀死其他会话。

正常启动仍自动启用。只有故障救回使用以下入口：

```bash
ros2 service call /neupan_runtime/recover std_srvs/srv/Trigger '{}'
ros2 topic echo /neupan_runtime/recovery_status
```

取消未完成的恢复：

```bash
ros2 service call /neupan_runtime/cancel_recovery std_srvs/srv/Trigger '{}'
```

流程为：记录旧身份及首故障 → 通过 Controller/Guard 的只撤权入口停车 → 独立零输出与新鲜静止里程计连续确认 → 保存证据 → 停止并检查整个旧进程组及遗留子进程退出 → 用固定环境重建新运行组 → 新 Worker/Guard/Controller 身份、定位、扫描、完整轮廓和静止条件检查 → 内部启用空闲运行 → 等待新目标。没有旧目标发布或重放入口。

新运行组在恢复检查期间关闭自动启用，最后一步由监督器调用已有启用服务；这不是正常导航的额外人工操作。总期限 45 秒；服务发现/响应每次最多 1 秒；停车确认最多 3 秒；进程停止采用有界信号升级。重复请求被拒绝，取消和失败不会触发无限重启。

Guard 新增 `/r1_collision_guard/stop_for_recovery`，只能锁存撤权，不能清故障或授权。Guard 已崩溃时，Controller 撤权确认后由监督器发布独立零输出，仍须取得新鲜静止里程计，才能重建。监督器退出或意外死亡时，由运行组的父死亡信号与整组清理停止其子进程。

证据保存在 `/home/rpp/neupan_failstop_ws/reports/runtime-recovery/`。正常状态发布不扫描 `/proc`；只有停止和证据采集阶段检查组成员，避免影响时效回调。

## 5. 停滞与脱困

新增实际底盘无响应判定：在已授权的最终非零输出、新鲜且递增的 `/odom` 和正确坐标系同时存在时，如果连续 3 秒没有超过 0.01 m 的位移或 0.02 rad 的实际转角，结束当前执行并报告 `base_command_no_odometry_progress`。不增加速度，不盲目倒车，也不把“求解器输出非零”视为已经移动。

零输出等待会清除该观察窗口；合法转向、实际平移及取消不触发补偿运动。保留现有路径进展检查、绕障几何检查、两次有限引导重建和一次 BT 重规划。这个实现不代表任意局部受困都能自行脱离；新增运动脱困尚未启用。

## 6. UI7 发布入口与版本

固定入口：`/home/rpp/neupan_failstop_ws/ui7/start_navigation_ui7_current.sh`。它每次点击读取 `/home/rpp/neupan_failstop_ws/ui7/current.json`，校验清单、完整文件校验值和组件路径后执行固定发布包的原启动脚本。指针用原子替换切换，旧发布包保持不可变。入口只校验版本和文件完整性，不恢复已撤掉的人工发布门禁。

候选包：`/home/rpp/neupan_failstop_ws/release_bundles/20261102-solver-recovery-04`。

- 编译源码：`33205cd566e3c585f97c500223f640fe94e23900`。
- NeuPAN/工具源码：`aec45f697b08e162a7debdcbef66c7584954158d`。
- 8 包独立构建，`BUILD_TESTING=OFF`，完整文件清单随包保存。
- 实机验收状态始终为未完成；清单的物理验收标记不得改为通过。

固定入口已注册到 UI7；原 UI8 配置未修改。实际发布指针状态与最后验收清单见同目录 `UI7_SOLVER_RECOVERY_RELEASE_20261102.json`，不能仅根据本文候选包名称推断现场正在运行哪个版本。

## 7. 隔离验证与剩余工作

隔离 ROS 域为 192，`ROS_LOCALHOST_ONLY=1`；不启动物理底盘或机械臂驱动，不向实机域发布目标。测试使用真实 Worker/NeuPAN、Controller、速度平滑器、最终 Guard，以及需要时的真实 Navigator；传感器、定位和全局路径由测试夹具提供。求解失败注入发生在真实 IPC 调用之后，原始数值分类另由独立单元和现场锥规划重放验证，不能混淆这两种证据。

最终套件分别检查单次恢复、持续失败后新目标、父任务跨 FollowPath 的累计限制、Worker/Guard 崩溃后的受控恢复、底盘无响应及真正故障停车。每套检查安装文件校验值和实际 Worker/NeuPAN 加载来源。受控恢复夹具明确替代生产子启动命令，按等效生命周期步骤激活真实组件；它不等于生产完整启动和真实地图的现场验收。

最终 `solver-recovery-04` 上五类核心套件均有通过记录，实际加载来源检查无错误。BT 套件最初两次在故障注入前遇到 FollowPath ACK 超时；测试夹具增加 1.2 秒输入持续发布的启动发现等待后连续两次通过，未修改生产 ACK、健康或授权期限。这只能确认发现稳定后的恢复流程，不能证明生产冷启动 ACK 问题已解决；保留两次失败日志供现场启动验收对照。

实现保存在独立分支 `codex/ui7-solver-recovery`，未合入目标分支 `codex/navigation-environment-snapshots`。原主工作树仍有未提交的独立求解器实验，本轮没有覆盖、暂存或混入这些修改。构建源码与求解器/工具源码提交分别记录，不以文档提交代替发布包源码版本。

阶段记录：P1/P2 主链路实现；P2b 受控重建实现；P3 现场锥规划和有限正常对照通过、完整有状态现场回归待补；P4 无响应识别及有界退出实现、额外运动脱困未启用；P5 固定入口实现；P6 最终隔离验证结果随发布清单记录；P7 未执行。

下一次实机验收应先停止旧导航并确认静止，然后通过 UI7 启动完整新版本，确认定位 READY、真实双臂/完整轮廓与扫描有效，再由现场依次验证空旷短路径、单障碍、失败后新目标、取消和受阻停车。不要在真实运动任务中主动杀进程验证故障恢复。必须保留失败前后的现场数据；不能把本报告标记成实机通过。
