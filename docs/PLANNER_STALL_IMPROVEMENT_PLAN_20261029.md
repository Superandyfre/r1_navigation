# NeuPAN 规划输出停止更新：改进方案

状态：方案，尚未实施本文件中的运行代码改动。目标分支：`codex/navigation-environment-snapshots`。本方案不改 UI8 的 1016 恢复环境，不替换正在运行的进程。当前分支合入 A+ 后，UI7 注册为独立的 unified 发布包；修改源码后必须重新构建发布包，不能假定 UI7 已加载源码修改。

## 1. 事实、未知项与目标

2026-10-29 UI8 的本轮重现：

- 22:48:06.254 至 22:48:08.801 共 50 次 ECOS 调用：49 次 exitFlag=0，一次 exitFlag=10。10 表示可接受的近似最优解，并非求解失败。
- 最后一次求解耗时约 16.7 ms。此后 ECOS 没有新调用。
- 22:48:14.877 ControllerServer 报 `NeuPAN envelope stale`，随后 BT 报目标失败。
- 停住后只读观察四秒：`/scan_neupan` 41 帧、`/odom` 311 帧，而规划结果、语义和桥接状态均未收到新消息。
- Worker 仍存在，采样时阻塞在 socket 等待；只能说明那个采样时刻它没在执行 ECOS，不能据此推断所有时刻的完整线程状态。
- 原始证据位于 `/home/rpp/nav_restore_20261016/evidence/failure-20261029-2248/`。

因此，本轮不能归因于 ECOS 数值失败；已定位的故障边界是主节点未持续产生新的规划输出。线程异常、锁等待、输入通知或状态门控、结果后处理失败，仍须用证据区分。此前的 ECOS 报错属于另一条故障路径，不应混为同一根因。

本轮目标：让下次故障可回答“哪次请求、哪个阶段、哪条线程、什么异常/等待、哪个版本的输入”，修复确认的失效路径，并在没有非预期运动的前提下完成导航验收。不是通过延长超时掩盖停止更新。

## 2. 已有机制与真实缺口

已存在：

- `LatestInputScheduler` 将新输入合并为单个执行周期；已有通知次数、周期数、开始/完成时间。
- `n1_16_node.py::_planner_scheduler_error` 已使用 `_handle_fatal_error("scheduler_exception", ...)`，不再采用 1016 旧版错误的 logger 位置参数调用。
- `_handle_fatal_error` 已先锁存 WorkerHealth 故障，再发布健康状态、零指令和错误状态。
- 健康定时器有独立 callback group；Controller 和 Guard 对结果、输入、健康及授权分别检查时效。
- 主节点已有 `plan_usable` 检查，注释明确针对到达目标时空轨迹导致几何函数抛错的问题。
- ECOS 诊断已记录原始 info 和失败锥规划输入。

缺口：

- Worker 的求解成功计数不等于主节点已成功发布结果，缺少两者之间的阶段记录。
- scheduler 异常目前主要保留类型和简短消息，没有完整异常链、线程堆栈及对应输入。
- scheduler 的 on_error 回调自身失败时仍可能逃出线程；缺少该路径的独立兜底证据。
- 空轨迹的已有检查尚须验证能覆盖后续所有几何调用、结果封装及严格 JSON 序列化，不能仅凭注释宣称已修复本次故障。
- 现有心跳只证明执行器仍服务健康回调；不能证明规划周期仍有进展。Python 全局停顿时，Python 内部观察线程也不保证运行。
- live solver 日志不能回答求解之后发生的后处理异常。现有 requests/summary 有部分内容在退出时才写，不能成为唯一事故证据。

## 3. 改动一：建立按阶段的规划记录

新增 `r1_neupan_integration/planner_diagnostics.py`，在 `n1_16_node.py` 与 `n1_16_bridge.py` 加入轻量调用。记录以下阶段的进入、完成或退出原因：

| 阶段 | 边界 | 必须记录 |
| --- | --- | --- |
| `input_gate` | TF、任务上下文、雷达、里程计、轮廓检查 | 等待原因、数据源时间、路径/目标版本 |
| `input_snapshot` | 获取一致输入、完成拷贝 | scan_counter、形状、有限性、锁等待时间 |
| `worker_startup` | 启动到 READY | worker_generation、PID、启动预算 |
| `request_send` | IPC 请求发送 | request_id、cycle_id、上下文标识 |
| `worker_wait` | 请求发送到收到响应 | 接收/解码完成时间、现有超时预算 |
| `response_validate` | 协议和新鲜度校验 | 检查结果、丢弃原因 |
| `trajectory_validate` | action、轨迹、状态分类 | dtype/shape/有限性、arrive/stop |
| `postprocess` | 几何、速度整形、gear 决策 | 子阶段、输入摘要、决策类别 |
| `envelope_build` | 生成控制结果和来源元数据 | 字段校验、序列化完成时间 |
| `publish` | 现有授权检查和发布 | 首次/最近发布、请求版本、耗时 |
| `cycle_complete` | 周期结束 | 总耗时、是否发布、未发布原因 |

区分四种计数：通知、开始周期、Worker 成功响应、主节点有效发布；不得用任一计数替代另一个。分别维护 last_worker_success 与 last_valid_publish。

每条事件带 boot_id、attempt/Action UUID（可用时）、request_id、cycle_id、scan_counter、path_version、goal_version、worker_generation、线程名，以及 monotonic 和墙上时间。业务数据时间戳保持原值，不用记录时间替代来源新鲜度。

热路径只更新小对象和有界内存环，不打印完整数组、不写大 JSON、不等待日志写线程。正常情况每秒汇总两次，异常时导出最近 128 个阶段事件。日志吞吐或磁盘失败必须通过 dropped_records/disk_error 显示出来，不能静默丢失。

## 4. 改动二：异常证据和线程失效处理

修改 `latest_input_scheduler.py`、`n1_16_node.py`：

1. 在异常对象仍有效时生成异常类型、消息、完整 traceback 和 cause/context，记录当前阶段与周期标识。
2. 调度异常继续调用现有 fatal latch；第一条故障 ID 不被后续诊断错误覆盖。诊断写盘失败不能阻止现有故障停车链。
3. on_error 回调再抛错时，使用不依赖 ROS logger/state_lock 的最小兜底输出，记录原异常和处理异常；调度线程进入终止状态，外部截止机制仍负责撤权。禁止无提示消失，也不循环恢复失败的线程。
4. 每个周期的 active_request、阶段状态在 finally 中清理。加上线程 is_alive、最近调度进展状态，区分 WAITING_INPUT、RUNNING、FAULTED、STOPPED。
5. 可选进程级 faulthandler 只使用未占用信号，例如经启动检查后的 SIGUSR2；现有 SIGUSR1 用于冻结结果，必须保留。信号只有确认目标进程已注册 handler 后才能发送，避免默认信号动作杀进程。默认由内部失速检测申请堆栈快照，不依赖 ptrace/sudo。

不要把修复异常日志等同于修复最初业务异常。新增记录的作用是获得根因，而不是使旧故障自动继续运行。

## 5. 改动三：从求解结果到发布的边界验证

新增可独立离线测试的 `planner_output_validation.py`，明确以下契约：

- `arrive=true` 与空 opt_state：走到达终态分支，跳过要求三维状态的几何计算，按当前任务契约发布零结果/完成状态。不能将合法到达当作 scheduler exception。
- `arrive=false` 且轨迹为空、维度错误、包含 NaN/Inf：分类为 invalid_trajectory，不进入几何计算，不发布运动。具体继续等下一帧还是锁存故障，沿用现有错误分级；持续无有效结果的截止时间不被每次坏输出重置。
- 分别验证 opt_state、action、ref_state、distance 等每个消费者的真实维度需求；一个 plan_usable 布尔值不能代替所有下游契约。
- 运动所需字段必须有限且有效。仅诊断字段中的无穷距离、NumPy 标量等可按定义转成 null/普通标量，不能把无效运动数值转成 0 来假装有效。
- 几何、gear、序列化错误携带具体子阶段和字段。纯诊断消息格式错误不应冒充 ECOS 故障；若诊断语义被运行授权消费，必须保持现有失效即停车契约，不能悄悄跳过。
- 审计当前 Twist、planner envelope、semantic 的发布顺序：所有必需校验先完成，发布前重新检查故障、执行上下文和版本。若需要调整顺序，必须同步验证 Controller、Guard、source/grant 配对规则；不把多 topic 发布误称为原子事务。

先用捕获的失败输入复现，再修改触发路径。若只有推测，不提前更改几何策略、绕障策略或求解器。

## 6. 改动四：规划进展观察和独立停车

新增轻量 `planner_progress_monitor.py`，独立于规划线程观察“有有效任务和新输入，但调度或发布不前进”。读取不可变状态快照，不获取业务 state_lock、不等待 worker IPC。

分类示例：

- 新输入且 pending，但线程已退出：scheduler_thread_dead。
- 线程存在且阶段长时间不变：scheduler_stage_stall，记录阶段/持锁等待及堆栈。
- Worker 成功响应继续增加、主节点发布不增加：postprocess_or_publish_stall。
- 无新雷达/TF/odom：input_wait，具体源超时仍由现有契约处理。
- 无活跃目标、主动取消、合法到达、正在关闭：不报规划失速。

必须沿用现有启动和运行预算：先读取本次解析后的参数，不能直接使用基类默认值。代码中 bridge 默认 0.25 秒、基类 n115_timeout_sec 默认 30 秒，真实 launch 可以覆盖，不能任选一个数字当真机预算。

现有配置参考：source/envelope 0.50 秒、planner input 0.75 秒、health 0.50 秒、恢复窗口 5 秒、启动结果窗口 40 秒。它们是不同含义的预算；第一次结果等待与运行中输出断流不得合并。此次改进不增大这些值，也不通过新心跳、重复零结果、清空 UUID/boot_id 来刷新旧结果时效。

Python 观察线程是诊断增强，不是最终安全保证。Controller/Guard 的独立进程定时器与实际底盘指令时效继续兜底 GIL 卡死、主进程停顿和 DDS 丢包。发现明确线程故障时调用既有 fatal 链；仅观察到慢周期时先记录证据，是否升级故障须与既有预算一致，不能任意缩短运动截止时间。

## 7. 事故证据包与资源上限

新增 `tools/diagnostics/collect_navigation_incident.py`，自动汇总：

- 分支 commit、dirty diff 摘要、实际 UI 脚本、发布包 manifest/hash；源码、构建和运行版本分别记录。
- 各阶段环形记录、完整 traceback、线程栈、输入源年龄和各计数。
- 最近请求和输出的数值快照：位置、路径、轮廓、雷达点、速度约束、worker 输出及上下文；与现有 exact_capture 复用，避免重复采集。
- ECOS JSONL/NPZ、ROS 节点日志、解析后的参数和库来源。
- 明确原始错误、停车决策、后续诊断错误的时间线。

每进程数值环目标上限 64 MiB，单条输入上限 16 MiB；超过限制记录明确的 capture_skipped 原因。异步队列有上限，满队列丢诊断而不阻塞控制。事故包使用临时目录加原子完成标记。诊断目录目标总预算 512 MiB，到上限停止新增大输入并报警；不自动删除人工保全的历史事故包。以上是实现约束，需性能测试确认，尚非已达到指标。

保留目前 ECOS 的原始退出码记录。实时不增加 solver verbose、不追加求解或自动切求解器。要看 ECOS 每次迭代，用保存的锥规划在离线 replay 中启用 verbose。

## 8. 具体文件和提交划分

| 提交 | 文件/模块 | 完成条件 |
| --- | --- | --- |
| P1：异常和阶段诊断 | 新增 planner_diagnostics.py；修改 latest_input_scheduler.py、n1_16_node.py、n1_16_bridge.py | 异常原文/完整栈可保存；on_error 失败可见；阶段可追踪 |
| P2：结果契约修复 | 新增 planner_output_validation.py；修改 n1_16_node.py 的后处理与发布 | 捕获问题有离线复现，错误不进入不适用的几何/发布路径 |
| P3：进展观察与事故包 | 新增 planner_progress_monitor.py、collect_navigation_incident.py；必要配置/启动集成 | 输出断流能分类，故障证据自动保存，停车预算不放宽 |
| P4：验收和发布 | tests、文档、release manifest/UI7 版本登记 | 隔离集成测试及现场验收分别记录，未验证状态明确 |

WorkerHealth 和 C++ 授权协议默认保持原 schema；规划诊断使用独立 topic `/neupan_planner_diagnostics` 和本地文件。若最终必须扩展运行时协议，应另立迁移提交、更新混版拒绝与发布兼容性测试，不能在诊断提交中暗改。

## 9. 验证矩阵与通过标准

| 故障/场景 | 预期 |
| --- | --- |
| 正常新输入、正常求解 | 进展计数推进，正常发布，无误报 |
| ECOS -1/-2/-3 等失败 | 保留原始码、info、失败输入；原有故障停车语义不变 |
| Worker 正常，但后处理主动抛异常 | 捕获完整 traceback、阶段和请求；故障锁存、撤权、零输出 |
| scheduler on_error 自身报错 | 同时保存两层异常；线程死亡可见，不能静默失去监督 |
| 调度未唤醒、state_lock 长等待、发布调用阻塞 | 区分阶段，导出堆栈；独立截止机制仍能停车 |
| Python 主进程或 GIL 停顿 | 即使 Python 诊断不可用，Controller/Guard 仍独立撤权 |
| arrive + 空轨迹 | 正常完成、零输出，不触发错误几何调用 |
| 非到达空轨迹、NaN/Inf、错误形状、诊断序列化异常 | 明确分类；没有无效运动输出或旧结果续命 |
| 路径更新、gear 切换、旧 Worker/旧 boot/同 UUID 旧授权重放 | 原有版本与授权屏障保持有效 |
| 冷启动、无目标、取消、终态、短时丢扫描 | 使用对应预算，不出现新增误报或未经授权重启 |
| 磁盘满、只读目录、写线程阻塞、队列满 | 诊断失败可见；不能卡住规划或绕过停车 |

依次运行纯 Python 单元测试、实际 CvxpyLayer/ECOS 重放、隔离 ROS 集成、发布包检查，最后才是现场运动验收。隔离 ROS 用模拟硬件与独立 Domain，禁止引入真实双臂驱动；只改输出 topic 不足以证明不会连接硬件。

实时性目标：同一份输入回放对比，诊断开启后正常周期 p95 总耗时增加不超过 5%，热路径阶段记录目标增加不超过 1 ms；同时记录 p99、丢日志数和 CPU/磁盘负载。目标若未达到，先降采样/异步化，不能放宽运动源截止时间。必须测试故障捕获期间与磁盘异常期间的撤权延迟。

功能通过条件：一次重现可生成完整因果时间线；能明确区分 solver_error、input_wait、scheduler_exception、stage_stall 和发布阶段故障。安全通过条件：没有有效新鲜结果和当前授权时非零命令为零，故障不因对象重建/部分重启自动清除；停止时间不劣于现有配置预算及调度容差。

现场验收建议固定地图和起终点，重复本轮失败路线至少 10 次，并覆盖到达、取消、短时输入中断；记录成功率、控制输出最大间隔、故障恢复次数和 CPU 负载。十次成功是回归门槛，不是对任意场景的保证。涉及主动故障注入的运动测试需先在无运动环境通过，不能拿真机带目标直接验证。

## 10. 上线、回退和非目标

- 保留现有未提交的 solver_recovery、planner 配置等修改，单独记录。它们不是本方案默认依赖，不混进 P1/P2 修复提交。
- 所有代码在目标分支形成小提交；独立 build/install，新建发布候选包，校验每个实际加载包和 Python 模块路径。
- 更新 UI7 之前保存原按钮配置和 unified 发布身份；不修改旧 bundle 内容。候选包验证完成后注册新 UI7 入口，当前进程不热更新，不在有活跃目标时自动重启。
- UI8 继续保留现有 1016 候选环境作为对照，本方案不把新分支的修复偷偷移入它。
- 回退使用已保留的旧 UI7 发布包和按钮配置，先停止活跃导航并确认静止，再进行版本切换；不可通过回退清除故障后自动恢复旧目标运动。
- 本方案不默认增加自动重启、换求解器、延长超时、关闭来源校验或改变 collision check。最终额外碰撞检查继续关闭，动态轮廓、输入处理和授权/停车约束按现有版本保留。

先完成 P1 和首轮隔离重现，再依据栈和数值证据落实 P2；如果证据指向锁或发布堵塞，就修对应路径，而不是继续围绕 ECOS 猜测。没有完整重现与验收记录，不标记“已恢复 1016 导航效果”或“卡住问题已解决”。
