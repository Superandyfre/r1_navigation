# R1 导航改动与 CogNav 接入交接（2026-10-29）

本文供 CogNav 负责人和导航负责人共同使用。覆盖本轮 Worker fail-stop、NeuPAN/ROS2 连接、Nav2 生命周期、UI7 入口与环境版本管理。实际运动验收尚未完成；“源码已提交”“程序已构建”“UI7 已选中”和“实机导航通过”必须分别记录。

## 1. 当前结论和协作边界

- 统一交付分支为 `codex/navigation-environment-snapshots`。本轮将 `codex/neupan-failstop-a-plus` 合入该分支，保留原有环境快照工具及已提交的 ECOS 诊断工具。
- UI7 应使用从统一分支的固定提交构建的独立发布包。发布清单记录源码 SHA；不能仅凭当前 checkout 分支名判断运行版本。具体新发布路径及 SHA 见同目录 `UI7_UNIFIED_VERSION_20261029.json`。
- 合并前 UI7 选中 `20261029-a-plus-09`，构建源码 `0e4b48e56909bfad23cf606b631278f498ef69c5`，工具源码 `2e96accbe524b2908ba6100c28c6ff49fe1d0156`。原主分支 `3385462` 仅有环境管理增量，未合入 A+，此次统一解决这一分叉。
- **生产 CogNav 不由本轮修改或部署。** `/home/rpp/cognav_ws/objnav_live_round3/robot_live` 由 CogNav 同学维护。仓库 `integrations/cognav/` 是历史隔离候选和参考，不是生产替换包；发布工具明确排除 CogNav。
- 当前生产 CogNav 尚不能按“已经兼容新链”处理：现有 bridge 使用 `/follow_path`，但只发 Action 不足以满足新增的执行归属续租协议。接入工作由 CogNav 负责人实施，双方联调后确认。
- 当前仍有启动通信/健康间歇问题和现场定位问题待关闭。不能据此宣布此前“卡住、给新目标仍不动”的现场故障已彻底解决。

**本轮切换核验补充：** 22:47 另一路启动了 `/home/rpp/nav_restore_20261016` 下的旧版导航（观测 PID 532094/532112/532114），且 UI7 配置在本任务期间被其他操作改回过主工作树脚本。本次不停止或热替换这些进程；已将 UI7 下次启动入口设为统一版本。当前仍运行的旧进程不代表新版本已经加载，须停止旧导航后再由现场启动新入口。并行任务请勿再次覆盖 UI7 注册。

## 2. 本轮导航端实现了什么

| 层次 | 已实现的主要变化 | 对上游的影响 |
| --- | --- | --- |
| Worker/IPC | 保留 spawn 子求解进程隔离；首次致命故障锁存；独立健康发布；退出、协议异常、超时传播；停止使用健康心跳冒充求解结果新鲜度 | Worker 仍活着不代表可以运动；FATAL 后重发目标不能恢复 |
| 输入/结果绑定 | 原子 execution context 绑定 boot、真实 Action UUID、path/goal version；Source v2 保存扫描、派发和求解等时间；拒绝过期、重复和错误身份 | 无身份的 Path/Goal、旧结果、旧 ledger 不能重建运动权限 |
| ControllerServer | 将实际 FollowPath goal handle 作为执行身份；接入 begin/end、取消、抢占、故障和 costmap 等待中断；执行线程失活也停止授权 | Action accepted 与最终 SUCCEEDED/CANCELED/ABORTED 是不同阶段；超时不能假定请求没被接受 |
| NeuPANController/runtime | 显式 run arm、owner 确认/续租、Grant/ACK/COMMIT、路由零速屏障、独立时限、周期只读状态；FATAL 不可由新目标清除 | 上游必须证明当前任务仍有效，不能只维持无条件心跳 |
| 最终 Guard | 保留独立授权、健康/Source/命令/里程计/定位检查、速度限值和故障停车；HW1 去掉额外障碍 collision geometry 检查 | 节点名仍是 `r1_collision_guard`，不能因为名字含 collision 就删除整个节点 |
| Nav2 BT | FollowPath owner 随实际 RUNNING tick 续租；halt/取消/终态停租；普通受限重试与不可重试故障区分 | FATAL 不进入无限重试或不断重发同目标；直接用 CogNav FollowPath 时也必须履行相同生命周期义务 |
| 启动与版本 | UI7 单实例、父进程退出清理、包来源检查；主分支显式加载固定 ROS/underlay；源码/环境采集、恢复和哈希清单 | 不允许从错误 overlay 加载旧 Controller、Guard 或 Python 后声称使用新分支 |
| 诊断 | 保存首故障、协议身份和测试证据；已提交 ECOS hook 记录原始状态和可重放失败输入 | 诊断结果不改变求解结果、授权或重试策略；诊断日志需要按运行目录管理 |

此外修复了首次 TF 未就绪后 Worker 调度未重新唤醒的具体问题。此修复不等于已确定所有现场停滞和 DDS 超时的共同根因。自动 Worker 替换/自动故障恢复（B 方案）未开放。

## 3. 运动链和职责

```text
CogNav driver（认知决策、参考路径、父任务租约）
  -> CogNav ROS bridge（接受句柄、取消、状态观察、owner 续租）
  -> /follow_path（Nav2 ControllerServer）
  -> NeuPANController <-> Worker/spawn NeuPAN solver
  -> 原始速度 -> velocity_smoother（所选 profile 启用时）
  -> 最终 Guard（A+ 授权与故障/超时停车）
  -> /cmd_vel -> 底盘驱动
```

常见平滑模式的内部速度话题为 `/neupan_nav2_raw_cmd_vel`、`/neupan_nav2_smoothed_cmd_vel`；direct profile 路径不同。必须以实际 launch/profile/remap 为准。CogNav 不发布上述速度，也不发布 `/cmd_vel`。RViz/Nav2 BT 与 CogNav 不应同时争用导航任务；双方需约定独占任务来源和切换时的取消/清理流程。

NeuPAN 的扫描、双雷达融合、mesh 自过滤、动态机器人几何和避障仍保留。移除的是 HW1 最后一级重复障碍检查，未移除 NeuPAN 自身避障，也未证明任何场景绝对无碰撞风险。`collision_check_enabled` 在 HW1 固定为 false，禁止拿旧配置重新打开。

## 4. CogNav 对外接口

以下是当前 A+ 配置/协议，联调前还要核对有效参数。所有 JSON 话题均需严格校验 schema、字段类型、有限数值、boot/UUID、序列及时间；缺失或旧 schema 不能当作健康。

| 接口 | 类型 | 发布/调用方与用途 |
| --- | --- | --- |
| `/follow_path` | `nav2_msgs/action/FollowPath` | CogNav bridge 发路径；controller_id=`FollowPath`，goal_checker_id=`general_goal_checker` |
| `/neupan_controller_status` | `std_msgs/msg/String`，`r1_neupan_execution_status_v1` | Controller 发，只读；约 50 ms 周期，Action 结束后仍发布 |
| `/neupan_worker_health` | `std_msgs/msg/String`，`r1_neupan_worker_health_v1` | Worker 发；观察 boot/generation、状态及首故障，不代表结果新鲜 |
| `/r1_collision_guard/status` | `std_msgs/msg/String`，`r1_collision_guard_status_v1` | Guard 发；观察授权/停止/故障原因，不能只看节点在线 |
| `/r1_localization_health` | `std_msgs/msg/String`，`r1_localization_health_v1` | 定位健康，须与实际 TF/odom 一起确认 |
| `/neupan_execution_owner` | `std_msgs/msg/String`，`r1_neupan_execution_owner_v1` | CogNav bridge 仅为自己已接受、仍执行的 Action 发确认/续租 |
| `/neupan_controller/can_retry` | `std_srvs/srv/Trigger` | 查询当下是否允许重试，不是 arm，不保证之后永远可重试 |
| `/neupan_controller/revoke_uncertain_execution` | `std_srvs/srv/Trigger` | 执行状态不确定时的只撤销通道；run-wide 锁存，可影响当前其他目标，不能当普通 cancel 用 |
| `/neupan_controller/arm` | `std_srvs/srv/Trigger` | 操作员/导航端显式启用 run；需要 idle、停止证据、有效状态且无 FATAL；CogNav 不自动调用 |

内部 `/neupan_execution_context`（context v1）、`/neupan_planner_result`（Source v2）、`/neupan_motion_grant`（grant v1）、`/neupan_path` 和 `/neupan_goal` 由导航端管理，CogNav 可观察但不得发布、伪造、重放或接管。后两者不是 CogNav 绕开 Action 的入口。

Controller 周期状态的关键字段：`authority_boot_id`、`controller_boot_id`、`sequence`、`monotonic`、`plugin_active`、`execution_active`、`fault_latched`、`fault_reason`、`navigation_attempt_id`。同一话题可能含其他诊断消息，必须按 schema 筛选。首次 fault_reason 应保留给上层显示，不能被后续泛化超时覆盖。

### 4.1 owner 协议是接入必需项

获得 **accepted goal handle 后**，从实际 `goal_id.uuid` 的 16 字节编码出 32 位小写十六进制字符串。不能用 CogNav plan_seq、自造 UUID 或无补零的展示字符串代替。再等待 Controller 新鲜状态/context 与该 UUID 匹配，取得两个当前 boot，发布：

```json
{
  "schema": "r1_neupan_execution_owner_v1",
  "authority_boot_id": "实际当前 ControllerServer boot",
  "controller_boot_id": "实际当前插件 boot",
  "navigation_attempt_id": "00112233445566778899aabbccddeeff00",
  "owner_id": "本次 bridge 实例的唯一 ID",
  "sequence": 1,
  "monotonic": 123456.789
}
```

示例值不能照抄到运行系统。当前 owner 实现：begin 后 1 秒内需首个确认，之后来源时间年龄不超过 0.5 秒；参考候选每 0.1 秒尝试续租。序列为正整数并严格递增，时间也要递增；重发相同包不会延长租约，同序列冲突或 owner 身份变化会导致故障。

这使用**同一台机器人操作系统的 monotonic/steady clock**，不是 ROS time、墙上时间或远端笔记本时钟。若 CogNav 算法跑在其他主机，应由机器人本机 bridge 在核实父任务新鲜度后生成 owner；不能直接复制远端 monotonic。当前协议不提供跨主机时钟同步或网络身份认证。

每次续租必须同时满足：句柄已接受且任务仍 executing、未取消/退出、Worker/Controller/Guard 状态当前且匹配、父 driver 租约有效、定位及所需机器人输入新鲜。**不能把无条件 owner 定时器放在独立线程里一直续租。** driver 卡死、输入停止、bridge 退出、取消或终态时必须停租；只剩健康发布线程存活不能代表上层任务仍有效。

### 4.2 Goal、取消和故障处理顺序

1. 导航端完成启动、来源和定位核对，现场人员按约定显式 arm；CogNav 只读确认准入条件。
2. CogNav 验证路径 frame、TF、有限坐标、有效四元数、时间和父任务身份；发送 FollowPath。当前导航 frame 配置为 `map`，实际以有效参数为准。
3. 分开管理发送中、accepted、executing、canceling、terminal。发送响应超时不是拒绝证据；保留旧请求身份，不直接再发第二份目标。
4. accepted 后绑定真实 UUID，完成首个 owner 并在有条件的执行循环中续租。新 Action 必须重新绑定；Controller 内部同 UUID 路径版本变化不允许上游自造身份。
5. 正常取消先停租，再请求精确 goal handle 的 cancel 并等待终态；若返回/执行归属不确定，使用双方约定的撤销处置，而非假装旧目标已经消失。撤销是整轮锁存，不是温和暂停。
6. 收到 FATAL、boot/generation 意外变化、状态过期或执行不确定时：停租、阻止新计划、保留首故障和旧 UUID，取消未解决目标；不能靠重建 bridge、改 session 或重复目标解除导航端锁存。
7. 健康的普通终态与 FATAL 区分处理。普通重试受 `can_retry` 和上层有限策略约束；FATAL 按导航端受控整套恢复流程处理，再显式 arm 和创建新 Action。禁止自动清 ledger、只重启 Worker 或改 fault 字段“恢复”。

参考协议源码：`packages/navigation_neupan_bridge/EXECUTION_OWNERSHIP.md`、`execution_owner_lease.hpp` 和 `packages/r1_neupan_runtime/`。历史候选 `integrations/cognav/robot_live/cognav_bridge.py` / `neupan_failstop.py` 可供 CogNav 同学比较实现，但必须先比较生产 baseline 漂移，不能整目录覆盖生产代码。完整 CogNav driver 联调未验证。

## 5. 哪些内容 CogNav 同学可以改，哪些不能自行改

| 范围 | 负责人及边界 |
| --- | --- |
| CogNav 决策、父 driver lease、spool、ROS bridge 的 Action/owner/状态适配 | CogNav 负责人维护；接入本协议并提交自己的变更，不由导航端覆盖 |
| Worker、IPC、Source/health/context、runtime 协议与锁存 | 导航端负责；CogNav 不改字段语义、刷新旧时间或屏蔽 FATAL |
| Nav2 ControllerServer、NeuPANController、BT owner/取消逻辑 | 导航端负责；禁止换回旧 server/plugin 来绕过 owner，禁止将无限重试当恢复 |
| Guard、速度链/remap、唯一最终发布者、授权握手 | 导航端负责；禁止删 Guard、旁路到 `/cmd_vel`、伪造 Grant/ACK/COMMIT 或健康 |
| UI7 配置、启动脚本、ROS overlay、固定 release、解释器和 solver 环境 | 导航端统一发布；CogNav 不在原地改 release/install、不改 `.rpprc` 或共享 venv 去试兼容 |
| 地图、雷达外参、机器人模型、mesh、自过滤、底盘/定位参数、限速和时限 | 需双方提出问题后由对应负责人联合确认；不得为“先动起来”临时关闭定位/新鲜度检查或放宽超时 |
| 现场启动、arm、急停释放、底盘运动和验收 | 现场操作员与导航负责人协调；这份文档不授权自动执行 |

“把门禁撤掉”仅指用户要求移除 UI7 的**发布批准启动检查**，已完成；不代表移除运行时 arm、owner、Guard、故障/超时停车。快照/发布清单里的 false 验收状态不得改成 true 以消除提示。旧配置里部分 collision 注释属于历史语境，当前 HW1 行为以 collision=false 代码路径为准。

## 6. 验证结果和仍未完成的事项

已完成过 Worker/协议单测、隔离 ROS Action 链和真实 spawn 求解器的定向验证，覆盖 FATAL、失源、取消/抢占、owner 失联、重放、握手丢失、部分/完整重启、BT tick 停止等局部场景。最后一次去额外碰撞层的整套安装链为 3/3；Guard 授权定向回归 15/15。它们使用合成输入、隔离 ROS 域和无底盘驱动配置，不能替代现场验收。

历史无额外观测工具的完整 43 场景为 40/43，3 项 Navigator configure 响应超时仍保留 OPEN；另有启动 `worker_health_stale` 等间歇故障。后续定向通过没有把这些历史失败关闭。新统一版本的构建/静态验证记录另见版本记录，不能继承旧版本的“全量通过”结论。

曾在急停条件下做过无目标、未 arm、最终输出隔离的启动观察：491 个输出样本全零，观察时真实 `/cmd_vel` 无发布者。但仍待 `map -> base` 定位链就绪，存在 costmap inflation 提示、停止阶段 planner 异常及不完整的图发现；这些都没有被称为已解决。

尚待双方协调：

1. CogNav 生产 bridge 的 owner/状态/父租约适配及完整 driver 联调。
2. 启动 DDS 响应/Worker 健康间歇问题定位与关闭；不能通过放宽时限掩盖。
3. 全启动有效参数、包/库/Python 来源和唯一速度发布者核对；UI7 入口切换不证明运行图正确。
4. 现场定位、正常导航、障碍处理、取消/抢占、故障后再发目标、停车时间/距离和驱动 watchdog 验收。
5. 完整隔离恢复/回滚演练；保留的旧包是恢复材料，不是已验收可靠版本。

先在无底盘输出的隔离环境验证：正常 accepted→owner→完成；driver/输入/bridge 停止时停租；Worker FATAL 和旧目标重发不恢复；取消/抢占准确终结；响应超时不重复发目标；新旧 boot/UUID 不混用。场景分别保存 UUID、boot、首故障、Action 终态和最终速度证据。进入真实机器人测试由现场另行安排。

## 7. 仓库、快照、日志怎么取

- GitHub：`https://github.com/Superandyfre/r1_navigation.git`，统一分支 `codex/navigation-environment-snapshots`。拉取后以交接记录中的具体 SHA 核对；分支可能继续前进。
- 局域网仓库：`ssh://pinggu@192.168.31.36/home/pinggu/r1_navigation.git`。这只是 Git 仓库，不是机器人导航运行目录。
- 机器人主源码：`/home/rpp/r1_navigation`；旧 A+ 工作树：`/home/rpp/neupan_failstop_ws/src/r1_navigation`，作为历史来源，不再当作另一套独立主版本。
- 已有基线 tag `nav-baseline-20261029-01` 指向合并前 `3385462`，保持不移动；不能用它声称已包含本次统一改动。
- 大文件基线：`/home/rpp/navigation_snapshots/20261029-baseline-01`（约 1.7 GB）。Git 只保存索引/哈希；大文件未上传 GitHub 或 192.168.31.36。恢复说明见 `docs/restore.md`。
- 基线状态为 captured=true、built/restored/navigation_validated/offline_complete=false。采集有 16 个节点参数查询失败/超时、513 个原目标缺失的符号链接；无正常导航 rosbag、无完整离线系统安装闭包。此次代码合并不追溯修改该基线。
- 历史证据目录：`/home/rpp/neupan_failstop_ws/reports/neupan_failstop_20261027`。旧大测试场景已经归档到 `final-cleanup-20261029/historical-test-artifacts.tar.gz`，不要将已清理路径不存在误判为当时测试不存在，也不要声称已重新运行旧包。
- ECOS 诊断目录由启动时 `ECOS diagnostics:` 输出标明，可用 `RPP_ECOS_DIAGNOSTICS_DIR` 指定。每 PID 轮转有上限，但历次运行目录总量不自动限额；先保留故障输入和版本身份，再由导航负责人清理。
- 主工作树中另有未提交的 planner/solver recovery 实验修改。本次统一构建仅采用已提交源码，不擅自提交或部署这些修改；`git diff` 非空不能直接当作已交付内容。

## 8. 联调前双方交换的信息

CogNav 同学提供：生产 bridge/driver commit 与工作区差异、实际启动命令/解释器、运行主机、父租约和 spool schema/时限、路径 frame/TF 来源、取消/超时/重试策略、当前使用的 action 和话题 remap。

导航端提供：统一源码 SHA、UI7 固定发布目录和 manifest/hash、实际 ROS 域/profile/参数、节点/Action/topic 图、包/动态库/Python 来源、首故障与诊断示例、未关闭问题列表和现场安排。

双方共同确认：同一时刻谁拥有导航任务、accepted UUID 如何绑定、何时停租/取消/撤销、FATAL 如何展示及恢复、哪些测试只隔离运行、何时允许现场验收。CogNav 侧的接入提交与导航版本需成对记录，不能只说“用了最新分支”。
