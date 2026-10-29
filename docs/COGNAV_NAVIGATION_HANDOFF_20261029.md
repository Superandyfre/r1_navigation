# R1 导航系统现状与 CogNav 接入技术说明

日期：2026-10-29

适用对象：CogNav 决策与 ROS 2 接口维护人员、导航系统维护人员

适用版本：`codex/navigation-environment-snapshots` 分支；UI7 发布包 `20261029-unified-01`

## 1. 系统状态与适用范围

当前导航端已实现 NeuPAN 求解进程故障隔离、执行身份绑定、上游任务有效性确认、控制授权以及故障/超时停车机制，并形成可构建的独立发布包。CogNav 生产程序尚未完成与这些机制的适配，完整 CogNav—导航—底盘链路尚未联调通过。

**新版尚未完成实机运动验证，实际导航效果未知。** 目前无法确认真实环境中的路径跟随、避障、持续运行稳定性、故障后停车时间与距离，也不能确认此前“导航停滞且重新发送目标无效”的现场问题已经解决。代码实现、构建成功及隔离环境测试结果，不构成上述实机效果的证明。

| 项目 | 当前状态 | 对接含义 |
| --- | --- | --- |
| 导航端代码与发布包 | 已实现并构建；发布包使用固定源码版本 | CogNav 应按本文协议适配，不应混用旧控制组件 |
| UI7 | 已配置为新版发布包入口；本次未通过 UI7 启动运动验证 | 入口配置完成不代表新版已通过现场验收 |
| UI8 | 旧版导航独立入口 | 不属于本次新版联调对象；联调时需避免两套导航争用底盘 |
| CogNav 生产适配 | 尚未完成 | 现有 `/follow_path` 调用不能直接视为兼容新版 |
| 隔离验证 | 有组件、协议和局部 ROS 2 链路验证证据 | 仅支持对应机制在所测条件下的行为结论 |
| 实机验证 | 未完成 | 导航质量、可靠性和物理停车效果均待验证 |

导航系统仍存在启动通信响应超时、Worker 健康状态间歇超时等未关闭问题；现场定位链、完整启动配置及停止阶段异常也需复核。CogNav 接入前应将这些问题作为导航端已知限制，而非通过上层重发目标或放宽时限规避。

## 2. 系统结构及功能边界

```text
CogNav 决策程序
  │ 参考路径、父任务有效性
  ▼
CogNav ROS bridge
  │ FollowPath Action、执行归属确认、取消及状态观察
  ▼
Nav2 ControllerServer / NeuPANController
  │ 带执行身份的路径上下文
  ▼
Worker 管理进程 ── IPC ── NeuPAN 求解子进程
  │ 当前任务对应的求解结果
  ▼
NeuPANController → 速度平滑器（启用时）→ 最终 Guard → /cmd_vel → 底盘驱动
```

CogNav 负责决策、参考路径及该任务是否仍应执行；导航端负责局部求解、速度生成、授权检查和故障处理。CogNav 通过 Action 提交路径，不直接驱动底盘，也不接管导航内部的结果与授权话题。

| 组件 | 当前职责及主要行为 |
| --- | --- |
| Worker 管理进程 | 通过独立子进程运行求解器；发布健康状态；求解失败、进程异常及协议错误按故障语义传播，首次致命故障保留 |
| NeuPAN 求解器 | 根据路径、扫描和机器人几何生成局部轨迹与速度；现有扫描、自过滤及避障功能保留 |
| ControllerServer | 以实际 FollowPath 目标句柄作为执行身份，管理开始、取消、抢占、完成和失败 |
| NeuPANController | 检查求解结果身份与时间、上游任务有效性及 Guard 握手；周期发布执行状态 |
| 最终 Guard | 独立检查授权、Worker/结果状态、命令时效、里程计、定位及速度限值，并在条件不满足时输出零速度 |
| Nav2 BT | 通过 Nav2 导航入口调用时管理 FollowPath、执行归属续租及受限重试；CogNav 直接调用 FollowPath 时须自行实现对应义务 |

HW1 已移除最终 Guard 中额外的障碍几何检查，`collision_check_enabled=false`。**Guard 节点本身仍是速度链的必要组成部分**，其授权及故障/超时处理保留。节点名称仍为 `r1_collision_guard`。NeuPAN 自身避障、双雷达输入和机器人 mesh 自过滤不因此移除；真实避障效果仍须实机验证。

## 3. 与 CogNav 有关的行为变化

### 3.1 Action 接受与运动许可分离

FollowPath 返回 accepted，仅表示导航端接受了该目标请求。允许非零速度还需要满足显式启用、上游执行归属确认、当前求解结果有效、定位/里程计有效以及 Guard 授权等条件。

因此，“Action 已接受但机器人不动”不能直接判断为求解器卡死。需联合检查执行状态、首故障、owner 确认、求解结果及 Guard 停止原因。

### 3.2 使用真实执行身份关联路径、结果与授权

导航端将实际 Action UUID、控制组件启动身份以及路径/目标版本关联。旧目标的结果、旧进程的消息或重复消息不能作为当前目标的运动依据。

Worker 健康心跳和求解结果新鲜度独立判断：管理进程仍存活，不代表求解子进程仍正常产出，也不代表旧结果可以继续使用。结果重复发布不能延长其有效时间。

### 3.3 致命故障采用锁存语义

FATAL 表示当前运行周期进入不可自动恢复的故障状态。新目标、重复路径、bridge 重建或健康消息恢复，均不能清除该状态。自动 Worker 替换及自动故障恢复未启用。

普通取消、正常完成和允许重试的失败，应与 FATAL 区分。发生 FATAL 后，CogNav 应停止提交新计划，保留原始原因，由导航负责人按受控恢复流程处理；不能将反复发送目标作为恢复策略。

### 3.4 上游任务必须持续有效

CogNav bridge 需要向导航端确认：当前 Action 确实属于自己，父决策任务仍有效且仍要求执行。该确认称为 **owner 续租**。

仅保持 bridge 进程或定时器存活不足以续租。父决策程序失联、输入过期、取消或任务结束后必须停止续租，避免上游已停止而底层仍执行旧路径。

## 4. ROS 2 接口契约

以下为当前发布配置中的名称。若联调使用 namespace 或 remap，须由双方确认完整映射，不能仅依据源码默认值。

### 4.1 路径执行接口

| 项目 | 定义 |
| --- | --- |
| Action | `/follow_path` |
| 类型 | `nav2_msgs/action/FollowPath` |
| `controller_id` | `FollowPath` |
| `goal_checker_id` | `general_goal_checker` |
| 导航参考坐标系 | 当前配置为 `map`；以实际有效参数为准 |
| 客户端责任 | 校验路径坐标、有限数值、有效四元数和所需 TF；维护请求、句柄、取消及终态 |

CogNav 应区分发送中、已接受、执行中、取消中和终态。发送响应超时不证明目标被拒绝，取消请求已发送也不证明取消完成。旧请求状态未解决前，不应直接创建重复执行。

### 4.2 状态观察接口

下列话题均为 `std_msgs/msg/String`，内容为 JSON。消费者须按 `schema` 选择消息，并校验字段、时间、序列和身份；消息缺失、过期或无法解析不能视为正常。

| 话题 | schema | 用途 |
| --- | --- | --- |
| `/neupan_controller_status` | `r1_neupan_execution_status_v1` | 判断插件/执行状态、执行身份及故障锁存；约 50 ms 周期，Action 结束后继续发布 |
| `/neupan_worker_health` | `r1_neupan_worker_health_v1` | 判断 Worker 启动身份、代次、状态及首故障；不能替代结果有效性判断 |
| `/r1_collision_guard/status` | `r1_collision_guard_status_v1` | 判断最终授权与停止/故障原因 |
| `/r1_localization_health` | `r1_localization_health_v1` | 判断定位健康，配合 TF、里程计及输入时效检查 |

Controller 状态关键字段为：

| 字段 | 含义 |
| --- | --- |
| `authority_boot_id` | 当前 ControllerServer 执行权威的启动身份 |
| `controller_boot_id` | 当前控制插件的启动身份 |
| `navigation_attempt_id` | 当前实际 FollowPath UUID |
| `sequence`、`monotonic` | 消息序列与同机单调时钟时间 |
| `plugin_active`、`execution_active` | 插件是否激活、是否存在活动执行 |
| `fault_latched`、`fault_reason` | 是否锁存故障及其原因 |

状态话题只提供观察信息，不授予运动权限。同一话题可能含其他 schema 的诊断消息；不能将任意消息到达都当作目标状态刷新。组件启动身份或 Worker 代次意外变化时，应中止旧执行关联，重新确认系统状态。

### 4.3 执行归属确认：owner 协议

发布话题：`/neupan_execution_owner`，类型：`std_msgs/msg/String`。

必须先获得 accepted goal handle，再从 `goal_id.uuid` 的 16 字节得到**32 位小写十六进制字符串**，作为 `navigation_attempt_id`。不能使用 CogNav 计划编号或自行生成的替代 UUID。编码示意：`bytes(goal_handle.goal_id.uuid).hex()`。

bridge 从新鲜 Controller 状态取得与该 UUID 对应的两个 boot ID 后，发送：

```json
{
  "schema": "r1_neupan_execution_owner_v1",
  "authority_boot_id": "当前 ControllerServer 的启动身份",
  "controller_boot_id": "当前控制插件的启动身份",
  "navigation_attempt_id": "00112233445566778899aabbccddeeff00",
  "owner_id": "当前 bridge 实例的唯一标识",
  "sequence": 1,
  "monotonic": 123456.789
}
```

以上为结构示例，运行值必须来自实际目标和当前进程。

| 约束 | 当前实现 |
| --- | --- |
| 首次确认期限 | 执行 begin 后 1 秒内；不是客户端收到响应后重新计时 |
| 持续确认时效 | 确认后来源时间年龄不得超过 0.5 秒 |
| 参考发送周期 | 0.1 秒；每次发送前均重新检查准入条件 |
| 序列 | 正整数，严格递增；相同消息重放不能续租 |
| 时间 | 有限数值且递增；不能用接收时间替换原时间以延长有效期 |
| 身份 | 必须匹配当前 boot 和实际 Action；同一执行中 owner 身份不可随意更换 |
| 协议错误 | 同序列内容冲突、owner 身份冲突等可能触发故障；晚到确认不能清除已锁存故障 |

此协议依赖**同一操作系统的 monotonic/steady clock**。不能使用 ROS time、UTC 时间或另一台主机的 monotonic 数值。若 CogNav 决策在远端运行，应由机器人本机 bridge 在核实远端父任务有效性后生成 owner。

每次续租至少检查：目标已接受且仍执行、没有取消/退出请求、组件状态当前且身份匹配、父决策程序租约有效、定位及所需观测输入有效。不能通过无条件独立心跳维持续租，否则父程序卡死时可能错误保留执行权限。

### 4.4 管理服务

| 服务 | 类型 | 语义与调用边界 |
| --- | --- | --- |
| `/neupan_controller/arm` | `std_srvs/srv/Trigger` | 显式启用当前运行周期；要求空闲、停止证据及有效状态且无致命故障。由现场/导航端管理，CogNav 不自动调用 |
| `/neupan_controller/can_retry` | `std_srvs/srv/Trigger` | 只查询当前重试许可；不是启用命令，也不是永久许可 |
| `/neupan_controller/revoke_uncertain_execution` | `std_srvs/srv/Trigger` | 执行归属不确定时撤销当前运行周期。具有 run-wide 锁存效应，可能终止更新的目标；不能替代日常 cancel |

正常取消应针对精确 goal handle 请求取消，并跟踪最终结果。取消开始后停止 owner 续租；若取消与续租超时发生竞争，系统可能进入故障终态，不能预设一定返回 CANCELED。执行状态无法确认时，按双方约定采取撤销和停止接入流程，不能假设旧目标已消失。

## 5. CogNav 侧需完成的适配

当前生产入口位于 `/home/rpp/cognav_ws/objnav_live_round3/robot_live`，由 CogNav 负责人维护。需要完成以下功能后，才能将其视为新版导航的兼容客户端：

1. **状态准入**：订阅并解析 Worker、Controller、Guard 与定位状态；缺失、过期、故障或身份不一致时禁止提交运动计划，保留首次有效故障原因。
2. **Action 身份关联**：取得 accepted handle 后绑定实际 UUID，与 Controller 当前状态匹配；新目标重新建立关联，取消或结束后及时退役旧关联。
3. **有条件 owner 续租**：实现第 4.3 节协议，并将续租条件绑定到父决策任务及机器人输入。父程序、观测或任务失效时停止续租。
4. **取消与不确定状态处理**：覆盖未收到发送响应、取消等待、bridge 退出等情况；不得在旧执行未解决时盲目提交下一份路径。
5. **故障与重试区分**：FATAL 后停止新计划；普通失败采用有限重试并查询当前许可。重启 CogNav 本身不能清除底层故障。
6. **诊断输出**：向上层记录计划编号、实际 Action UUID、组件启动身份、Action 状态、首故障和最后状态时间，便于区分决策未提交、授权未成立和底层执行失败。

仓库 `integrations/cognav/` 提供隔离参考实现，包含状态观察、owner 续租与取消处理，但**不是可直接覆盖生产目录的发布包**。应由 CogNav 负责人审查与现有程序的差异后选择性适配；完整生产 driver 的兼容性尚未验证。

## 6. 修改权限与协作边界

| 范围 | 维护责任与限制 |
| --- | --- |
| CogNav 决策、父任务租约、spool、ROS bridge | CogNav 侧维护，可按接口契约修改；接口语义变更须双方确认 |
| Worker/IPC、求解结果身份与时间、故障锁存 | 导航侧维护；CogNav 不屏蔽故障、不修改结果时间、不清除运行状态以恢复运动 |
| ControllerServer、NeuPANController、Nav2 BT、owner/授权协议 | 导航侧维护；不得换回旧组件绕过新版检查，不得用无限重试掩盖失败 |
| Guard 与速度链 | 导航侧维护；不得删除 Guard、绕过平滑/授权链或新增 `/cmd_vel` 发布者 |
| UI7、发布目录、ROS 环境加载、Python/求解器依赖 | 导航侧统一维护；不得原地修改 release/install、全局环境或共享虚拟环境以试配 |
| 地图、定位、TF、标定、模型、mesh 自过滤、限速和超时 | 由对应负责人确认后调整；不得为使机器人运动而临时关闭检查或放宽时限 |
| arm、故障恢复、底盘运动及实机验收 | 现场人员与导航负责人协调；CogNav 不自动解除故障或执行整套重启 |

CogNav 不应发布导航内部的 `/neupan_execution_context`、`/neupan_planner_result`、`/neupan_motion_grant`、`/neupan_path` 或 `/neupan_goal`。这些接口可以只读观测，不能作为绕开 Action 的控制入口。

当前平滑模式内部速度话题为 `/neupan_nav2_raw_cmd_vel`、`/neupan_nav2_smoothed_cmd_vel`，最终输出为 `/cmd_vel`；其他 profile 以实际映射为准。CogNav 不发布这些速度。RViz/Nav2 BT、CogNav 和 UI8 旧导航之间必须明确任务独占关系，避免同时控制同一底盘。

UI7 当前不以发布验收批准作为启动前置条件，但运行时 arm、owner、授权握手和故障停车仍有效。**允许启动不代表验收通过，也不代表允许旁路运行时检查。**

## 7. 联调与实机验证要求

已有验证支持部分协议和故障传播行为，但尚未建立完整现场结论。需区分以下证据：

- Action 进入终态：证明软件任务结束。
- 最终速度输出为零：证明所观测软件输出归零。
- 实际底盘停止：须通过真实里程计、驱动和现场测量确认；前两项不能代替该结论。

建议先进行不接底盘输出的双方联调，至少确认正常提交与完成、父程序失联、输入过期、取消、目标抢占、Worker FATAL、旧目标重发、响应超时及 bridge 重启。每个场景独立记录执行身份、首故障、Action 结果和速度输出，避免仅凭“节点正常”或“有心跳”判断通过。

完成软件联调并处理导航端已知问题后，再由现场安排实机测试：

| 待验证项 | 需要确认的实际结果 |
| --- | --- |
| 启动和定位 | 正确加载新版组件，定位与 TF 稳定，无冲突的速度发布者 |
| 正常导航 | 可接受路径并稳定执行，目标到达、跟踪误差及持续运行表现满足现场要求 |
| 避障 | 真实障碍、机器人几何和自过滤条件下的行为符合预期 |
| 取消/抢占 | 旧任务确实停止，新任务不会继承错误权限 |
| 故障处理 | 求解失效、失源、进程失联等情况下的真实停车时间与距离满足要求 |
| 恢复与回滚 | 整套恢复后版本、配置和权限一致，不复用旧执行授权 |

**上述实机项目当前均不能视为通过。新版是否改善现场停滞问题、是否具备预期导航效果，需要测试后给出结论。**

## 8. 版本定位与联调资料

| 项目 | 地址或标识 |
| --- | --- |
| 主仓库 | `https://github.com/Superandyfre/r1_navigation.git` |
| 协作分支 | `codex/navigation-environment-snapshots` |
| 机器人主源码 | `/home/rpp/r1_navigation` |
| UI7 发布包 | `/home/rpp/neupan_failstop_ws/release_bundles/20261029-unified-01` |
| 发布包构建源码 | `dc5c6a736efdbfb86772910563e9d24317d6ed28` |
| 发布清单 | 发布包内 `releases/a-plus/manifest.json` |
| 版本定位记录 | `docs/UI7_UNIFIED_VERSION_20261029.json` |
| 执行归属协议 | `packages/navigation_neupan_bridge/EXECUTION_OWNERSHIP.md` |
| CogNav 参考适配 | `integrations/cognav/README.md` 及 `integrations/cognav/robot_live/` |

分支后续可能前进，联调应记录双方具体 commit 及实际发布清单，不使用“最新版本”作为唯一版本标识。Git 源码不包含全部地图、依赖环境和安装产物；需要恢复机器人环境时由导航端按 `docs/restore.md` 提供对应材料。

联调前 CogNav 侧提供：driver/bridge 版本、启动命令和运行主机、路径坐标系、父任务租约与输入时效、取消/重试策略、Action/topic 映射。导航侧提供：实际发布身份、有效参数、运行组件来源、定位与状态样本、已知问题及现场条件。双方先确认任务独占、失联处理和故障恢复边界，再开始接入测试。
