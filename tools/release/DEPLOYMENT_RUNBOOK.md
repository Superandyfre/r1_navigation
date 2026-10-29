# A+ 整套部署、故障处理与回滚操作书（待演练）

状态：DRAFT / NOT_DEPLOYABLE。本文是待审核的操作顺序，不是现场执行批准。
候选05首轮全30项因验收Python SIGSEGV中断；Action显式清理修正后的重跑仍待结论。
本表只是未批准版本的定位记录，不代表可接受的回滚目标。
截至候选05/bundle04，生产UI7未切换。测试使用合成scan/TF/odom、独立domain192；
不能用其零速记录证明底盘停车。没有完成验收的“上一套A+已知可靠版本”。
保留的旧生产版本和bundle01只可作审计/比较对象，不能标为安全回滚目标。

## 固定对象与版本记录

| 对象 | 当前路径/含义 |
|---|---|
| 生产UI7 | /home/rpp/r1_navigation/tools/starterbutton/start_navigation_high_sampling_neupan.sh |
| 开发源码 | /home/rpp/neupan_failstop_ws/src/r1_navigation |
| 候选05 | /home/rpp/neupan_failstop_ws/release_candidates/20261029-a-plus-05 |
| bundle04 | /home/rpp/neupan_failstop_ws/release_bundles/20261029-a-plus-04 |
| 候选05构建SHA | 824adfd43426621e2de30b9fd1cb8c8d7438dd2d |
| 候选清单 | candidate05根目录/candidate-inventory.json |
| bundle清单 | bundle04根目录/releases/a-plus/manifest.json |
| 验收证据 | /home/rpp/neupan_failstop_ws/reports/neupan_failstop_20261027 |

上表路径中的candidate05/bundle04简称只用于说明；命令使用下列绝对路径。
记录构建SHA、工具SHA、清单SHA和批准人/时间；原有日志、源码改动、ledger、
旧候选目录均保留。新版本写入新目录。审批清单由独立审核方保管，不能通过
重算哈希或自行改deployment_approved绕过未完成的验收。

整套版本必须包含：ControllerServer可执行/core、共享runtime接口、Controller
插件、Guard、Worker/桥接/context、neupan_ros2、BT插件和两份XML、配置/模型/
planner、launch、入口脚本、CogNav协调器、NeuPAN源码及外部运行依赖。
版本组合接受规则见原计划§11.3；旧server/plugin/Guard不能作为新A+的兼容替身。

## S0：现场运动隔离与当前实例留证

以下现场步骤必须由操作员确认底盘停止、运动链被物理禁用后执行；禁止自动
解除急停、向实际/cmd_vel发送测试速度或热替换正在运行的插件。具体物理
隔离办法、底盘watchdog/firmware值和停车判据仍需现场确认，未确认则停在S0。

1. 记录操作员、隔离方式/时间、实际odom停止证据、驱动/固件状态。
2. 保存生产入口脚本和UI7注册目标、Git状态/diff、源文件/安装文件哈希、
   活跃PID及start_ticks、进程命令行、ROS域/环境、节点/话题/发布者图、
   参数dump、/proc/PID/maps和原始日志/故障时间线；敏感环境项不公开记录。
3. 单独记录Worker/Guard/controller boot、Action UUID、首次fault_id/code、
   当前grant和路径/目标版本；ledger只复制快照，不编辑或清空。
4. 由操作员批准后停止这次明确识别的launch进程组及其所有运动组件。
   不使用模糊pkill；验证Worker子进程、Controller、smoother、Guard和协调器
   都已退出，防止两个实例共存。Guard退出后的物理停止必须由独立驱动机制保证。

## S1：离线审核和兼容检查

只读兼容诊断命令（不启动节点、不授予运动许可）：

```bash
source /home/rpp/neupan_failstop_ws/release_bundles/20261029-a-plus-04/install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
/usr/bin/python3 /home/rpp/neupan_failstop_ws/release_bundles/20261029-a-plus-04/tools/release/preflight.py \
  --manifest /home/rpp/neupan_failstop_ws/release_bundles/20261029-a-plus-04/releases/a-plus/manifest.json \
  --compatibility-only
```

诊断通过不代表可部署；当前部署模式应退出2，因为批准/验收门禁为false。
不要在正在运行生产导航的shell里source候选后继续操作生产节点。

尚须补齐并审核：ROS/venv/native solver/smoother/驱动完整外部依赖清单、
实际校准文件/home/rpp/rpp_ws/config/r1_hw1_calibration.json、地图、profile、
所有launch覆盖参数、.rpprc来源和环境、UI7注册、完整回滚目标及其验收证据。
启动入口目前允许scan_source/mesh/collision相关环境覆盖；必须核验实际生效值，
不能仅审核YAML默认值。当前preflight未证明这些全部事实，未齐则禁止进入S2。

## S2：整套切换和禁运动启动（尚未执行）

保持S0物理隔离。整套指向审核后的独立release目录，不向生产目录逐文件复制；
先记录可恢复的旧UI7目标及所有启动引用，再一次切换入口/环境选择。
确切UI7注册机制及原值须现场审计后形成操作记录，目前不提供未经核实的改写命令。
任何一步失败都保留运动隔离，进入R0。

1. 全部旧运动实例退出后才启动新整套；不得复用旧Action/授权/缓存。
2. 无arm、无目标期间观察最终输出持续零；核验无其他实际/cmd_vel发布者。
3. 验证实际Worker解释器/import和NeuPAN/模型路径；对实际Controller与Guard
   的PID/start_ticks执行runtime_maps.py核验四个C++角色，保存输入与输出。
   工具使用--manifest和--bindings两个参数，bindings必须是本轮实际进程身份。
4. dump实际参数，逐项与审核版本核对：协议/topic/remap、超时、授权强制开启、
   自动恢复关闭、collision/footprint/localization、安全profile和BT等。
   arm前有效参数与maps门禁尚未集成；目前此步骤不能以静态检查代替。
5. 验证显式arm与新Action是两个动作，任何FATAL后旧进程arm必须拒绝，
   仅改参数/重启一个组件/重放ledger不得恢复许可。

## S3：现场验收及放行（尚未执行）

须先完成全部无运动验收并由操作员批准实机步骤。验收方案使用原计划T01–T50和
Gate判据；除Action终态、最终输出零外，分别测实际cmd_vel、odom、停车时间/
距离、驱动/固件watchdog。还须验证正常导航、动态障碍等待、取消/抢占、故障
再发目标、整套受控重启及回滚。记录原始bag/时钟基准和明确阈值，未有现场
阈值不能宣称通过。不在本文提供自动arm/发目标命令。

## R0：失败后的整套回滚（待隔离演练/现场验证）

1. 保持物理隔离/停止；保留失败进程证据、参数、日志、首次故障、ledger快照。
2. 按S0识别并停止新整套所有相关进程，确认无残余运动发布者或solver子进程。
3. 只有经审核的整套回滚目标可进入下步。当前未有这样的A+目标；若只有旧生产
   基线，可恢复其文件/入口供停用状态下检查，但不能因此解除隔离或标为A+可靠。
4. 一次恢复旧入口及整套依赖/配置/BT/launch/协调器；不得只换Python或.so。
5. 在全新进程中重复S1/S2身份、实际参数和零速启动核验。新的boot/epoch必须
   重新记录，不继承旧授权；按目标版本规则处理ledger且始终保留原快照。
6. 回滚后仍需现场认可的启动/停车验收，之后才可重新申请运动许可。

禁止通过清除运行中fault、重设generation、刷新旧消息时间戳完成所谓回滚。

## 记录与退出条件

每步记录pre/post PID/start_ticks、路径/哈希、boot/Action/grant、开始结束时间、
原始命令输出和批准记录。任何加载/参数/消息语义不匹配、无法解释的停车或
旧结果复现：保持禁运动，归档证据、停止放行。独立恢复物理停止与Action失败
必须各有证据。当前历史worker_health_stale与配置服务响应超时仍OPEN。

文档完成不等于演练完成；deployment gates仍false。待补交：UI7实际注册切换、
完整依赖/有效参数门禁、可接受的整套回滚目标、隔离切换演练、现场停车/回滚证据。
