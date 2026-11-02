# UI7 顿挫/摇摆观察与诊断编码缺陷

实机运行包确认：20261102-passage-diagnostic-05。用户报告行驶顿挫、左右摇摆；本次接入采样时各级指令已经是零，不能从这段数据确认摇摆起源。日志中的多次 execution_context_changed 表示正在求解期间路径/任务上下文发生改变，旧结果被丢弃；尚无同步移动轨迹证明它是顿挫主因。禁止据此放宽身份或时效检查。

采样期间发生明确故障：scheduler_exception，ValueError: Out of range float values are not JSON compliant，Worker/Controller/Guard 随后锁存停车。最后发布的规划语义 arrive=true，最后上下文路径为重复终点位置、不同朝向；没有持久化失败周期的完整语义，不能断言具体异常字段已现场确认。

离线复现：空 optimized horizon 使路径偏差函数返回 inf；该值是原有停车判断的无效测量哨兵，但原语义消息原样编码 inf，触发相同 JSON 错误。修正仅将这两个诊断测量的不可用值编码为 null，增加 measurement_valid；控制判断仍接收原始值，任意非有限动作/轨迹仍被拒绝，未全局过滤 NaN/Inf。3 个序列化回归通过，覆盖空轨迹、原 NumPy 类型及无效数据拒绝。源码修正尚未部署，不宣称摇摆问题已解决。

证据：/home/rpp/neupan_failstop_ws/reports/oscillation-20261102/。下一次采集需覆盖实际移动时段，比较原始 NeuPAN 转向、稳定器/换挡等待、Controller、平滑器、Guard、odom 与 path_version 的同步变化，才能确定抑制顿挫/摆动的修改位置。

## 诊断异常隔离修复与部署

后续实现提交 dcb9b81：将已经完成运动结果验证和发布之后的 timing/semantic 构造、编码与发布放入独立异常边界。异常不调用 scheduler fatal 回调；记录 diagnostics_degraded、cycle_id、错误、累计次数及限频 traceback，并将简明事件写入 diagnostic_events.jsonl。错误处理本身的日志或 DDS 异常也被隔离。动作、轨迹、IPC、来源身份及其 JSON 编码仍在隔离边界之外，保留原有失败停车行为。该边界不延长健康或来源有效期；消费者若失去诊断，不会因此获得运动许可。

49 项单元回归通过，包含空轨迹不可用偏差、非有限动作/来源拒绝及诊断错误隔离。旧执行上下文测试夹具补齐现有 SolveBudget 初始化。真实 Worker/NeuPAN、Controller、平滑器和 Guard 的域 192 回归：连续 8 次诊断发布异常，后续新鲜来源和非零输出继续，取消与新目标正常，无致命锁存；真正 Worker 故障仍最终零输出并结束任务。测试仅使用合成传感器，无物理运动。新测试首次未等足 .5 秒即断言输出，修正测试等待后通过；未修改生产超时。

发布包 /home/rpp/neupan_failstop_ws/release_bundles/20261102-diagnostic-isolation-06，构建/工具源码 dcb9b81dc1c4a7d91fc28f1bc7dcc5b9f88c55df。8 包独立构建、594 个 install 文件、773 个包内文件完成校验。UI7 固定入口已切换至本包；未启动新实机导航，实机验收仍未完成。验收和部署清单见 /home/rpp/neupan_failstop_ws/reports/oscillation-20261102/fix06-deployment.json。

此前监控数据库在录制期间发生 SQLite 锁冲突并退出，旧包已 reindex 保留。新录制 /home/rpp/neupan_failstop_ws/reports/live-navigation-20261102-161148/ 后台运行，只检查进程和日志，不查询写入中的数据库。录制重新发现重启后的话题；不能声称中断期间的导航已录到。

本次修复解决诊断引起的错误锁存，不证明此前顿挫或左右摇摆已解决，也不清除正在运行组件的真实故障。旧导航已退出时，下一次 UI7 启动将建立全新的运行上下文；不自动发送或重放旧目标。
