# UI7 顿挫/摇摆观察与诊断编码缺陷

实机运行包确认：20261102-passage-diagnostic-05。用户报告行驶顿挫、左右摇摆；本次接入采样时各级指令已经是零，不能从这段数据确认摇摆起源。日志中的多次 execution_context_changed 表示正在求解期间路径/任务上下文发生改变，旧结果被丢弃；尚无同步移动轨迹证明它是顿挫主因。禁止据此放宽身份或时效检查。

采样期间发生明确故障：scheduler_exception，ValueError: Out of range float values are not JSON compliant，Worker/Controller/Guard 随后锁存停车。最后发布的规划语义 arrive=true，最后上下文路径为重复终点位置、不同朝向；没有持久化失败周期的完整语义，不能断言具体异常字段已现场确认。

离线复现：空 optimized horizon 使路径偏差函数返回 inf；该值是原有停车判断的无效测量哨兵，但原语义消息原样编码 inf，触发相同 JSON 错误。修正仅将这两个诊断测量的不可用值编码为 null，增加 measurement_valid；控制判断仍接收原始值，任意非有限动作/轨迹仍被拒绝，未全局过滤 NaN/Inf。3 个序列化回归通过，覆盖空轨迹、原 NumPy 类型及无效数据拒绝。源码修正尚未部署，不宣称摇摆问题已解决。

证据：/home/rpp/neupan_failstop_ws/reports/oscillation-20261102/。下一次采集需覆盖实际移动时段，比较原始 NeuPAN 转向、稳定器/换挡等待、Controller、平滑器、Guard、odom 与 path_version 的同步变化，才能确定抑制顿挫/摆动的修改位置。
