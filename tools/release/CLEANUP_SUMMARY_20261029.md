# 本次规划产物累计清理汇总（2026-10-29）

按用户要求汇总从本次规划开始至今各轮删除清单，非单轮统计。

| 类别 | 原始文件大小 GB（十进制） |
|---|---:|
| 旧编译产物 | 4.583 |
| 重复临时发布包 | 0.301 |
| 废弃候选及旧包 | 4.915 |
| 历史原始测试数据 | 4.787 |
| candidate07重复构建安装 | 1.043 |
| 系统重复崩溃报告 | 0.044 |
| 合计 | 15.673 |

历史测试数据保留为 431.8 MB 的归档；17,030文件均经解压流SHA256比对后才删除散落原件。累计净减少逻辑大小（扣除新归档）约 15.24 GB。Btrfs实际回收量与逻辑文件大小不同。

保留核心工作树、仍被 symlink-install 引用的开发 build、当前 bundle09、上一版 bundle08、最新三项关键测试场景、精简报告、归档索引与回滚配置。两套发布包均重新核验完整性。

本任务临时 ledger、UUID 探针、临时脚本、Python缓存、/var/tmp候选重复构建安装及候选残留已清理。系统崩溃报告仅在与已归档本任务报告字节相同后删除。/home下同名历史NeuPAN文件早于本任务，其他项目及机械臂/历史ROS日志未纳入删除。CogNav核心源码未改，未启动导航。

旧验收现场散落目录已合并，历史报告所指的tests旧路径可依据archive-index.json在归档中恢复。旧候选运行包已退役，不再声称可直接重放。后续重新构建时需从核心源码构建，不能再使用已删除candidate07/install。

详细清单：/home/rpp/neupan_failstop_ws/reports/neupan_failstop_20261027/final-cleanup-20261029/cumulative-summary.json
历史证据：/home/rpp/neupan_failstop_ws/reports/neupan_failstop_20261027/final-cleanup-20261029/historical-test-artifacts.tar.gz
