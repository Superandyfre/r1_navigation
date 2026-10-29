# 导航版本、基线快照与恢复

`r1_navigation` 是主仓库；Git 只存源码、工具、依赖清单、机器人配置和快照索引。
`/home/rpp/navigation_snapshots/<ID>/` 存实际源码 bundle、源文件/未提交修改、
ROS/底层 install、Python环境、地图模型及运行状态。JSON 格式的 `.yaml/.repos`
文件是合法 YAML 1.2，工具只依赖 Python 标准库。

## 当前状态与边界

主仓库工作树、A+开发工作树和 UI7 正在使用的发布包分别记录，不能用单个 commit
概括三者。`dependencies.lock.yaml` 是旧的人工参考，新的完整采集以
`deployment/dependencies.repos` 和快照仓库清单为准。脏工作树会保留二进制补丁、
未跟踪清单和实际源文件压缩包；不自动提交别人的求解器改动。

`captured`、`built`、`restored`、`navigation_validated` 独立记录。当前采集不是
10月16日历史状态重建，不是实机验收；不得生成 `*-validated-*` 标签。
没有现场正常导航录包时明确 NOT_RUN，不为采集自动启动导航、arm 或发布目标。
外部依赖可能有绝对路径、外链和系统本地库，单凭源码和版本锁不能宣称离线恢复。

## 采集（拒绝覆盖同名快照）

```bash
cd /home/rpp/r1_navigation
/usr/bin/python3 tools/snapshot/capture.py --id 20261029-baseline-01 --runtime
/usr/bin/python3 tools/restore/restore.py /home/rpp/navigation_snapshots/20261029-baseline-01
/usr/bin/python3 tools/snapshot/register.py /home/rpp/navigation_snapshots/20261029-baseline-01
```

`--runtime` 只读取实际 ROS 图/参数/TF；每条命令有超时。无运行节点、服务失联、
TF缺失必须查看 runtime 的 returncode，不能把一次空图当作导航已验收。
采集不 source `.rpprc`；明确依次加载 ROS、底层 install、UI7 实际发布 install。
全局脚本只作历史配置文件留存，不执行。参数内容与实际进程 maps 只存私有快照。
私有目录权限0700；排除凭证文件名及疑似口令赋值，排除项单列；不是任意Git历史
的秘密扫描器，外发前仍需审查git bundle历史和运行参数，令牌不得进入Git索引。

Git bundle 包含可达历史；源文件压缩包包含实际工作树，恢复时不能只checkout HEAD
后忽略补丁。未独立Git管理的底层源码仍包含在 underlay-source 归档中。
快照保留完整 venv、ROS 和已验证导航 install；它们是同机环境归档，不是通用镜像。
系统包版本已锁定，但没有收齐所有 deb/wheel/CUDA外部库时 offline_complete=false。
后续离线全恢复需补完整OS/依赖镜像或与锁定版本匹配的deb/wheel仓库再独立验证。

## 核验与恢复到隔离目录

1. 从 Git 的 `snapshots/<ID>/manifest.yaml` 查存储位置；首先检查 manifest 和
   checksums 文件 SHA256 是否与Git指针一致，然后运行恢复工具核验全部快照文件。
2. 使用 `--compare-live` 对比当下原路径的普通文件/符号链接。该操作不写运行目录，
   也不证明进程内存代码与文件相同；已加载来源另查 runtime/package-prefixes.json
   和 runtime/processes.json 中的 maps、解释器/进程路径。
3. 隔离展开指定归档，拒绝覆盖现有目录与路径逃逸：

```bash
/usr/bin/python3 tools/restore/restore.py /home/rpp/navigation_snapshots/20261029-baseline-01 \
  --artifact payload/calibration.tar.gz --destination /home/rpp/navigation_restore/calibration-test
```

符号/硬链接保存在 DEFERRED_LINKS.json，工具不自动创建。恢复环境前审核每个
绝对链接和库依赖，不能把“展开成功”当作“可运行”。默认不解包到生产原路径。

4. 从 sources/*.bundle 执行 `git clone <bundle> <新目录>`，按 repositories.json 的
   commit checkout。源文件归档才是脏工作树的最终内容；或按 HEAD→index patch→
   worktree patch 恢复并核对未跟踪文件（worktree patch 已相对HEAD，不要双重应用；
   index patch仅用于恢复暂存状态）。build-settings、Python和系统锁用于新环境核对，
   不盲目安装到现有机器人。编译在新工作区进行。
5. 全链恢复后核对实际包前缀、Python模块、共享库maps、参数/TF、地图/标定校验值，
   保持底盘禁运动完成启动/回滚验证；实机导航与停车验收另由操作员明确授权。

## 标签与不可移动性

采集基线使用 `nav-baseline-<ID>`；真实验收完成并归档证据后才可使用
`nav-<日期>-validated-<序号>`。标签应指向提交了索引和锁定清单的commit；标注说明
包含快照manifest SHA256。创建前检查标签不存在，禁止 `git tag -f`。
本地Git不能阻止管理员删改标签；远程仓库需配置受保护标签和备份。不要自动推送。

## 启动环境迁移

新增 `tools/starterbutton/navigation_environment.bash` 显式加载固定 ROS/底层环境，
主仓库UI7脚本改用该文件，不再依赖 `.rpprc`。当前操作台仍指向已验收范围记录中的
bundle09，本次仓库整理不热替换它；该旧包的环境脚本依赖已被快照记录。下一次
打包切换需带上新环境文件并重新核对包来源，不能宣称旧发布已同步这一改动。
