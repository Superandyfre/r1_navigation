# Deployment inventory

这些文件是 `snapshots/20261029-baseline-01` 的采集结果，不是机器曾于10月16日
运行状态的推断，也不表示实际导航已验证。

- dependencies.repos：29个实际Git根及准确commit；路径映射和脏状态见快照
  sources/repositories.json。完整source归档还覆盖未被独立Git管理的源码。
- python.lock：两个解释器环境的精确已安装版本；不是带wheel哈希的pip安装锁。
- system-packages.lock：本机dpkg实际安装包、版本、架构。
- build-settings.yaml：实际CMakeCache中的编译器、参数、来源/依赖目录。
- profiles/r1-hw1.json：下次采集的显式路径输入，不自动改动现场程序。

不得手改这些清单冒充已验证版本。重新采集使用新快照ID，注册前保留旧快照索引；
Git中的最新清单可能前进，恢复旧版要先checkout对应基线标签。大文件和未提交补丁
在快照存储，不进入普通Git历史；恢复步骤见 ../docs/restore.md。
