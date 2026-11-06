# UI7 只读诊断录包

在已设置好与导航相同 ROS 域和环境的终端中执行：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1
bash /home/rpp/r1_navigation/tools/diagnostics/record_ui7.sh /absolute/new-recording
# 观察完成后 Ctrl-C，等待 rosbag 完整退出；不要分析仍在写入的数据库。
/usr/bin/python3 /home/rpp/r1_navigation/tools/diagnostics/analyze_ui7_bag.py \
  /absolute/new-recording/bag --output /absolute/new-recording/analysis.json
```

录制程序只订阅，不发布控制指令；扫描、TF 和关节数据可能占用较大磁盘和 CPU，
现场应先确认存储空间和负载。话题发现是动态的，录包中缺失的话题不代表该节点健康。
分析支持 sqlite3 rosbag，记录接收间隔、状态变化及速度从零变非零的时间。接收时间不是
传感器采样时间，相关性不等于首因；实际停车须结合里程计和硬件观察。单个空录包或
不存在的类型会在结果中体现，不能当作通过。设备以外的身份、调度和求解耗时请结合
Worker 的 `/neupan_timing` 和日志进一步分析。本工具不改变任何速度阈值。
