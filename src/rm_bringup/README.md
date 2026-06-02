## rm_bringup README

本包提供整车启动（bringup）以及两类常用“工程化能力”：
- **rosbag2 全话题内录（db3）+ 一条命令回放**：运行时自动录制、支持分段、便于 RViz 回看

### 快速开始
启动（默认：自动开始全话题 rosbag2 录制；不自动保存参数）：

```bash
ros2 launch rm_bringup bringup.launch.py
```

### rosbag2 全话题录制（db3）与回放
#### 启动即自动录制（默认开启）
bringup 默认会自动执行全话题录制：
- `ros2 bag record -a --storage sqlite3 ...`
- 保存到：`~/rosbags/rm_bringup/<YYYYMMDD_HHMMSS>/`
- 支持按时长分段（默认 300 秒）

#### 常用录制参数（launch 参数）
- **关闭自动录制**：

```bash
ros2 launch rm_bringup bringup.launch.py enable_bag_record:=false
```

- **修改保存根目录**（默认 `~/rosbags/rm_bringup`）：

```bash
ros2 launch rm_bringup bringup.launch.py bag_root_dir:=~/rosbags/my_project
```

- **修改分段时长（秒）**（默认 300；设 0 表示不分段）：

```bash
ros2 launch rm_bringup bringup.launch.py bag_split_duration_s:=600
```

- **可选压缩（按需开启）**：

```bash
ros2 launch rm_bringup bringup.launch.py bag_compression_mode:=file bag_compression_format:=zstd
```

#### 一条命令回放所有话题（推荐带 --clock）

```bash
ros2 bag play ~/rosbags/rm_bringup/20260407_213000 --clock
```

也可以用脚本减少输入：

```bash
ros2 run rm_bringup bag_play_all.sh ~/rosbags/rm_bringup/20260407_213000
```

#### （可选）不通过 bringup，手动开始全话题录制

```bash
ros2 run rm_bringup bag_record_all.sh
ros2 run rm_bringup bag_record_all.sh ~/rosbags/rm_bringup 600
```

### bag 转 MP4（离线，不影响运行时实时性）
将某个图像话题导出为 MP4：

```bash
ros2 run rm_bringup bag_to_mp4.py \
  --bag ~/rosbags/rm_bringup/20260407_213000 \
  --topic /camera_driver/image_raw \
  --out ~/rosbags/rm_bringup/20260407_213000/camera.mp4 \
  --fps 30
```

常用参数：
- **`--fps`**：0 表示从时间戳推断；失败回退 30
- **`--fourcc`**：默认 `mp4v`；如 OpenCV/FFmpeg 支持 H.264 可尝试 `avc1`
- **`--storage`**：默认 `sqlite3`（db3）；如果录的是 mcap，填 `mcap`

依赖（Ubuntu 22.04）：

```bash
sudo apt update
sudo apt install -y python3-opencv python3-numpy
```

### 注意事项
- **全话题 `-a` 数据量很大**：图像/点云会快速占满磁盘，建议 SSD，并合理设置分段时长
- **回放建议带 `--clock`**：需要的话在 RViz 中启用 `use_sim_time`

