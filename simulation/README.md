# Mecanum robot simulation — LTMang

## Mục đích
Đây là bản sao **mã nguồn và tài nguyên mô phỏng** robot Mecanum phục vụ
đề tài khảo sát Wi-Fi bằng ROS 2. Nguồn làm việc trên PC Ubuntu được
snapshot vào repo, không đụng tới Mini PC robot thật.

## Liên hệ với phần RSSI hiện hữu
- `rssi_sim.py` tại root repo: nhận `/map`, đọc TF `map -> base_footprint`,
  phát `/wifi/rssi_dbm`, ghi CSV; không phải dữ liệu Wi-Fi thật.
- `analyze_rssi.py`: tổng hợp CSV thành heatmap theo ô, MAD và IDW.
- `result_te1n/`: kết quả thí nghiệm cũ, được giữ nguyên.
- `maps/` ở root: bản đồ lưu từ thí nghiệm trước, được giữ nguyên.
- `simulation/maps/`: bản sao **my_map** cùng world đang chạy trên PC.

## Bố cục snapshot
```text
simulation/
  launch/mecanum_gazebo_corridor_repo.launch.py    Gazebo corridor từ repo
  ros2_ws/src/mecanum_robot_viz/                  URDF, meshes, launch và package
  ros2_ws/src/mecanum_visual_animator/             Plugin C++ hiển thị bánh + /joint_states
  ros2_ws/src/<optional DWB critic>/             Nếu tìm thấy plugin PreferForward custom
  config/nav2_params_mecanum_sim.yaml             Nav2/AMCL/footprint/use_sim_time
  config/nav2_recovery_backup_first.xml           Custom Behavior Tree (nếu có)
  maps/my_map.yaml + <image>                      Map tương ứng
  maps/my_map_corridor.world                      Gazebo Classic world
  models/                                        Model SDF cục bộ được world tham chiếu (nếu có)
  scripts/mecanum_safe_cmd_relay.py               Giới hạn /cmd_vel, deadman
  scripts/sim_env.sh                              ROS_DOMAIN_ID=31, overlay, plugins
  scripts/run_{gazebo,localization,navigation,rviz}.sh
  MANIFEST.sha256                                 SHA256 snapshot
```

## Xây dựng workspace của bản sao (trên Ubuntu 22.04, ROS 2 Humble)
```bash
cd ~/LTMANG/simulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```
Nếu thiếu plugin hoặc package trên máy khác, hãy cài ROS Humble,
`gazebo_ros`, Nav2, `nav2_bringup`, plugin DWB bổ sung và các dependencies.
`colcon build` **không bảo đảm** file URDF đúng nếu thiếu mesh.

## Chạy (mỗi lệnh trong terminal riêng)
```bash
bash ~/LTMANG/simulation/scripts/run_gazebo.sh
bash ~/LTMANG/simulation/scripts/run_localization.sh
bash ~/LTMANG/simulation/scripts/run_navigation.sh
bash ~/LTMANG/simulation/scripts/run_rviz.sh
```
Không chạy đồng thời với phiên Gazebo/relay/RSP cũ. Map là `my_map.yaml`,
world `my_map_corridor.world`, spawn mặc định `(5, 0, 0.25)` yaw 0.
Ban đầu `AMCL initial_pose=(5,0,0)` chỉ hợp lệ nếu world và map căn chỉnh.
RViz fixed frame `map`; kiểm tra `map -> odom -> base_link -> base_footprint`.

## Trở lại thí nghiệm khảo sát RSSI
```bash
cd ~/LTMANG
source simulation/scripts/sim_env.sh
python3 rssi_sim.py --ros-args -p use_sim_time:=true \
    -p ap_x:=-3.0 -p ap_y:=-1.0 -p csv:=/tmp/ltmang_rssi.csv
python3 analyze_rssi.py /tmp/ltmang_rssi.csv --ap -3.0 -1.0 \
    --map simulation/maps/my_map.yaml --out result_new
```
Vị trí AP `(-3,-1)` là ví dụ thí nghiệm cũ; thay theo bài thử thực tế.

## Các ràng buộc cần kiểm tra
1. Gazebo URDF có plugin `safe_mecanum_planar` và `mecanum_gui_only_wheels`;
   RViz URDF dùng joint `continuous`, lấy `/joint_states` từ plugin Gazebo.
   **Không** chạy thêm `mecanum_rviz_joint_states.py` (sẽ trùng publisher).
2. `package://mecanum_robot_viz/meshes/...` phải được cài vào ROS overlay.
3. File Nav2 từng trỏ tới `~/nav2_recovery_backup_first.xml`; nếu chuyển máy
   phải cập nhật `default_nav_to_pose_bt_xml` trong YAML sang file dưới `config/`.
4. Safe relay giới hạn vận tốc có thể thấp hơn DWB. Đối chiếu `/cmd_vel`
   với `/cmd_vel_gazebo` khi thử Nav2; không phát teleop cùng Nav2.
5. Footprint trong YAML là từ bản Nav2 đã hiệu chỉnh; cần xác minh nó
   thực sự bao toàn bộ thân và bánh mới trước thử tránh vật cản.
6. World/map trong repo là bản snapshot. Đừng coi RSSI mô phỏng là phép
   đo RF thật, và đừng coi Gazebo planar_move là mô hình động học/ma sát bánh.

<!-- LTMANG_MANUAL_TELEOP_BEGIN -->
## Điều khiển thủ công bằng Keyboard Teleop

Package: `ros2_ws/src/keyboard_teleop/` (ROS 2 Python, phím bấm đọc qua TTY).
Script: `scripts/run_teleop.sh`. Dùng thay cho `run_navigation.sh` để tránh
hai publisher tranh nhau `/cmd_vel`. AMCL vẫn chạy để duy trì TF cho RSSI.

```bash
# Build lần đầu (hoặc sau khi cập nhật package)
cd ~/LTMANG/simulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select keyboard_teleop

# Terminal 1: Gazebo
bash ~/LTMANG/simulation/scripts/run_gazebo.sh
# Terminal 2: AMCL
bash ~/LTMANG/simulation/scripts/run_localization.sh
# Terminal 3: RViz2 (tùy chọn)
bash ~/LTMANG/simulation/scripts/run_rviz.sh
# Terminal 4: Điều khiển từ bàn phím, KHÔNG chạy Nav2 Navigation
bash ~/LTMANG/simulation/scripts/run_teleop.sh
```

Phím: `w` tiến, `x` lùi, `a/d` quay, `q/e` trượt ngang,
`1/2/3` tốc độ chậm/thường/nhanh, `s` hoặc Space dừng, `z` thoát.
Chương trình duy trì vận tốc cho đến khi có lệnh mới; luôn nhấn `s`
để dừng robot sau mỗi thao tác. Các giới hạn lệnh có thể bị relay giảm thêm.

Khi TF `map -> base_footprint` hoạt động, chạy `rssi_sim.py` ở root để ghi CSV,
sau đó chạy `analyze_rssi.py`. Xem [README gốc](../README.md).
<!-- LTMANG_MANUAL_TELEOP_END -->
