#!/usr/bin/env python3
"""Snapshot the working ROS 2 Humble Mecanum simulation into ~/LTMANG.

Default: inspect without modifying files. --apply: copy & validate.
--apply --push: additionally commit *scoped* repository files and push main.
Never modifies the original robot workspace or the real robot Mini PC.
"""
from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import textwrap
import xml.etree.ElementTree as ET

HOME = Path.home()
EXCLUDE_DIRS = {'.git', '__pycache__', '.pytest_cache', '.mypy_cache', '.vscode',
                'build', 'install', 'log', 'logs', 'venv', '.venv'}
EXCLUDE_SUFFIXES = {'.pyc', '.pyo', '.swp', '.swo', '.bak', '.tmp', '.db3', '.mcap', '.so', '.o', '.a', '.dll', '.dylib', '.pem', '.key'}
EXCLUDE_NAMES = {'.env', '.env.local', '.env.production', '.aws', '.ssh',
                 'id_rsa', 'id_ed25519', 'credentials.json', 'token.json',
                 'service_account.json', 'service-account.json'}


def sh(*args: str, cwd: Path | None = None, check=True) -> subprocess.CompletedProcess:
    return subprocess.run(args, cwd=cwd, text=True, capture_output=True, check=check)


def copy_ignore(_directory: str, names: list[str]) -> set[str]:
    return {name for name in names if name in EXCLUDE_DIRS or name in EXCLUDE_NAMES or
            Path(name).suffix.lower() in EXCLUDE_SUFFIXES or
            name.endswith('~') or name.startswith('.#')}


def cptree(src: Path, dst: Path) -> None:
    shutil.copytree(src, dst, dirs_exist_ok=True, ignore=copy_ignore,
                    copy_function=shutil.copy2)


def copyfile(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)


def find_map_image(path: Path) -> Path:
    text = path.read_text(encoding='utf-8')
    m = re.search(r'^\s*image:\s*(.+?)\s*(?:#.*)?$', text, re.M)
    if not m:
        raise RuntimeError('Map YAML lacks image: ' + str(path))
    value = m.group(1).strip().strip('"\'')
    if not value:
        raise RuntimeError('Empty image: ' + str(path))
    image_path = Path(value)
    if not image_path.is_absolute():
        image_path = path.parent / image_path
    if not image_path.is_file():
        raise RuntimeError(f'Map image not found: {image_path}')
    return image_path.resolve()


def set_map_image_local(map_yaml: Path, image: Path) -> None:
    data = map_yaml.read_text(encoding='utf-8')
    data, count = re.subn(r'^(\s*image:)\s*.+$',
                          lambda m: m.group(1) + ' ' + image.name,
                          data, count=1, flags=re.M)
    if count != 1:
        raise RuntimeError('Could not update copied map YAML image path')
    map_yaml.write_text(data, encoding='utf-8')


def urdf_mesh_check(pkg: Path, urdfs: list[Path]) -> list[str]:
    problems = []
    for f in urdfs:
        root = ET.parse(f).getroot()
        for mesh in root.findall('.//mesh'):
            uri = mesh.get('filename', '')
            if uri.startswith('package://mecanum_robot_viz/'):
                relative = uri.removeprefix('package://mecanum_robot_viz/')
                if not (pkg / relative).is_file():
                    problems.append(f'{f.name}: missing mesh {relative}')
            elif uri.startswith('package://'):
                problems.append(f'{f.name}: extra ROS package needed for {uri}')
            elif uri.startswith('file://'):
                problems.append(f'{f.name}: file URL may be machine-specific: {uri}')
        # Gazebo-specific: ensure plugins remain in the Gazebo URDF, not RViz.
    return problems


def include_local_world_models(world: Path, sim: Path) -> list[str]:
    """Vendor locally available model:// dependencies. Gazebo builtins stay external."""
    names = set()
    root = ET.parse(world).getroot()
    for elem in root.iter():
        text = (elem.text or '').strip()
        if text.startswith('model://'):
            name = text[len('model://'):].split('/', 1)[0]
            if name and name not in {'ground_plane', 'sun'}:
                names.add(name)
    search_paths = [HOME / '.gazebo/models', HOME / 'mecanum_viz_ws/src',
                    HOME / 'ros2_ws/src']
    warnings = []
    for name in sorted(names):
        source = next((d / name for d in search_paths if (d / name).is_dir()), None)
        if source:
            cptree(source, sim / 'models' / name)
            print('Vendored model://', name, '<-', source)
        else:
            warnings.append('model://' + name)
    return warnings


def copy_related_dwb_packages(src_roots: list[Path], dstsrc: Path) -> list[str]:
    """Best effort: preserve custom PreferForward critic if source exists."""
    matches = []
    for srcroot in src_roots:
        if not srcroot.is_dir():
            continue
        for package_xml in srcroot.glob('*/package.xml'):
            package = package_xml.parent
            if package.name in {'mecanum_robot_viz', 'mecanum_visual_animator'}:
                continue
            for code_file in list(package.rglob('*.cpp')) + list(package.rglob('*.hpp')):
                if any(x in code_file.parts for x in EXCLUDE_DIRS):
                    continue
                if code_file.stat().st_size > 200_000:
                    continue
                text = code_file.read_text(encoding='utf-8', errors='ignore')
                if 'PreferForwardCritic' in text:
                    target = dstsrc / package.name
                    cptree(package, target)
                    matches.append(f'{package} -> {target}')
                    break
    return matches


def portable_launch() -> str:
    return textwrap.dedent('''\
        #!/usr/bin/env python3
        """Repo-local Gazebo corridor launcher: run after source sim_env.sh."""
        import re
        import sys
        import tempfile
        from pathlib import Path
        from ament_index_python.packages import get_package_share_directory
        from launch import LaunchDescription
        from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
        from launch.conditions import IfCondition
        from launch.launch_description_sources import PythonLaunchDescriptionSource
        from launch.substitutions import LaunchConfiguration
        from launch_ros.actions import Node

        SIM = Path(__file__).resolve().parents[1]
        ROBOT = SIM / 'ros2_ws/src/mecanum_robot_viz/urdf'
        GAZEBO_URDF = ROBOT / 'mecanum_robot_gazebo_real_visual.urdf'
        RVIZ_URDF = ROBOT / 'mecanum_robot_rviz_real_visual.urdf'
        WORLD = SIM / 'maps/my_map_corridor.world'
        RELAY = SIM / 'scripts/mecanum_safe_cmd_relay.py'

        def read_urdf(path):
            contents = path.read_text(encoding='utf-8-sig')
            return re.sub(r'^\\s*<\\?xml\\s+[^?]*\\?>\\s*', '', contents, count=1)

        def generate_launch_description():
            for path in (GAZEBO_URDF, RVIZ_URDF, WORLD):
                if not path.is_file():
                    raise FileNotFoundError(path)
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                    prefix='ltmang_spawn_', suffix='.urdf', delete=False) as f:
                f.write(read_urdf(GAZEBO_URDF))
                spawn_file = f.name
            gazebo_launch = Path(get_package_share_directory('gazebo_ros')) / 'launch/gazebo.launch.py'
            return LaunchDescription([
                DeclareLaunchArgument('world', default_value=str(WORLD)),
                DeclareLaunchArgument('x', default_value='5.0'),
                DeclareLaunchArgument('y', default_value='0.0'),
                DeclareLaunchArgument('z', default_value='0.25'),
                DeclareLaunchArgument('yaw', default_value='0.0'),
                DeclareLaunchArgument('use_relay', default_value='true'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(gazebo_launch)),
                    launch_arguments={'world': LaunchConfiguration('world')}.items()),
                Node(package='robot_state_publisher', executable='robot_state_publisher',
                     name='robot_state_publisher', output='screen', parameters=[{
                         'robot_description': read_urdf(RVIZ_URDF), 'use_sim_time': True,
                         'publish_frequency': 40.0,
                     }]),
                Node(package='gazebo_ros', executable='spawn_entity.py',
                     name='spawn_mecanum_corridor', output='screen',
                     arguments=['-file', spawn_file, '-entity', 'mecanum_robot',
                                '-x', LaunchConfiguration('x'),
                                '-y', LaunchConfiguration('y'),
                                '-z', LaunchConfiguration('z'),
                                '-Y', LaunchConfiguration('yaw')]),
                ExecuteProcess(cmd=[sys.executable, str(RELAY)],
                               name='mecanum_safe_cmd_relay', output='screen',
                               condition=IfCondition(LaunchConfiguration('use_relay'))),
            ])
    ''')


def environment_script() -> str:
    return textwrap.dedent('''\
        #!/usr/bin/env bash
        # Source me from EACH terminal. Build simulation/ros2_ws first.
        _LTMANG_SIM_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
        source /opt/ros/humble/setup.bash
        if [[ ! -f "$_LTMANG_SIM_DIR/ros2_ws/install/setup.bash" ]]; then
          echo "Missing built overlay: cd $_LTMANG_SIM_DIR/ros2_ws && colcon build --symlink-install" >&2
          return 1
        fi
        source "$_LTMANG_SIM_DIR/ros2_ws/install/setup.bash"
        export ROS_DOMAIN_ID=31
        export ROS_LOCALHOST_ONLY=1
        export LTMANG_SIM_ROOT="$_LTMANG_SIM_DIR"
        export GAZEBO_MODEL_PATH="$_LTMANG_SIM_DIR/models:$_LTMANG_SIM_DIR/ros2_ws/src${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
        _LTMANG_PLUGIN_LIB="$(ros2 pkg prefix mecanum_visual_animator)/lib"
        export GAZEBO_PLUGIN_PATH="$_LTMANG_PLUGIN_LIB${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
        unset _LTMANG_PLUGIN_LIB _LTMANG_SIM_DIR
    ''')


def runner(which: str) -> str:
    if which == 'gazebo':
        command = 'ros2 launch "$LTMANG_SIM_ROOT/launch/mecanum_gazebo_corridor_repo.launch.py" use_relay:=true'
    elif which == 'localization':
        command = 'ros2 launch nav2_bringup localization_launch.py use_sim_time:=true autostart:=true \\\n            map:="$LTMANG_SIM_ROOT/maps/my_map.yaml" \\\n            params_file:="$LTMANG_SIM_ROOT/config/nav2_params_mecanum_sim.yaml"'
    elif which == 'navigation':
        command = 'ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true autostart:=true \\\n            params_file:="$LTMANG_SIM_ROOT/config/nav2_params_mecanum_sim.yaml"'
    elif which == 'rviz':
        command = 'ros2 launch nav2_bringup rviz_launch.py use_sim_time:=true'
    else:
        raise ValueError(which)
    return textwrap.dedent('''\
        #!/usr/bin/env bash
        set -euo pipefail
        _here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
        source "$_here/sim_env.sh"
        ''') + command + '\n'


def documentation() -> str:
    return textwrap.dedent('''\
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
        python3 rssi_sim.py --ros-args -p use_sim_time:=true \\
            -p ap_x:=-3.0 -p ap_y:=-1.0 -p csv:=/tmp/ltmang_rssi.csv
        python3 analyze_rssi.py /tmp/ltmang_rssi.csv --ap -3.0 -1.0 \\
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
    ''')


def add_root_readme(readme: Path) -> bool:
    marker = '## Mô phỏng robot Mecanum mới (Gazebo Classic + Nav2)'
    current = readme.read_text(encoding='utf-8')
    if marker in current:
        return False
    section = ('\n\n' + marker + '\n\n'
        'Đã bổ sung snapshot robot Gazebo/RViz2, plugin bánh Mecanum, '
        'LiDAR/IMU, world hành lang, bản đồ `my_map`, cấu hình Nav2 và script '
        'khởi chạy ở [`simulation/README.md`](simulation/README.md). '
        'Giữ nguyên pipeline RSSI và các kết quả `result_te1n/`. '
        'Thành phần mô phỏng chạy riêng trên PC (`ROS_DOMAIN_ID=31`); '
        'không sửa chương trình Mini PC robot thật.\n')
    readme.write_text(current.rstrip() + section, encoding='utf-8')
    return True


def manifest(sim: Path) -> None:
    paths = sorted(p for p in sim.rglob('*') if p.is_file() and p.name != 'MANIFEST.sha256'
                   and not any(x in p.parts for x in EXCLUDE_DIRS))
    lines = []
    for path in paths:
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        lines.append(f'{digest}  {path.relative_to(sim)}')
    (sim / 'MANIFEST.sha256').write_text('\n'.join(lines) + '\n', encoding='utf-8')


def make_package_if_missing(pkg: Path) -> None:
    if not (pkg / 'package.xml').exists():
        (pkg / 'package.xml').write_text(textwrap.dedent('''\
            <?xml version="1.0"?>
            <package format="3">
              <name>mecanum_robot_viz</name><version>0.1.0</version>
              <description>Gazebo and RViz visualization meshes and URDF for Mecanum simulation.</description>
              <maintainer email="noreply@localhost">Local simulation snapshot</maintainer>
              <license>Unspecified</license><buildtool_depend>ament_cmake</buildtool_depend>
              <exec_depend>robot_state_publisher</exec_depend>
              <exec_depend>gazebo_ros</exec_depend>
              <export><build_type>ament_cmake</build_type></export>
            </package>
        '''), encoding='utf-8')
    cmake = pkg / 'CMakeLists.txt'
    if not cmake.exists():
        dirs = [d for d in ('urdf', 'meshes', 'launch') if (pkg / d).is_dir()]
        cmake.write_text('cmake_minimum_required(VERSION 3.8)\nproject(mecanum_robot_viz)\n'
                         'find_package(ament_cmake REQUIRED)\n'
                         f'install(DIRECTORY {" ".join(dirs)} DESTINATION share/${{PROJECT_NAME}})\n'
                         'ament_package()\n', encoding='utf-8')
    elif not re.search(r'install\s*\(\s*DIRECTORY\s', cmake.read_text(), re.I):
        data = cmake.read_text()
        dirs = [d for d in ('urdf', 'meshes', 'launch') if (pkg / d).is_dir()]
        if dirs and re.search(r'ament_package\s*\(', data):
            data = re.sub(r'ament_package\s*\(',
                          f'install(DIRECTORY {" ".join(dirs)} DESTINATION share/${{PROJECT_NAME}})\nament_package(',
                          data, count=1)
            cmake.write_text(data)


def run():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--repo', type=Path, default=None, help='Local checkout ~/LTMANG or ~/LTMang')
    ap.add_argument('--apply', action='store_true', help='Write snapshot into the local repository')
    ap.add_argument('--push', action='store_true', help='Commit and push scoped files on branch main (requires --apply)')
    args = ap.parse_args()
    if args.push and not args.apply:
        ap.error('--push requires --apply')
    repo = args.repo.expanduser().resolve() if args.repo else next(
        (p for p in (HOME / 'LTMANG', HOME / 'LTMang') if (p / '.git').exists()), HOME / 'LTMANG')
    ws = HOME / 'mecanum_viz_ws/src'
    robot = ws / 'mecanum_robot_viz'
    plugin = ws / 'mecanum_visual_animator'
    required = {
        'Local Git checkout': repo / '.git',
        'Original project README': repo / 'README.md',
        'Robot source package': robot,
        'Gazebo URDF': robot / 'urdf/mecanum_robot_gazebo_real_visual.urdf',
        'RViz URDF': robot / 'urdf/mecanum_robot_rviz_real_visual.urdf',
        'Working corridor launch': robot / 'launch/mecanum_gazebo_corridor_new.launch.py',
        'Gazebo visual wheel plugin source': plugin / 'src',
        'Plugin package.xml': plugin / 'package.xml',
        'Gazebo world': HOME / 'maps/my_map_corridor.world',
        'Map YAML': HOME / 'maps/my_map.yaml',
        'Nav2 sim parameters': HOME / 'nav2_params_mecanum_sim.yaml',
        'ROS safe command relay': HOME / 'mecanum_safe_cmd_relay.py',
    }
    missing = [f'{name}: {path}' for name, path in required.items() if not path.exists()]
    if missing:
        print('ERROR: Missing required local files:\n' + '\n'.join('  - ' + p for p in missing), file=sys.stderr)
        print('These are on your PC, not accessible to ChatGPT; correct paths before retrying.', file=sys.stderr)
        return 2
    image = find_map_image(HOME / 'maps/my_map.yaml')
    print('Repository:', repo)
    print('Source ROS2:', ws)
    print('World:   ', required['Gazebo world'])
    print('Map:     ', required['Map YAML'], 'image:', image)
    print('Robot:   ', robot)
    print('Plugin:  ', plugin)
    print('Nav2:    ', required['Nav2 sim parameters'])
    print('Relay:   ', required['ROS safe command relay'])
    print('Target:  ', repo / 'simulation')
    if not args.apply:
        print('\nDRY RUN ONLY. Use --apply to copy; add --push to commit/push.')
        return 0

    # Prevent accidentally publishing somebody else's repository or existing staged work.
    try:
        remote = sh('git', 'remote', 'get-url', 'origin', cwd=repo).stdout.strip()
        branch = sh('git', 'branch', '--show-current', cwd=repo).stdout.strip()
        dirty_tracked = sh('git', 'status', '--porcelain', '--untracked-files=no', cwd=repo).stdout.strip()
        staged = sh('git', 'diff', '--cached', '--name-only', cwd=repo).stdout.strip()
    except subprocess.CalledProcessError as exc:
        raise RuntimeError('Not a valid Git checkout; check ~/LTMANG/.git') from exc
    remote_normalized = remote.lower().removesuffix('.git').removesuffix('/')
    if not remote_normalized.endswith('github.com/dungphung27/ltmang') and not remote_normalized.endswith('github.com:dungphung27/ltmang'):
        raise RuntimeError(f'Unexpected Git remote origin: {remote!r}')
    if branch != 'main':
        raise RuntimeError(f'Expected main branch, found {branch!r}. Do not push on unknown branch.')
    if dirty_tracked or staged:
        raise RuntimeError('Tracked changes/staged files exist. Commit/stash them first to keep this commit isolated.\n'
                           + dirty_tracked[:2000])

    sim = repo / 'simulation'
    pkgroot = sim / 'ros2_ws/src'
    dstrobot = pkgroot / 'mecanum_robot_viz'
    dstplugin = pkgroot / 'mecanum_visual_animator'
    cptree(robot, dstrobot)
    cptree(plugin, dstplugin)
    make_package_if_missing(dstrobot)
    (sim / 'maps').mkdir(parents=True, exist_ok=True)
    (sim / 'config').mkdir(parents=True, exist_ok=True)
    (sim / 'launch').mkdir(parents=True, exist_ok=True)
    (sim / 'scripts').mkdir(parents=True, exist_ok=True)
    copyfile(HOME / 'maps/my_map_corridor.world', sim / 'maps/my_map_corridor.world')
    world_missing = include_local_world_models(HOME / 'maps/my_map_corridor.world', sim)
    for name in world_missing:
        print('WARNING: world references a model not found locally:', name)
    copyfile(HOME / 'maps/my_map.yaml', sim / 'maps/my_map.yaml')
    copyfile(image, sim / 'maps' / image.name)
    set_map_image_local(sim / 'maps/my_map.yaml', image)
    copyfile(HOME / 'nav2_params_mecanum_sim.yaml', sim / 'config/nav2_params_mecanum_sim.yaml')
    copyfile(HOME / 'mecanum_safe_cmd_relay.py', sim / 'scripts/mecanum_safe_cmd_relay.py')
    custom_bt = HOME / 'nav2_recovery_backup_first.xml'
    if custom_bt.is_file():
        copyfile(custom_bt, sim / 'config/nav2_recovery_backup_first.xml')
    else:
        print('WARNING: custom Nav2 BT XML not found, may be required:', custom_bt)
    old_env = HOME / '.mecanum_sim_env.sh'
    if old_env.is_file():
        copyfile(old_env, sim / 'config/original_pc_sim_env.sh')
    other_packages = copy_related_dwb_packages([ws, HOME / 'ros2_ws/src'], pkgroot)
    if other_packages:
        print('Copied custom DWB plugin packages:')
        for pkg in other_packages:
            print(' ', pkg)
    else:
        print('WARNING: No PreferForwardCritic source found under PC ROS src roots.')
        print('         Check controller_server plugin availability in Nav2 runtime.')

    (sim / 'launch/mecanum_gazebo_corridor_repo.launch.py').write_text(portable_launch(), encoding='utf-8')
    (sim / 'scripts/sim_env.sh').write_text(environment_script(), encoding='utf-8')
    for which in ('gazebo', 'localization', 'navigation', 'rviz'):
        f = sim / f'scripts/run_{which}.sh'
        f.write_text(runner(which), encoding='utf-8')
        f.chmod(0o755)
    (sim / 'README.md').write_text(documentation(), encoding='utf-8')
    (sim / '.gitignore').write_text('__pycache__/\n*.pyc\nros2_ws/build/\nros2_ws/install/\nros2_ws/log/\n*.db3\n*.mcap\n', encoding='utf-8')
    (sim / 'scripts/sim_env.sh').chmod(0o755)
    add_root_readme(repo / 'README.md')
    script_copy = repo / 'scripts/sync_ltmang_robot_sim.py'
    copyfile(Path(__file__).resolve(), script_copy)

    urdfs = [dstrobot / 'urdf/mecanum_robot_gazebo_real_visual.urdf',
             dstrobot / 'urdf/mecanum_robot_rviz_real_visual.urdf']
    errs = urdf_mesh_check(dstrobot, urdfs)
    ET.parse(sim / 'maps/my_map_corridor.world')
    if not (sim / 'maps' / image.name).exists():
        errs.append('missing map image after copy')
    for path in (sim / 'launch/mecanum_gazebo_corridor_repo.launch.py',
                 dstrobot / 'launch/mecanum_gazebo_corridor_new.launch.py',
                 sim / 'scripts/mecanum_safe_cmd_relay.py'):
        compile(path.read_text(encoding='utf-8'), str(path), 'exec')
    try:
        import yaml
        parsed = yaml.safe_load((sim / 'config/nav2_params_mecanum_sim.yaml').read_text(encoding='utf-8'))
        footprint = parsed['local_costmap']['local_costmap']['ros__parameters']['footprint']
        footprint2 = parsed['global_costmap']['global_costmap']['ros__parameters']['footprint']
        if footprint != footprint2:
            errs.append('Nav2 local/global footprints differ')
        if parsed['map_server']['ros__parameters'].get('yaml_filename') != str(HOME / 'maps/my_map.yaml'):
            print('NOTE: YAML map_server path differs from original home map; launch map argument overrides it.')
        if parsed['amcl']['ros__parameters']['initial_pose']['x'] != 5.0:
            print('NOTE: AMCL initial X != launch spawn X=5.0; confirm map/world alignment.')
        print('Nav2 footprint:', footprint)
    except ImportError:
        print('NOTE: PyYAML absent, skipped YAML semantic parse; Nav2 will still parse YAML itself.')
    manifest(sim)
    # Run before any push and do not publish invalid packages.
    if errs:
        raise RuntimeError('Validation failed; no Git commit/push. Fix local sources:\n  - ' + '\n  - '.join(errs))
    print('Copied and verified snapshot. SHA256 manifest:', sim / 'MANIFEST.sha256')
    print('\nGit preview:')
    print(sh('git', 'status', '--short', cwd=repo).stdout)
    if not args.push:
        print('No commit/push requested. Review with git diff; then stage, commit and push manually.')
        print('To use automatic push, start on a clean checkout with --apply --push.')
        return 0
    # No force push; stage ONLY the new simulation, sync tool and README.
    sh('git', 'add', '--', 'simulation', 'scripts/sync_ltmang_robot_sim.py', 'README.md', cwd=repo)
    staged_paths = sh('git', 'diff', '--cached', '--name-only', cwd=repo).stdout.strip()
    if not staged_paths:
        print('Nothing new to commit.')
        return 0
    too_large = []
    unsafe = []
    for relative in staged_paths.splitlines():
        p = repo / relative
        parts = p.parts
        if any(part in EXCLUDE_NAMES for part in parts):
            unsafe.append(relative)
        if p.is_file() and p.stat().st_size > 90 * 1024 * 1024:
            too_large.append(f'{relative} ({p.stat().st_size / (1024**2):.1f} MiB)')
    if too_large or unsafe:
        sh('git', 'reset', '-q', '--', 'simulation', 'scripts/sync_ltmang_robot_sim.py', 'README.md', cwd=repo)
        raise RuntimeError('Unsafe files excluded from push: ' + ', '.join(unsafe + too_large))
    sh('git', 'commit', '-m', 'Add complete Gazebo Mecanum simulation and Nav2 assets', cwd=repo)
    print('Commit created:', sh('git', 'rev-parse', '--short', 'HEAD', cwd=repo).stdout.strip())
    # Fail safely if repo has diverged; no --force or history rewriting.
    p = sh('git', 'push', 'origin', 'main', cwd=repo, check=False)
    if p.returncode:
        print('PUSH FAILED; local commit is preserved. Error:\n' + p.stderr)
        print('Resolve Git credentials/upstream, then: cd ~/LTMANG && git push origin main')
        return 3
    print('PUSH OK: ' + remote + ' (main)')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(run())
    except (OSError, RuntimeError, ET.ParseError, subprocess.CalledProcessError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        sys.exit(1)
