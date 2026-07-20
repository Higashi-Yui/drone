#!/usr/bin/env bash
# 基于 https://gitee.com/edl-diana/multi 的 Ubuntu 20.04 + ROS Noetic
# MID360/MID360S + Point-LIO + MAVROS + bridge 自动配置脚本。
#
# 默认行为：
#   1. 检查 Ubuntu 20.04 / ROS Noetic
#   2. 安装 MAVROS、catkin-tools、Point-LIO 等依赖
#   3. 克隆 multi 仓库（已存在则保留，不强制 pull）
#   4. 编译 Livox-SDK2、livox_ros_driver2、Point-LIO、bridge
#   5. 修改 MID360(S) 配置文件中的主机 IP 和雷达 IP
#   6. 写入 ~/.bashrc 环境加载块
#   7. 生成基于 tmux、按话题就绪状态依次启动的 start_uav_stack.sh
#
# 注意：本脚本不会自动写 PX4 EKF 参数。飞控参数与固件、定位方案及外参有关，
# 请在 QGroundControl 中人工核对后设置。

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_NAME="$(basename "$0")"
ROS_DISTRO="noetic"
REPO_URL="https://gitee.com/edl-diana/multi.git"
REPO_DIR="${HOME}/multi"
HOST_IP="192.168.1.50"
LIDAR_IP=""
SN_SUFFIX=""
LIDAR_MODEL="mid360s"
NETWORK_IFACE=""
NETWORK_CONNECTION=""
FCU_URL="/dev/ttyACM0:921600"
CONFIGURE_NETWORK=0
SKIP_APT=0
SKIP_CLONE=0
CLEAN_BUILD=0
NON_INTERACTIVE=0
ALLOW_UNSUPPORTED=0
SKIP_ROSDEP=0

REBOOT_REQUIRED=0
LOG_FILE="${HOME}/multi_uav_setup_$(date +%Y%m%d_%H%M%S).log"

usage() {
  cat <<EOF
用法：
  ./${SCRIPT_NAME} [选项]

常用示例：
  ./${SCRIPT_NAME} --lidar-ip 192.168.1.181 \\
    --interface enp3s0 --configure-network

  ./${SCRIPT_NAME} --sn-suffix 81 --interface enp3s0 --configure-network

选项：
  --repo-dir DIR            仓库目录，默认：${REPO_DIR}
  --repo-url URL            仓库地址，默认：${REPO_URL}
  --host-ip IP              上位机静态 IP，默认：${HOST_IP}
  --lidar-ip IP             雷达完整 IP，例如 192.168.1.181
  --sn-suffix XX            雷达 SN/广播码最后两位，用于生成 192.168.1.1XX
  --lidar-model MODEL       mid360s 或 mid360，默认：${LIDAR_MODEL}
  --interface IFACE         连接雷达的有线网卡，例如 enp3s0
  --connection NAME         NetworkManager 连接名，默认自动生成
  --configure-network       自动创建/更新雷达专用静态 IP 连接
  --fcu-url URL             MAVROS FCU URL，默认：${FCU_URL}
  --skip-apt                不安装 apt 依赖
  --skip-clone              不克隆仓库，要求 --repo-dir 已存在
  --skip-rosdep             不执行 rosdep/rosdepc
  --clean                   删除三个工作空间的 build/devel/logs 后重编
  --non-interactive         非交互模式；缺少必要参数时直接报错
  --allow-unsupported       非 Ubuntu 20.04 或非 Noetic 时仅警告
  -h, --help                显示帮助
EOF
}

color() {
  local code="$1"; shift
  printf '\033[%sm%s\033[0m\n' "$code" "$*"
}

info()  { color '1;34' "[INFO] $*" | tee -a "$LOG_FILE"; }
warn()  { color '1;33' "[WARN] $*" | tee -a "$LOG_FILE" >&2; }
error() { color '1;31' "[ERROR] $*" | tee -a "$LOG_FILE" >&2; }
die()   { error "$*"; exit 1; }

run() {
  info "+ $*"
  "$@" 2>&1 | tee -a "$LOG_FILE"
}

on_error() {
  local exit_code=$?
  local line_no=${1:-unknown}
  error "脚本在第 ${line_no} 行失败，退出码：${exit_code}"
  error "完整日志：${LOG_FILE}"
  exit "$exit_code"
}
trap 'on_error $LINENO' ERR

while (($#)); do
  case "$1" in
    --repo-dir) REPO_DIR="${2:?缺少目录}"; shift 2 ;;
    --repo-url) REPO_URL="${2:?缺少 URL}"; shift 2 ;;
    --host-ip) HOST_IP="${2:?缺少 IP}"; shift 2 ;;
    --lidar-ip) LIDAR_IP="${2:?缺少 IP}"; shift 2 ;;
    --sn-suffix) SN_SUFFIX="${2:?缺少后两位}"; shift 2 ;;
    --lidar-model) LIDAR_MODEL="${2:?缺少型号}"; shift 2 ;;
    --interface) NETWORK_IFACE="${2:?缺少网卡名}"; shift 2 ;;
    --connection) NETWORK_CONNECTION="${2:?缺少连接名}"; shift 2 ;;
    --configure-network) CONFIGURE_NETWORK=1; shift ;;
    --fcu-url) FCU_URL="${2:?缺少 FCU URL}"; shift 2 ;;
    --skip-apt) SKIP_APT=1; shift ;;
    --skip-clone) SKIP_CLONE=1; shift ;;
    --skip-rosdep) SKIP_ROSDEP=1; shift ;;
    --clean) CLEAN_BUILD=1; shift ;;
    --non-interactive) NON_INTERACTIVE=1; shift ;;
    --allow-unsupported) ALLOW_UNSUPPORTED=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) usage; die "未知选项：$1" ;;
  esac
done

[[ "$LIDAR_MODEL" == "mid360s" || "$LIDAR_MODEL" == "mid360" ]] \
  || die "--lidar-model 只能是 mid360s 或 mid360"

validate_ip() {
  python3 - "$1" <<'PY'
import ipaddress, sys
try:
    ipaddress.ip_address(sys.argv[1])
except ValueError:
    raise SystemExit(1)
PY
}

resolve_lidar_ip() {
  if [[ -n "$LIDAR_IP" ]]; then
    validate_ip "$LIDAR_IP" || die "雷达 IP 格式无效：$LIDAR_IP"
    return
  fi

  if [[ -n "$SN_SUFFIX" ]]; then
    [[ "$SN_SUFFIX" =~ ^[0-9]{1,2}$ ]] || die "SN 后两位必须为 1~2 位数字"
    local suffix_num=$((10#$SN_SUFFIX))
    (( suffix_num <= 99 )) || die "SN 后两位范围应为 00~99"
    LIDAR_IP="192.168.1.$((100 + suffix_num))"
    return
  fi

  if (( NON_INTERACTIVE )); then
    die "非交互模式下必须提供 --lidar-ip 或 --sn-suffix"
  fi

  read -r -p "请输入 MID360(S) 的完整 IP（例如 192.168.1.181）：" LIDAR_IP
  validate_ip "$LIDAR_IP" || die "雷达 IP 格式无效：$LIDAR_IP"
}

select_network_interface() {
  [[ -n "$NETWORK_IFACE" ]] && return
  (( CONFIGURE_NETWORK )) || return

  mapfile -t ethernet_ifaces < <(
    find /sys/class/net -mindepth 1 -maxdepth 1 -printf '%f\n' \
      | grep -Ev '^(lo|docker|veth|virbr|br-|tun|tap|wg)' \
      | while read -r dev; do
          [[ -e "/sys/class/net/${dev}/type" ]] || continue
          [[ "$(cat "/sys/class/net/${dev}/type")" == "1" ]] && echo "$dev"
        done
  )

  ((${#ethernet_ifaces[@]} > 0)) || die "未发现可用的以太网接口"

  if ((${#ethernet_ifaces[@]} == 1)); then
    NETWORK_IFACE="${ethernet_ifaces[0]}"
    info "自动选择有线网卡：${NETWORK_IFACE}"
    return
  fi

  if (( NON_INTERACTIVE )); then
    die "检测到多个网卡，请使用 --interface 指定：${ethernet_ifaces[*]}"
  fi

  echo "检测到以下网卡："
  local i
  for i in "${!ethernet_ifaces[@]}"; do
    printf '  %d) %s\n' "$((i + 1))" "${ethernet_ifaces[$i]}"
  done
  local choice
  read -r -p "请选择连接雷达的网卡编号：" choice
  [[ "$choice" =~ ^[0-9]+$ ]] || die "输入不是有效编号"
  (( choice >= 1 && choice <= ${#ethernet_ifaces[@]} )) || die "编号超出范围"
  NETWORK_IFACE="${ethernet_ifaces[$((choice - 1))]}"
}

check_platform() {
  [[ $EUID -ne 0 ]] || die "请使用普通用户运行，不要直接 sudo 执行整个脚本"

  local ubuntu_version="unknown"
  if command -v lsb_release >/dev/null 2>&1; then
    ubuntu_version="$(lsb_release -rs)"
  elif [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    source /etc/os-release
    ubuntu_version="${VERSION_ID:-unknown}"
  fi

  if [[ "$ubuntu_version" != "20.04" ]]; then
    if (( ALLOW_UNSUPPORTED )); then
      warn "当前系统为 Ubuntu ${ubuntu_version}，仓库教程目标环境为 Ubuntu 20.04"
    else
      die "当前系统为 Ubuntu ${ubuntu_version}。如确认继续，请添加 --allow-unsupported"
    fi
  fi

  if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    if (( ALLOW_UNSUPPORTED )); then
      warn "未发现 /opt/ros/${ROS_DISTRO}/setup.bash，后续编译可能失败"
    else
      cat >&2 <<'EOF'
未检测到 ROS Noetic。
请先安装 ROS Noetic；国内环境可使用鱼香 ROS：
  wget http://fishros.com/install -O fishros && . fishros
安装完成并重新打开终端后，再运行本脚本。
EOF
      exit 1
    fi
  fi
}

install_dependencies() {
  (( SKIP_APT )) && { warn "已跳过 apt 依赖安装"; return; }

  run sudo apt-get update
  run sudo apt-get install -y \
    build-essential cmake git tmux jq network-manager iputils-ping \
    python3 python3-pip python3-rosdep python3-catkin-tools \
    libgoogle-glog-dev libgflags-dev libeigen3-dev libpcl-dev \
    "ros-${ROS_DISTRO}-mavros" \
    "ros-${ROS_DISTRO}-mavros-extras" \
    "ros-${ROS_DISTRO}-pcl-ros" \
    "ros-${ROS_DISTRO}-tf" \
    "ros-${ROS_DISTRO}-tf2-ros" \
    "ros-${ROS_DISTRO}-eigen-conversions"

  local geo_script="/opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh"
  if [[ -x "$geo_script" ]]; then
    run sudo "$geo_script"
  else
    warn "未找到 GeographicLib 数据安装脚本：${geo_script}"
  fi

  if ! id -nG "$USER" | tr ' ' '\n' | grep -qx dialout; then
    run sudo usermod -aG dialout "$USER"
    REBOOT_REQUIRED=1
    warn "已将 ${USER} 加入 dialout 组；需注销登录或重启后生效"
  fi
}

clone_repository() {
  if [[ -d "$REPO_DIR/.git" ]]; then
    info "仓库已存在，保留本地修改且不自动 pull：${REPO_DIR}"
    return
  fi

  if (( SKIP_CLONE )); then
    [[ -d "$REPO_DIR" ]] || die "--skip-clone 已启用，但目录不存在：${REPO_DIR}"
    return
  fi

  [[ ! -e "$REPO_DIR" || -z "$(ls -A "$REPO_DIR" 2>/dev/null || true)" ]] \
    || die "目录存在但不是 Git 仓库且非空：${REPO_DIR}"

  mkdir -p "$(dirname "$REPO_DIR")"
  if ! git clone "$REPO_URL" "$REPO_DIR" 2>&1 | tee -a "$LOG_FILE"; then
    cat >&2 <<EOF
仓库克隆失败。请检查 DNS、网络或代理，然后重试。
可先测试：
  ping -c 2 gitee.com
  git config --global --get http.proxy
仓库地址：${REPO_URL}
EOF
    exit 1
  fi
}

require_repo_layout() {
  local missing=0
  for path in \
    "$REPO_DIR/mid360_ws/src/livox_ros_driver2" \
    "$REPO_DIR/point_ws" \
    "$REPO_DIR/uav_ws"; do
    if [[ ! -e "$path" ]]; then
      error "仓库中缺少：$path"
      missing=1
    fi
  done
  (( missing == 0 )) || die "仓库结构与教程不一致，请确认分支或仓库内容"
}

configure_network() {
  (( CONFIGURE_NETWORK )) || { info "未修改系统网卡；请确保 ${HOST_IP}/24 已配置到雷达网口"; return; }
  command -v nmcli >/dev/null 2>&1 || die "未找到 nmcli，请安装 network-manager"
  [[ -d "/sys/class/net/${NETWORK_IFACE}" ]] || die "网卡不存在：${NETWORK_IFACE}"

  NETWORK_CONNECTION="${NETWORK_CONNECTION:-mid360-${NETWORK_IFACE}}"

  if nmcli -t -f NAME connection show | grep -Fxq "$NETWORK_CONNECTION"; then
    run sudo nmcli connection modify "$NETWORK_CONNECTION" \
      connection.interface-name "$NETWORK_IFACE" \
      ipv4.method manual \
      ipv4.addresses "${HOST_IP}/24" \
      ipv4.gateway "" \
      ipv4.dns "" \
      ipv4.never-default yes \
      ipv6.method disabled \
      connection.autoconnect yes
  else
    run sudo nmcli connection add type ethernet \
      ifname "$NETWORK_IFACE" \
      con-name "$NETWORK_CONNECTION" \
      ipv4.method manual \
      ipv4.addresses "${HOST_IP}/24" \
      ipv4.never-default yes \
      ipv6.method disabled \
      connection.autoconnect yes
  fi

  run sudo nmcli connection up "$NETWORK_CONNECTION"
  info "雷达专用网络已配置：${NETWORK_IFACE} -> ${HOST_IP}/24"
}

find_sdk_dir() {
  local sdk_dir=""
  sdk_dir="$(find "$REPO_DIR/mid360s_sdk" -type d -name Livox_SDK2 -print -quit 2>/dev/null || true)"
  [[ -n "$sdk_dir" ]] || sdk_dir="$(find "$REPO_DIR" -type d -name Livox_SDK2 -print -quit 2>/dev/null || true)"
  [[ -n "$sdk_dir" ]] || die "未在仓库内找到 Livox_SDK2 目录"
  printf '%s\n' "$sdk_dir"
}

clean_workspace_if_requested() {
  local ws="$1"
  (( CLEAN_BUILD )) || return
  info "清理工作空间：${ws}"
  rm -rf "$ws/build" "$ws/devel" "$ws/logs" "$ws/.catkin_tools"
}

source_setup() {
  local file="$1"
  [[ -f "$file" ]] || die "环境文件不存在：$file"
  set +u
  # shellcheck disable=SC1090
  source "$file"
  set -u
}

install_ros_dependencies() {
  (( SKIP_ROSDEP )) && { warn "已跳过 rosdep/rosdepc"; return; }

  source_setup "/opt/ros/${ROS_DISTRO}/setup.bash"
  local dep_cmd=""
  if command -v rosdepc >/dev/null 2>&1; then
    dep_cmd="rosdepc"
  elif command -v rosdep >/dev/null 2>&1; then
    dep_cmd="rosdep"
  else
    warn "未找到 rosdep 或 rosdepc，跳过依赖解析"
    return
  fi

  local ws
  for ws in "$REPO_DIR/mid360_ws" "$REPO_DIR/point_ws" "$REPO_DIR/uav_ws"; do
    [[ -d "$ws/src" ]] || continue
    info "解析 ROS 依赖：${ws}"
    if ! "$dep_cmd" install --from-paths "$ws/src" --ignore-src -r -y 2>&1 | tee -a "$LOG_FILE"; then
      warn "${dep_cmd} 在 ${ws} 中存在未解析项；继续尝试编译"
    fi
  done
}

build_livox_sdk() {
  local sdk_dir
  sdk_dir="$(find_sdk_dir)"
  local build_dir="${sdk_dir}/build"

  (( CLEAN_BUILD )) && rm -rf "$build_dir"
  run cmake -S "$sdk_dir" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release
  run cmake --build "$build_dir" --parallel "$(nproc)"
  run sudo cmake --install "$build_dir"
  run sudo ldconfig
}

configure_lidar_json() {
  local driver_dir="$REPO_DIR/mid360_ws/src/livox_ros_driver2"
  local config_file=""

  if [[ "$LIDAR_MODEL" == "mid360s" ]]; then
    config_file="$driver_dir/config/MID360s_config.json"
  else
    config_file="$driver_dir/config/MID360_config.json"
  fi

  [[ -f "$config_file" ]] || die "找不到雷达配置文件：${config_file}"

  local backup="${config_file}.bak.$(date +%Y%m%d_%H%M%S)"
  cp -a "$config_file" "$backup"
  info "已备份雷达配置：${backup}"

  python3 - "$config_file" "$HOST_IP" "$LIDAR_IP" <<'PY'
import json
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
host_ip = sys.argv[2]
lidar_ip = sys.argv[3]

data = json.loads(path.read_text(encoding="utf-8"))
changed = []

# MID360 旧格式：host_net_info 是字典，含 cmd_data_ip 等字段。
# MID360S 新格式：host_net_info 是列表，含 host_ip 字段。
for model_key in ("MID360", "Mid360s", "MID360S", "mid360s"):
    model = data.get(model_key)
    if not isinstance(model, dict):
        continue
    host_info = model.get("host_net_info")
    entries = host_info if isinstance(host_info, list) else [host_info]
    for entry in entries:
        if not isinstance(entry, dict):
            continue
        if "host_ip" in entry:
            entry["host_ip"] = host_ip
            changed.append(f"{model_key}.host_net_info.host_ip")
        for key in ("cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip"):
            if key in entry:
                entry[key] = host_ip
                changed.append(f"{model_key}.host_net_info.{key}")

lidar_configs = data.get("lidar_configs")
if not isinstance(lidar_configs, list) or not lidar_configs:
    raise SystemExit("配置文件中未找到 lidar_configs")

for index, item in enumerate(lidar_configs):
    if isinstance(item, dict) and "ip" in item:
        item["ip"] = lidar_ip
        changed.append(f"lidar_configs[{index}].ip")

if not changed:
    raise SystemExit("没有识别到可修改的主机 IP 字段")

path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
print("已修改：")
for key in changed:
    print(f"  - {key}")
PY

  python3 -m json.tool "$config_file" >/dev/null
  info "雷达配置完成：主机 ${HOST_IP}，雷达 ${LIDAR_IP}"
}

build_livox_driver() {
  local mid_ws="$REPO_DIR/mid360_ws"
  local driver_dir="$mid_ws/src/livox_ros_driver2"
  clean_workspace_if_requested "$mid_ws"
  source_setup "/opt/ros/${ROS_DISTRO}/setup.bash"

  [[ -f "$driver_dir/build.sh" ]] || die "找不到：${driver_dir}/build.sh"
  run chmod +x "$driver_dir/build.sh"
  (
    cd "$driver_dir"
    run ./build.sh ROS1
  )

  [[ -f "$mid_ws/devel/setup.bash" ]] || die "livox_ros_driver2 编译后未生成 ${mid_ws}/devel/setup.bash"
}

catkin_build_workspace() {
  local ws="$1"
  local extend_path="$2"
  shift 2
  local packages=("$@")

  clean_workspace_if_requested "$ws"
  source_setup "/opt/ros/${ROS_DISTRO}/setup.bash"
  [[ -f "$extend_path" ]] && source_setup "$extend_path"

  mkdir -p "$ws/src"
  (
    cd "$ws"
    if [[ ! -d .catkin_tools ]]; then
      run catkin init
    fi
    if [[ -f "$extend_path" ]]; then
      run catkin config --merge-devel --extend "$(dirname "$extend_path")" \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
    else
      run catkin config --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release
    fi

    if ((${#packages[@]} > 0)); then
      run catkin build "${packages[@]}" --no-status --summarize
    else
      run catkin build --no-status --summarize
    fi
  )

  [[ -f "$ws/devel/setup.bash" ]] || die "编译后未生成：${ws}/devel/setup.bash"
}

build_workspaces() {
  local mid_setup="$REPO_DIR/mid360_ws/devel/setup.bash"
  local point_setup="$REPO_DIR/point_ws/devel/setup.bash"

  catkin_build_workspace "$REPO_DIR/point_ws" "$mid_setup"
  catkin_build_workspace "$REPO_DIR/uav_ws" "$point_setup" bridge
}

write_bashrc_block() {
  local bashrc="${HOME}/.bashrc"
  local start_marker="# >>> multi UAV stack >>>"
  local end_marker="# <<< multi UAV stack <<<"

  python3 - "$bashrc" "$start_marker" "$end_marker" "$REPO_DIR" "$ROS_DISTRO" <<'PY'
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
start, end, repo, ros = sys.argv[2:]
text = path.read_text(encoding="utf-8") if path.exists() else ""

block = f'''{start}
source /opt/ros/{ros}/setup.bash
[[ -f "{repo}/mid360_ws/devel/setup.bash" ]] && source "{repo}/mid360_ws/devel/setup.bash"
[[ -f "{repo}/point_ws/devel/setup.bash" ]] && source "{repo}/point_ws/devel/setup.bash"
[[ -f "{repo}/uav_ws/devel/setup.bash" ]] && source "{repo}/uav_ws/devel/setup.bash"
export LD_LIBRARY_PATH="/usr/local/lib:${{LD_LIBRARY_PATH:-}}"
{end}'''

if start in text and end in text:
    before, rest = text.split(start, 1)
    _, after = rest.split(end, 1)
    text = before.rstrip() + "\n\n" + block + after
else:
    text = text.rstrip() + "\n\n" + block + "\n"

path.write_text(text, encoding="utf-8")
PY
  info "已更新 ${bashrc}，重复运行不会重复追加"
}

generate_start_script() {
  local output="$REPO_DIR/start_uav_stack.sh"
  cat > "$output" <<EOF
#!/usr/bin/env bash
set -Eeuo pipefail

SESSION="\${SESSION:-uav_core}"
FCU_URL="\${FCU_URL:-${FCU_URL}}"
ROS_DISTRO="${ROS_DISTRO}"
REPO_DIR="${REPO_DIR}"
LIVOX_LAUNCH="\${LIVOX_LAUNCH:-msg_$([[ "$LIDAR_MODEL" == "mid360s" ]] && echo MID360s || echo MID360).launch}"
POINT_LIO_LAUNCH="\${POINT_LIO_LAUNCH:-mapping_mid360.launch}"
BRIDGE_COMMAND="\${BRIDGE_COMMAND:-roslaunch bridge bridge.launch || rosrun bridge bridge_node}"
WAIT_TIMEOUT="\${WAIT_TIMEOUT:-45}"

source_env() {
  set +u
  source "/opt/ros/\${ROS_DISTRO}/setup.bash"
  source "\${REPO_DIR}/mid360_ws/devel/setup.bash"
  source "\${REPO_DIR}/point_ws/devel/setup.bash"
  source "\${REPO_DIR}/uav_ws/devel/setup.bash"
  set -u
  export LD_LIBRARY_PATH="/usr/local/lib:\${LD_LIBRARY_PATH:-}"
}

wait_until() {
  local description="\$1"
  local command="\$2"
  local timeout="\${3:-\$WAIT_TIMEOUT}"
  local start
  start=\$(date +%s)
  printf '[WAIT] %s' "\$description"
  until eval "\$command" >/dev/null 2>&1; do
    if (( \$(date +%s) - start >= timeout )); then
      printf '\n[WARN] 等待超时：%s（继续启动下一项）\n' "\$description" >&2
      return 1
    fi
    printf '.'
    sleep 0.5
  done
  printf ' OK\n'
}

command -v tmux >/dev/null 2>&1 || { echo '缺少 tmux：sudo apt install tmux' >&2; exit 1; }
source_env

if tmux has-session -t "\$SESSION" 2>/dev/null; then
  echo "tmux 会话 \$SESSION 已存在，直接进入。"
  exec tmux attach -t "\$SESSION"
fi

ENV_CMD="source /opt/ros/\${ROS_DISTRO}/setup.bash; source \${REPO_DIR}/mid360_ws/devel/setup.bash; source \${REPO_DIR}/point_ws/devel/setup.bash; source \${REPO_DIR}/uav_ws/devel/setup.bash; export LD_LIBRARY_PATH=/usr/local/lib:\${LD_LIBRARY_PATH:-}"

tmux new-session -d -s "\$SESSION" -n roscore "bash -lc '\$ENV_CMD; roscore; exec bash'"
wait_until 'ROS master' 'rosparam list'

tmux new-window -t "\$SESSION" -n mavros "bash -lc '\$ENV_CMD; roslaunch mavros px4.launch fcu_url:=\"\$FCU_URL\"; exec bash'"
wait_until 'MAVROS 话题' 'rostopic list | grep -qx /mavros/state'
wait_until 'PX4 connected=True' 'rostopic echo -n 1 /mavros/state 2>/dev/null | grep -q "connected: True"' 30 || true

tmux new-window -t "\$SESSION" -n livox "bash -lc '\$ENV_CMD; roslaunch livox_ros_driver2 \$LIVOX_LAUNCH; exec bash'"
wait_until 'Livox 点云话题' 'rostopic list | grep -qx /livox/lidar'

tmux new-window -t "\$SESSION" -n pointlio "bash -lc '\$ENV_CMD; roslaunch point_lio \$POINT_LIO_LAUNCH; exec bash'"
wait_until 'Point-LIO 里程计' 'rostopic list | grep -qx /aft_mapped_to_init'

tmux new-window -t "\$SESSION" -n bridge "bash -lc '\$ENV_CMD; \$BRIDGE_COMMAND; exec bash'"
wait_until '视觉位姿话题' 'rostopic list | grep -qx /mavros/vision_pose/pose' 20 || true

tmux select-window -t "\$SESSION:bridge"
exec tmux attach -t "\$SESSION"
EOF
  chmod +x "$output"
  info "已生成启动脚本：${output}"
}

write_config_summary() {
  local output="$REPO_DIR/multi_uav_config.env"
  cat > "$output" <<EOF
# 由 ${SCRIPT_NAME} 于 $(date --iso-8601=seconds) 生成
REPO_DIR=${REPO_DIR}
ROS_DISTRO=${ROS_DISTRO}
HOST_IP=${HOST_IP}
LIDAR_IP=${LIDAR_IP}
LIDAR_MODEL=${LIDAR_MODEL}
NETWORK_IFACE=${NETWORK_IFACE}
NETWORK_CONNECTION=${NETWORK_CONNECTION}
FCU_URL=${FCU_URL}
EOF
  info "配置摘要：${output}"
}

post_checks() {
  info "执行安装后检查"

  if (( CONFIGURE_NETWORK )); then
    ip -4 addr show dev "$NETWORK_IFACE" | tee -a "$LOG_FILE" || true
    if ping -c 2 -W 1 "$LIDAR_IP" >/dev/null 2>&1; then
      info "雷达网络连通：${LIDAR_IP}"
    else
      warn "暂时无法 ping 通 ${LIDAR_IP}。请检查雷达供电、网线、实际 IP 和 SN 后两位"
    fi
  fi

  [[ -f "$REPO_DIR/mid360_ws/devel/setup.bash" ]] || warn "MID360 工作空间环境文件缺失"
  [[ -f "$REPO_DIR/point_ws/devel/setup.bash" ]] || warn "Point-LIO 工作空间环境文件缺失"
  [[ -f "$REPO_DIR/uav_ws/devel/setup.bash" ]] || warn "UAV 工作空间环境文件缺失"
}

print_px4_checklist() {
  cat <<EOF

================ PX4 参数人工核对 ================
教程对应的室内 Vision 定位方案通常需要在 QGroundControl 中核对：
  EKF2_EV_CTRL      = 11（按教程的 vins_to_mavros：位置 + 偏航，不融合速度）
  EKF2_HGT_REF      = Vision
  EKF2_EV_POS_X/Y/Z = 雷达相对机体重心的 FRD 外参，单位 m

是否关闭 GPS、磁罗盘和气压计融合必须按你的实机方案决定，
本脚本没有自动修改这些飞控参数。
==================================================
EOF
}

main() {
  touch "$LOG_FILE"
  info "日志文件：${LOG_FILE}"

  validate_ip "$HOST_IP" || die "主机 IP 格式无效：$HOST_IP"
  resolve_lidar_ip
  [[ "$HOST_IP" != "$LIDAR_IP" ]] || die "主机 IP 与雷达 IP 不能相同：$HOST_IP"
  select_network_interface
  check_platform

  info "配置摘要："
  info "  仓库目录：${REPO_DIR}"
  info "  上位机 IP：${HOST_IP}"
  info "  雷达 IP：${LIDAR_IP}"
  info "  雷达型号：${LIDAR_MODEL}"
  [[ -n "$NETWORK_IFACE" ]] && info "  雷达网卡：${NETWORK_IFACE}"

  install_dependencies
  clone_repository
  require_repo_layout
  configure_network
  install_ros_dependencies
  build_livox_sdk
  configure_lidar_json
  build_livox_driver
  build_workspaces
  write_bashrc_block
  generate_start_script
  write_config_summary
  post_checks

  info "自动配置完成"
  echo
  echo "重新加载环境："
  echo "  source ~/.bashrc"
  echo
  echo "启动完整链路："
  echo "  ${REPO_DIR}/start_uav_stack.sh"
  echo
  echo "单独测试 MID360："
  echo "  roslaunch livox_ros_driver2 msg_$([[ "$LIDAR_MODEL" == "mid360s" ]] && echo MID360s || echo MID360).launch"
  echo "  rostopic hz /livox/lidar"
  echo
  echo "完整日志：${LOG_FILE}"

  if (( REBOOT_REQUIRED )); then
    warn "dialout 用户组刚修改，建议现在重启：sudo reboot"
  fi
  print_px4_checklist
}

main "$@"
