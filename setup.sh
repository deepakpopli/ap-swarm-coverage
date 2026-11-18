#!/usr/bin/env bash
# Check the whole stack needed by run_eval.sh and print an OK / MISSING table.
#
#   ./setup.sh            check only; prints the commands that would fix what is missing
#   ./setup.sh --build    also pip-installs the Python packages (--user) and clones + builds
#                         ArduCopter SITL and the ardupilot_gazebo plugin where they are missing
#
# Never runs sudo. System packages (apt) are only printed for you to run.
# Environment: ARDUPILOT_HOME (default ~/ardupilot), ARDUPILOT_GAZEBO (default ~/ardupilot_gazebo),
#              BACKEND (default gazebo: the backend the default evaluation needs).
# Exit code: 0 if everything required for the default evaluation is there, 1 otherwise.
set -u
ROOT="$(cd "$(dirname "$0")" && pwd)"
ARDUPILOT_HOME=${ARDUPILOT_HOME:-$HOME/ardupilot}
ARDUPILOT_GAZEBO=${ARDUPILOT_GAZEBO:-$HOME/ardupilot_gazebo}
BACKEND=${BACKEND:-gazebo}
JOBS=${JOBS:-4}          # build parallelism kept low on purpose (thermal limits)
BUILD=0
[ "${1:-}" = "--build" ] && BUILD=1

ROWS=()        # "status|name|detail"
FIXES=()       # commands to print at the end
missing_required=0

row() {  # $1 OK|MISSING|WARN  $2 name  $3 detail  $4 required (1/0)
  ROWS+=("$1|$2|$3")
  if [ "$1" = MISSING ] && [ "${4:-1}" = 1 ]; then missing_required=1; fi
}
fix() { FIXES+=("$1"); }

# ---------------------------------------------------------------- python
check_python() {
  if ! command -v python3 > /dev/null; then
    row MISSING python3 "not on PATH"
    fix "sudo apt install python3 python3-pip"
    return
  fi
  local v; v=$(python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])')
  if python3 -c 'import sys; sys.exit(sys.version_info < (3, 8))'; then
    row OK python3 "$v"
  else
    row MISSING python3 "$v (need >= 3.8)"
  fi
  local pkgs="pymavlink numpy matplotlib" missing=0 m ver
  if [ "$BUILD" = 1 ] && ! python3 -c "import pymavlink, numpy, matplotlib" 2> /dev/null; then
    echo ">> python3 -m pip install --user -r requirements.txt"
    python3 -m pip install --user -r "$ROOT/requirements.txt"
  fi
  for m in $pkgs; do
    if python3 -c "import $m" 2> /dev/null; then
      ver=$(python3 -c "import importlib.metadata as md; print(md.version('$m'))" 2> /dev/null)
      row OK "$m" "${ver:-installed}"
    else
      row MISSING "$m" "python3 cannot import it"
      missing=1
    fi
  done
  [ "$missing" = 1 ] && fix "python3 -m pip install --user -r $ROOT/requirements.txt"
}

# ---------------------------------------------------------------- tools
check_tools() {
  if command -v ffmpeg > /dev/null; then
    row OK ffmpeg "$(ffmpeg -version 2>/dev/null | head -1 | awk '{print $3}')"
  else
    row MISSING ffmpeg "needed for the animation (analysis.animate)"
    fix "sudo apt install ffmpeg"
  fi
  local t
  for t in git nice setsid awk; do
    if command -v "$t" > /dev/null; then row OK "$t" "$(command -v "$t")"; else row MISSING "$t" "not on PATH"; fi
  done
  if grep -qx k10temp /sys/class/hwmon/hwmon*/name 2>/dev/null; then
    row OK "cpu temp sensor" "k10temp (tools/thermal_guard.sh)"
  else
    row WARN "cpu temp sensor" "no k10temp: thermal_guard falls back to thermal_zone0, wait_cool.sh reads 0 C" 0
  fi
}

# ---------------------------------------------------------------- ArduCopter SITL
ardupilot_cmds() {
  cat <<EOF
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git $ARDUPILOT_HOME
(cd $ARDUPILOT_HOME && Tools/environment_install/install-prereqs-ubuntu.sh -y)   # uses sudo: run it yourself once
(cd $ARDUPILOT_HOME && ./waf configure --board sitl && nice -n 19 ./waf copter -j $JOBS)
EOF
}

build_ardupilot() {
  echo ">> building ArduCopter SITL in $ARDUPILOT_HOME"
  if [ ! -d "$ARDUPILOT_HOME/.git" ]; then
    git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_HOME" || return 1
  fi
  (cd "$ARDUPILOT_HOME" && ./waf configure --board sitl && nice -n 19 ./waf copter -j "$JOBS")
}

check_ardupilot() {
  local bin="$ARDUPILOT_HOME/build/sitl/bin/arducopter"
  if [ ! -x "$bin" ] && [ "$BUILD" = 1 ]; then
    build_ardupilot || echo "!! ArduCopter build failed (prerequisites? see the commands below)"
  fi
  if [ -x "$bin" ]; then
    row OK "arducopter SITL" "$bin"
  else
    row MISSING "arducopter SITL" "$bin"
    while IFS= read -r c; do fix "$c"; done < <(ardupilot_cmds)
  fi
  local p="$ARDUPILOT_HOME/Tools/autotest/default_params"
  if [ -f "$p/copter.parm" ] && [ -f "$p/gazebo-iris.parm" ]; then
    row OK "copter default params" "$p"
  else
    row MISSING "copter default params" "$p/copter.parm, gazebo-iris.parm"
  fi
}

# ---------------------------------------------------------------- Gazebo Harmonic + plugin
gz_apt_cmds() {
  cat <<'EOF'
sudo apt-get install curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt-get update && sudo apt-get install gz-harmonic
EOF
}

plugin_cmds() {
  cat <<EOF
sudo apt install libgz-sim8-dev rapidjson-dev libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
git clone https://github.com/ArduPilot/ardupilot_gazebo $ARDUPILOT_GAZEBO
(cd $ARDUPILOT_GAZEBO && mkdir -p build && cd build && GZ_VERSION=harmonic cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && nice -n 19 make -j $JOBS)
EOF
}

build_plugin() {
  echo ">> building ardupilot_gazebo in $ARDUPILOT_GAZEBO"
  if [ ! -d "$ARDUPILOT_GAZEBO/.git" ]; then
    git clone https://github.com/ArduPilot/ardupilot_gazebo "$ARDUPILOT_GAZEBO" || return 1
  fi
  (cd "$ARDUPILOT_GAZEBO" && mkdir -p build && cd build \
     && GZ_VERSION=harmonic cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && nice -n 19 make -j "$JOBS")
}

check_gazebo() {
  local req=1
  [ "$BACKEND" = gazebo ] || req=0
  if command -v gz > /dev/null && gz sim --versions 2>/dev/null | grep -q '^8\.'; then
    row OK "gz sim Harmonic" "$(gz sim --versions 2>/dev/null | grep '^8\.' | head -1)"
  else
    row MISSING "gz sim Harmonic" "need gz sim 8.x (found: $(gz sim --versions 2>/dev/null | tr '\n' ' '))" "$req"
    while IFS= read -r c; do fix "$c"; done < <(gz_apt_cmds)
  fi
  local so="$ARDUPILOT_GAZEBO/build/libArduPilotPlugin.so"
  if [ ! -f "$so" ] && [ "$BUILD" = 1 ]; then
    build_plugin || echo "!! ardupilot_gazebo build failed (apt deps? see the commands below)"
  fi
  if [ -f "$so" ]; then
    row OK "ardupilot_gazebo plugin" "$so"
  else
    row MISSING "ardupilot_gazebo plugin" "$so" "$req"
    while IFS= read -r c; do fix "$c"; done < <(plugin_cmds)
  fi
  if [ -d "$ARDUPILOT_GAZEBO/models/iris_with_ardupilot" ]; then   # copied by sim/gazebo/gen_world.py
    row OK "ardupilot_gazebo models" "$ARDUPILOT_GAZEBO/models"
  else
    row MISSING "ardupilot_gazebo models" "$ARDUPILOT_GAZEBO/models (iris)" "$req"
  fi
}

# ---------------------------------------------------------------- path consistency
check_paths() {
  # sim/launch.py hard-codes ~/ardupilot and ~/ardupilot_gazebo; warn if the env points elsewhere
  if [ "$(realpath -m "$ARDUPILOT_HOME")" != "$(realpath -m "$HOME/ardupilot")" ] \
     && ! grep -q ARDUPILOT_HOME "$ROOT/sim/launch.py"; then
    row WARN "ARDUPILOT_HOME" "sim/launch.py ignores it and uses ~/ardupilot" 0
  fi
  if [ "$(realpath -m "$ARDUPILOT_GAZEBO")" != "$(realpath -m "$HOME/ardupilot_gazebo")" ] \
     && ! grep -q ARDUPILOT_GAZEBO "$ROOT/sim/launch.py"; then
    row WARN "ARDUPILOT_GAZEBO" "sim/launch.py ignores it and uses ~/ardupilot_gazebo" 0
  fi
}

# ---------------------------------------------------------------- main
check_python
check_tools
check_ardupilot
check_gazebo
check_paths

echo
printf '%-8s %-26s %s\n' STATUS COMPONENT DETAIL
printf '%-8s %-26s %s\n' ------ --------- ------
for r in "${ROWS[@]}"; do
  IFS='|' read -r st name detail <<< "$r"
  printf '%-8s %-26s %s\n' "$st" "$name" "$detail"
done
if [ ${#FIXES[@]} -gt 0 ]; then
  echo
  echo "To fix (run these yourself; nothing here was run with sudo):"
  printf '  %s\n' "${FIXES[@]}" | awk '!seen[$0]++'
  [ "$BUILD" = 0 ] && echo "  (or: ./setup.sh --build  to clone/build ArduPilot + ardupilot_gazebo and pip-install --user)"
fi
echo
if [ "$missing_required" = 0 ]; then
  echo "setup: OK for BACKEND=$BACKEND (./run_eval.sh)"
  exit 0
fi
echo "setup: something required for BACKEND=$BACKEND is MISSING (see above)"
exit 1
