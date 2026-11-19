#!/usr/bin/env bash
# Check what run_eval.sh needs and print an OK / MISSING table with the command that fixes each.
#
#   ./setup.sh           check only
#   ./setup.sh --build   also clone and build ArduCopter SITL and ardupilot_gazebo at the pinned
#                        commits into $ARDUPILOT_HOME and $ARDUPILOT_GAZEBO (if not there yet)
#
# The code depends on parameter names of this ArduPilot commit (WP_SPD, MAV_GCS_SYSID, ...);
# other commits may have renamed them, and MavVehicle then refuses to arm.
# Never runs sudo: system packages are printed for you to install.
set -u
ROOT="$(cd "$(dirname "$0")" && pwd)"
ARDUPILOT_HOME=${ARDUPILOT_HOME:-$HOME/ardupilot}
ARDUPILOT_GAZEBO=${ARDUPILOT_GAZEBO:-$HOME/ardupilot_gazebo}
ARDUPILOT_COMMIT=9165d22419406556fc9815d9f4e3e299e13713a8
AP_GAZEBO_COMMIT=082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5
MISSING=0

row() {   # $1 = OK or MISSING, $2 = what, $3 = detail or the fix
  printf '%-8s %-24s %s\n' "$1" "$2" "$3"
  [ "$1" = OK ] || MISSING=1
}

clone_at() {   # $1 = url, $2 = directory, $3 = commit
  [ -d "$2/.git" ] && return 0
  git clone "$1" "$2" && git -C "$2" checkout "$3" \
    && git -C "$2" submodule update --init --recursive
}

build() {
  echo ">> ArduCopter SITL in $ARDUPILOT_HOME (prerequisites: Tools/environment_install/)"
  clone_at https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_HOME" "$ARDUPILOT_COMMIT" \
    && (cd "$ARDUPILOT_HOME" && ./waf configure --board sitl && nice -n 19 ./waf copter -j 4)
  echo ">> ardupilot_gazebo in $ARDUPILOT_GAZEBO"
  clone_at https://github.com/ArduPilot/ardupilot_gazebo "$ARDUPILOT_GAZEBO" "$AP_GAZEBO_COMMIT" \
    && mkdir -p "$ARDUPILOT_GAZEBO/build" && (cd "$ARDUPILOT_GAZEBO/build" \
    && GZ_VERSION=harmonic cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && nice -n 19 make -j 4)
}

check_commit() {   # $1 = name, $2 = directory, $3 = pinned commit
  local head
  head=$(git -C "$2" rev-parse HEAD 2> /dev/null)
  if [ "$head" = "$3" ]; then
    row OK "$1 commit" "${3:0:10} (pinned)"
  else
    row MISSING "$1 commit" "${head:-none}, pinned ${3:0:10}: git -C $2 checkout $3"
  fi
}

[ "${1:-}" = --build ] && build

echo
printf '%-8s %-24s %s\n' STATUS COMPONENT "DETAIL / FIX"
if python3 -c 'import sys; sys.exit(sys.version_info < (3, 10))' 2> /dev/null; then
  row OK python3 "$(python3 --version 2>&1)"
else
  row MISSING python3 "need >= 3.10: sudo apt install python3 python3-pip"
fi
# the packages listed in pyproject.toml: dependencies = ["pymavlink", "numpy", "matplotlib"]
for package in $(sed -n 's/^dependencies = \[\(.*\)\]/\1/p' "$ROOT/pyproject.toml" | tr -d '",'); do
  if python3 -c "import $package" 2> /dev/null; then
    row OK "$package" "$(python3 -c "import importlib.metadata as m; print(m.version('$package'))")"
  else
    row MISSING "$package" "python3 -m pip install --user -r $ROOT/requirements.txt"
  fi
done
if command -v ffmpeg > /dev/null; then
  row OK ffmpeg "$(command -v ffmpeg)"
else
  row MISSING ffmpeg "sudo apt install ffmpeg"
fi
if [ -x "$ARDUPILOT_HOME/build/sitl/bin/arducopter" ]; then
  row OK "arducopter SITL" "$ARDUPILOT_HOME/build/sitl/bin/arducopter"
else
  row MISSING "arducopter SITL" "./setup.sh --build (ArduPilot prerequisites first:"
  echo "         $ARDUPILOT_HOME/Tools/environment_install/install-prereqs-ubuntu.sh -y)"
fi
check_commit ardupilot "$ARDUPILOT_HOME" "$ARDUPILOT_COMMIT"
if gz sim --versions 2> /dev/null | grep -q '^8\.'; then
  row OK "gz sim Harmonic" "$(gz sim --versions | grep '^8\.' | head -1)"
else
  row MISSING "gz sim Harmonic" "see https://gazebosim.org/docs/harmonic/install_ubuntu, then:"
  echo "         sudo apt-get update && sudo apt-get install gz-harmonic"
fi
if [ -f "$ARDUPILOT_GAZEBO/build/libArduPilotPlugin.so" ]; then
  row OK "ardupilot_gazebo plugin" "$ARDUPILOT_GAZEBO/build/libArduPilotPlugin.so"
else
  row MISSING "ardupilot_gazebo plugin" "./setup.sh --build (first: sudo apt install"
  echo "         libgz-sim8-dev rapidjson-dev libopencv-dev libgstreamer1.0-dev"
  echo "         libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav)"
fi
check_commit ardupilot_gazebo "$ARDUPILOT_GAZEBO" "$AP_GAZEBO_COMMIT"

echo
if [ "$MISSING" = 0 ]; then echo "setup: OK, ./run_eval.sh can run"
else echo "setup: MISSING items above"; fi
exit "$MISSING"
