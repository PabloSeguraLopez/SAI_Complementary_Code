#!/usr/bin/env bash

PROJECT_DIR="/root/aerostack2_ws/project_gazebo"
OUTPUT_DIR="$PROJECT_DIR/Out"
PYTHON_SCRIPT="..."

cleanup() {
  echo "Cerrando procesos..."

  if [ -n "$PID2" ] && kill -0 "$PID2" 2>/dev/null; then
    kill "$PID2" 2>/dev/null
    sleep 1
    kill -9 "$PID2" 2>/dev/null
  fi

  if [ -n "$PID1" ] && kill -0 "$PID1" 2>/dev/null; then
    kill "$PID1" 2>/dev/null
    sleep 1
    kill -9 "$PID1" 2>/dev/null
  fi

  if [ -f "$PROJECT_DIR/stop.bash" ]; then
    bash "$PROJECT_DIR/stop.bash"
  else
    echo "No se encontró $PROJECT_DIR/stop.bash"
  fi
}

on_exit() {
  exit_code=$?
  cleanup
  exit "$exit_code"
}

trap on_exit SIGINT SIGTERM

cd "$PROJECT_DIR" || {
  echo "No se pudo acceder a $PROJECT_DIR"
  exit 1
}

mkdir -p "$OUTPUT_DIR" || {
  echo "No se pudo crear $OUTPUT_DIR"
  exit 1
}

bash launch_as2.bash > "$OUTPUT_DIR/launch_as2.log" 2>&1 &
PID1=$!

sleep 5

python3 "$PYTHON_SCRIPT" > "$OUTPUT_DIR/main.log" 2>&1 &
PID2=$!

echo "launch_as2.bash PID: $PID1"
echo "$PYTHON_SCRIPT PID: $PID2"
echo "Logs en: $OUTPUT_DIR"

wait "$PID2"
py_exit_code=$?

echo "$PYTHON_SCRIPT terminó con código: $py_exit_code"

cleanup
exit "$py_exit_code"