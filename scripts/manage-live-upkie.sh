#!/usr/bin/env bash
set -euo pipefail

BONESAW_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BONESAW_SERVER_BIN="$BONESAW_PROJECT_ROOT/target/release/bonesaw-server"
BONESAW_UPKIE_URDF="$BONESAW_PROJECT_ROOT/models/upkie/upkie.urdf"
BONESAW_PLANT_WORKER="$BONESAW_PROJECT_ROOT/python/evals/upkie_live_plant_worker.py"
BONESAW_PLANT_PYTHON="${BONESAW_PLANT_PYTHON:-/tmp/bonesaw-mujoco/bin/python}"
BONESAW_TUNNEL_BIN="${BONESAW_TUNNEL_BIN:-/tmp/cloudflared}"
BONESAW_LIVE_TTL="${BONESAW_LIVE_TTL:-8h}"
BONESAW_LIVE_RUSTFLAGS="${BONESAW_LIVE_RUSTFLAGS:--C target-cpu=native}"
BONESAW_UNITS=(bonesaw-local bonesaw-public bonesaw-tunnel)
BONESAW_PORT=8777

cleanup_project_orphans() {
  local pid executable arguments
  while read -r pid executable arguments; do
    if [[ "$executable" == "$BONESAW_SERVER_BIN" ]]; then
      kill -TERM "$pid" 2>/dev/null || true
    elif [[ "$executable" == "$BONESAW_TUNNEL_BIN" ]] \
      && [[ "$arguments" == *"tunnel --url http://127.0.0.1:8777"* \
        || "$arguments" == *"tunnel --url http://127.0.0.1:8799"* \
        || "$arguments" == *"tunnel --url http://127.0.0.1:8819"* ]]; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done < <(ps -eo pid=,args=)
}

stop_live() {
  systemctl --user stop "${BONESAW_UNITS[@]}" 2>/dev/null || true
  systemctl --user reset-failed "${BONESAW_UNITS[@]}" 2>/dev/null || true
  for unit in "${BONESAW_UNITS[@]}"; do
    if systemctl --user is-active --quiet "$unit"; then
      echo "failed to stop $unit" >&2
      exit 1
    fi
  done
  cleanup_project_orphans
}

current_tunnel_url() {
  local invocation
  invocation="$(systemctl --user show bonesaw-tunnel -p InvocationID --value)"
  [[ -n "$invocation" ]] || return 1
  journalctl --user --no-pager -o cat _SYSTEMD_INVOCATION_ID="$invocation" \
    | rg -o 'https://[a-z0-9-]+\.trycloudflare\.com' \
    | tail -n 1
}

start_live() {
  if [[ ! -x "$BONESAW_TUNNEL_BIN" ]]; then
    echo "cloudflared not executable at $BONESAW_TUNNEL_BIN" >&2
    echo "set BONESAW_TUNNEL_BIN to an explicit binary path" >&2
    exit 1
  fi
  if [[ ! -x "$BONESAW_PLANT_PYTHON" ]] \
    || ! "$BONESAW_PLANT_PYTHON" -c 'import bonesaw, mujoco, numpy' >/dev/null 2>&1; then
    echo "live plant environment is unavailable at $BONESAW_PLANT_PYTHON" >&2
    echo "prepare it with scripts/run-upkie-rooted-capture-plant-report.sh" >&2
    exit 1
  fi
  if [[ ! -f "$BONESAW_PLANT_WORKER" ]]; then
    echo "live plant worker is missing: $BONESAW_PLANT_WORKER" >&2
    exit 1
  fi
  if [[ ! -x "$BONESAW_PLANT_PYTHON" ]]; then
    echo "live plant Python is not executable at $BONESAW_PLANT_PYTHON" >&2
    exit 1
  fi

  RUSTFLAGS="$BONESAW_LIVE_RUSTFLAGS" \
    cargo build --manifest-path "$BONESAW_PROJECT_ROOT/Cargo.toml" \
    --release --bin bonesaw-server

  # Replace only our three exact user units. Never process-match cloudflared:
  # this machine also owns an unrelated named tunnel.
  stop_live
  local cleanup_on_exit=1
  trap 'if [[ "${cleanup_on_exit:-0}" == 1 ]]; then stop_live; fi' EXIT

  if ss -H -ltn "sport = :$BONESAW_PORT" | rg -q .; then
    echo "port $BONESAW_PORT remains occupied after project cleanup" >&2
    exit 1
  fi

  systemd-run --user --unit=bonesaw-public --collect --quiet \
    --property="RuntimeMaxSec=$BONESAW_LIVE_TTL" \
    --working-directory="$BONESAW_PROJECT_ROOT" \
    --setenv=BONESAW_BIND=0.0.0.0:$BONESAW_PORT \
    --setenv=BONESAW_LIVE_GUIDED=1 \
    --setenv=BONESAW_LIVE_PLANT=1 \
    --setenv=BONESAW_PLANT_PYTHON="$BONESAW_PLANT_PYTHON" \
    --setenv=BONESAW_PLANT_WORKER="$BONESAW_PLANT_WORKER" \
    --setenv=RUST_LOG=info \
    "$BONESAW_SERVER_BIN" "$BONESAW_UPKIE_URDF"

  systemd-run --user --unit=bonesaw-tunnel --collect --quiet \
    --property="RuntimeMaxSec=$BONESAW_LIVE_TTL" \
    --property=After=bonesaw-public.service \
    --property=BindsTo=bonesaw-public.service \
    "$BONESAW_TUNNEL_BIN" tunnel --url http://127.0.0.1:$BONESAW_PORT --no-autoupdate

  local public_url=""
  for _ in {1..80}; do
    public_url="$(current_tunnel_url || true)"
    if [[ -n "$public_url" ]]; then
      break
    fi
    sleep 0.25
  done
  if [[ -z "$public_url" ]]; then
    echo "current tunnel invocation did not publish a URL" >&2
    exit 1
  fi

  local public_host="${public_url#https://}"
  local connect_address=""
  local verified=0
  local probe_error=""
  for _ in {1..80}; do
    if [[ -z "$connect_address" ]]; then
      connect_address="$(dig +short @1.1.1.1 "$public_host" A \
        | rg '^[0-9]+(\.[0-9]+){3}$' \
        | head -n 1 || true)"
    fi
    if [[ -z "$connect_address" ]]; then
      probe_error="Cloudflare public DNS has not published an A record yet"
      sleep 0.5
      continue
    fi
    local probe_command=(
      python3 "$BONESAW_PROJECT_ROOT/python/evals/live_editor_smoke.py"
      --url "$public_url"
    )
    if [[ -n "$connect_address" ]]; then
      probe_command+=(--connect-address "$connect_address")
    fi
    if probe_error="$("${probe_command[@]}" 2>&1)"; then
      local plant_probe_command=(
        "$BONESAW_PLANT_PYTHON" "$BONESAW_PROJECT_ROOT/python/evals/live_plant_gateway_smoke.py"
        --url "$public_url"
      )
      if [[ -n "$connect_address" ]]; then
        plant_probe_command+=(--connect-address "$connect_address")
      fi
      if probe_error="$("${plant_probe_command[@]}" 2>&1)"; then
        local wrench_probe_command=(
          "$BONESAW_PLANT_PYTHON" "$BONESAW_PROJECT_ROOT/python/evals/live_wrench_application_probe.py"
          --url "$public_url"
        )
        if [[ -n "$connect_address" ]]; then
          wrench_probe_command+=(--connect-address "$connect_address")
        fi
        if probe_error="$("${wrench_probe_command[@]}" 2>&1)"; then
          verified=1
          break
        fi
      fi
    fi
    sleep 0.5
  done
  if [[ "$verified" != 1 ]]; then
    echo "public HTTP/WebSocket Upkie verification failed; URL withheld" >&2
    echo "$probe_error" >&2
    exit 1
  fi

  echo "started one Upkie server and one tunnel (automatic cleanup after $BONESAW_LIVE_TTL)"
  echo "$public_url"
  cleanup_on_exit=0
  trap - EXIT
}

status_live() {
  systemctl --user --no-pager --full status "${BONESAW_UNITS[@]}" || true
}

url_live() {
  current_tunnel_url
}

case "${1:-status}" in
  start) start_live ;;
  stop) stop_live ;;
  restart) start_live ;;
  status) status_live ;;
  url) url_live ;;
  *)
    echo "usage: $0 {start|stop|restart|status|url}" >&2
    exit 2
    ;;
esac
