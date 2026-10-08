#!/usr/bin/env bash
set -euo pipefail
workspace_root="${FORKLIFT_WS:-$(pwd)}/maps"; map_name=""; force=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace-root) workspace_root="$2"; shift 2;;
    --map-name) map_name="$2"; shift 2;;
    --force) force=1; shift;;
    -h|--help) echo "Usage: close_loops.sh --map-name NAME [--workspace-root PATH] [--force]"; exit 0;;
    *) echo "Unknown argument: $1" >&2; exit 2;;
  esac
done
[[ -n "$map_name" ]] || { echo "--map-name is required" >&2; exit 2; }
input="${workspace_root}/${map_name}.simplemap"; output="${workspace_root}/${map_name}_lc.simplemap"; externals="${workspace_root}/${map_name}_Images"
if [[ ! -f "$input" && -f "${workspace_root}/${map_name}" ]]; then
  input="${workspace_root}/${map_name}"
fi
[[ -f "$input" ]] || { echo "Input map not found: ${workspace_root}/${map_name}.simplemap or ${workspace_root}/${map_name}" >&2; exit 1; }
if [[ -e "$output" && "$force" -ne 1 ]]; then echo "Output exists: $output (use --force to replace it)" >&2; exit 1; fi
command -v mola-sm-lc-cli >/dev/null || { echo "mola-sm-lc-cli is missing. Install ros-jazzy-mola-sm-loop-closure." >&2; exit 1; }
pkg_prefix="$(ros2 pkg prefix mola_sm_loop_closure 2>/dev/null || true)"; pipeline="${pkg_prefix}/share/mola_sm_loop_closure/pipelines/loop-closure-lidar3d-gicp.yaml"
[[ -f "$pipeline" ]] || { echo "Loop-closure pipeline not found: $pipeline" >&2; exit 1; }
args=(-a mola::SimplemapLoopClosure -p "$pipeline" -i "$input" -o "$output"); [[ -d "$externals" ]] && args+=(--externals-dir "$externals")
echo "Running loop closure: $input -> $output"; mola-sm-lc-cli "${args[@]}"
