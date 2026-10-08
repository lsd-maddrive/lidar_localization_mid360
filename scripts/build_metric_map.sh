#!/usr/bin/env bash
set -euo pipefail
workspace_root="${FORKLIFT_WS:-$(pwd)}/maps"; map_name=""; force=0; decimate_nth=""; decimate_max=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace-root) workspace_root="$2"; shift 2;;
    --map-name) map_name="$2"; shift 2;;
    --force) force=1; shift;;
    --decimate-nth) decimate_nth="$2"; shift 2;;
    --decimate-max) decimate_max="$2"; shift 2;;
    -h|--help) echo "Usage: build_metric_map.sh --map-name NAME [--workspace-root PATH] [--force] [--decimate-nth N] [--decimate-max N]"; exit 0;;
    *) echo "Unknown argument: $1" >&2; exit 2;;
  esac
done
[[ -n "$map_name" ]] || { echo "--map-name is required" >&2; exit 2; }
input="${workspace_root}/${map_name}_lc.simplemap"; output="${workspace_root}/${map_name}_lc.mm"; externals="${workspace_root}/${map_name}_Images"
if [[ ! -f "$input" && -f "${workspace_root}/${map_name}_lc" ]]; then
  input="${workspace_root}/${map_name}_lc"
fi
[[ -f "$input" ]] || { echo "Loop-closed map not found: ${workspace_root}/${map_name}_lc.simplemap or ${workspace_root}/${map_name}_lc" >&2; exit 1; }
if [[ -e "$output" && "$force" -ne 1 ]]; then echo "Output exists: $output (use --force to replace it)" >&2; exit 1; fi
command -v sm2mm >/dev/null || { echo "sm2mm is missing; install mp2p_icp." >&2; exit 1; }
pkg_prefix="$(ros2 pkg prefix lidar_localization_mid360 2>/dev/null || true)"; pipeline="${pkg_prefix}/share/lidar_localization_mid360/config/sm2mm_pointcloud_voxelize.yaml"
[[ -f "$pipeline" ]] || { echo "Metric-map pipeline not found: $pipeline" >&2; exit 1; }
args=(-i "$input" -o "$output" -p "$pipeline"); [[ -d "$externals" ]] && args+=(--externals-dir "$externals"); [[ -n "$decimate_nth" ]] && args+=(--decimate-nth "$decimate_nth"); [[ -n "$decimate_max" ]] && args+=(--decimate-max "$decimate_max")
echo "Running sm2mm: $input -> $output"; sm2mm "${args[@]}"
