#!/usr/bin/env python3
import argparse
import shutil
import subprocess
import tempfile
from pathlib import Path

import numpy as np
from PIL import Image


MIN_Z = 0.05
MAX_Z = 1.5
RESOLUTION = 0.05
MIN_POINTS_PER_CELL = 3
ISOLATED_POINT_RADIUS_M = 0.25
MIN_NEIGHBOR_CELLS = 1
HERE = Path(__file__).resolve().parent
EXTRACTOR = HERE / ".mm_extract"


def remove_isolated_cells(occupied: np.ndarray) -> np.ndarray:
    """Remove occupied cells with too few occupied neighbours nearby."""
    radius_cells = int(np.ceil(ISOLATED_POINT_RADIUS_M / RESOLUTION))
    if radius_cells <= 0 or MIN_NEIGHBOR_CELLS <= 0:
        return occupied

    height, width = occupied.shape
    padded = np.pad(occupied, radius_cells, mode="constant", constant_values=False)
    neighbour_count = np.zeros((height, width), dtype=np.uint16)
    radius_sq = radius_cells * radius_cells

    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if (dx == 0 and dy == 0) or dx * dx + dy * dy > radius_sq:
                continue
            y0 = radius_cells + dy
            x0 = radius_cells + dx
            neighbour_count += padded[y0:y0 + height, x0:x0 + width]

    return occupied & (neighbour_count >= MIN_NEIGHBOR_CELLS)


def build_extractor() -> Path:
    source = HERE / "mm_extract.cpp"
    if EXTRACTOR.exists() and EXTRACTOR.stat().st_mtime >= source.stat().st_mtime:
        return EXTRACTOR
    compiler = shutil.which("g++")
    if not compiler:
        raise RuntimeError("g++ is required to read MOLA .mm files")
    include_args = [arg for path in sorted(Path("/opt/ros/jazzy/include/mrpt").glob("*/include"))
                    for arg in ("-I", str(path))]
    cmd = [compiler, "-std=c++17", "-O2", "-I/opt/ros/jazzy/include", *include_args,
           str(source), "-o", str(EXTRACTOR), "-L/opt/ros/jazzy/lib/x86_64-linux-gnu",
           "-L/opt/ros/jazzy/lib", "-lmola_metric_maps", "-lmp2p_icp_map", "-lmola_kernel",
           "-lmrpt-maps", "-lmrpt-obs", "-lmrpt-poses", "-lmrpt-serialization", "-lmrpt-io",
           "-lmrpt-img", "-lmrpt-config", "-lmrpt-math", "-lmrpt-rtti", "-lmrpt-core",
           "-Wl,-rpath,/opt/ros/jazzy/lib/x86_64-linux-gnu", "-Wl,-rpath,/opt/ros/jazzy/lib"]
    subprocess.run(cmd, check=True)
    return EXTRACTOR


def mm_to_txt(path: Path, temp_dir: Path) -> Path:
    zstd = shutil.which("zstd")
    if not zstd:
        raise RuntimeError("zstd is required to read compressed MOLA .mm files")
    raw = temp_dir / "map.raw.mm"
    with raw.open("wb") as stream:
        subprocess.run([zstd, "-d", "-c", str(path)], stdout=stream, check=True)
    points = temp_dir / "points.xyz"
    subprocess.run([str(build_extractor()), str(raw), str(points)], check=True)
    return points


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert XYZ text or MOLA .mm to a Nav2 map")
    parser.add_argument("input", type=Path, help="XYZ text file or compressed MOLA .mm")
    parser.add_argument("--output-dir", type=Path, default=Path.cwd())
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="mm_to_nav2-") as temporary:
        input_path = args.input
        if input_path.suffix == ".mm":
            input_path = mm_to_txt(input_path, Path(temporary))
        points = np.loadtxt(input_path, usecols=(0, 1, 2), dtype=np.float32)
        points = np.atleast_2d(points)
        points = points[np.isfinite(points).all(axis=1)]
        points = points[(points[:, 2] >= MIN_Z) & (points[:, 2] <= MAX_Z)]
        if len(points) == 0:
            raise RuntimeError("No points remain after Z filtering")

        min_x, min_y = points[:, :2].min(axis=0)
        max_x, max_y = points[:, :2].max(axis=0)
        width = int(np.ceil((max_x - min_x) / RESOLUTION)) + 1
        height = int(np.ceil((max_y - min_y) / RESOLUTION)) + 1

        px = np.floor((points[:, 0] - min_x) / RESOLUTION).astype(np.int64)
        py = height - 1 - np.floor((points[:, 1] - min_y) / RESOLUTION).astype(np.int64)
        valid = (px >= 0) & (px < width) & (py >= 0) & (py < height)

        point_count = np.zeros((height, width), dtype=np.uint16)
        np.add.at(point_count, (py[valid], px[valid]), 1)
        image = np.full((height, width), 255, dtype=np.uint8)
        occupied = point_count >= MIN_POINTS_PER_CELL
        occupied = remove_isolated_cells(occupied)
        image[occupied] = 0
        Image.fromarray(image, mode="L").save(args.output_dir / "nav2_map.pgm")

        (args.output_dir / "nav2_map.yaml").write_text(
            "image: nav2_map.pgm\nmode: trinary\n"
            f"resolution: {RESOLUTION}\norigin: [{min_x:.6f}, {min_y:.6f}, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
        )
        print(f"points={len(points)} map={width}x{height} output={args.output_dir}")


if __name__ == "__main__":
    main()
