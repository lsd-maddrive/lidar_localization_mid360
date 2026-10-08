#!/usr/bin/env python3
import sys
from pathlib import Path

import numpy as np


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit(f"Usage: {Path(sys.argv[0]).name} input.pcd output.txt")

    input_path, output_path = map(Path, sys.argv[1:])
    header = bytearray()

    with input_path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise RuntimeError("PCD header has no DATA line")
            header.extend(line)
            if line.startswith(b"DATA "):
                data_mode = line.decode("ascii").strip().split()[1].lower()
                break

        fields = sizes = types = counts = points = None
        for raw_line in bytes(header).splitlines():
            parts = raw_line.decode("ascii").split()
            if not parts:
                continue
            key = parts[0].upper()
            if key == "FIELDS":
                fields = parts[1:]
            elif key == "SIZE":
                sizes = [int(x) for x in parts[1:]]
            elif key == "TYPE":
                types = parts[1:]
            elif key == "COUNT":
                counts = [int(x) for x in parts[1:]]
            elif key == "POINTS":
                points = int(parts[1])

        if data_mode != "binary":
            raise RuntimeError(f"Expected binary PCD, got DATA {data_mode}")
        if fields != ["x", "y", "z", "intensity"]:
            raise RuntimeError(f"Expected x y z intensity fields, got {fields}")
        if sizes != [4, 4, 4, 4] or types != ["F", "F", "F", "F"]:
            raise RuntimeError("Expected four float32 fields")
        if counts not in (None, [1, 1, 1, 1]) or points is None:
            raise RuntimeError("Unsupported PCD COUNT/POINTS header")

        values = np.fromfile(stream, dtype="<f4", count=points * 4)
        if values.size != points * 4:
            raise RuntimeError(f"Expected {points * 4} values, got {values.size}")

    np.savetxt(output_path, values.reshape(points, 4), fmt="%.7g")
    print(f"Wrote {points} points to {output_path}")


if __name__ == "__main__":
    main()
