"""Compare actual Python camera methods with C++; seed and units are explicit."""
import pathlib
import subprocess
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(sys.argv[2]).resolve()))
from camera_model.rgbd_camera import RGBDCamera

K = np.array([[517.3, 0, 318.6], [0, 521.8, 239.2], [0, 0, 1]])
# Registered grid: depth_K equals RGB K. Both APIs convert registered raw depth to Z meters.
rgbd = RGBDCamera("baseline", K, np.zeros(5), (640, 480), depth_scale=0.001)
rng = np.random.default_rng(20260910)
points = rng.uniform(-5, 5, size=(256, 3))
points[:, 2] = rng.uniform(0.05, 20, size=len(points))
uv = rgbd.project(points)
expected = np.column_stack((uv, rgbd.unproject(uv, points[:, 2] / rgbd.depth_scale)))
payload = "\n".join(" ".join(format(v, ".17g") for v in row) for row in points)
result = subprocess.run([sys.argv[1]], input=payload + "\n", text=True,
                        check=True, capture_output=True)
actual = np.array([list(map(float, row.split())) for row in result.stdout.splitlines()])
np.testing.assert_allclose(actual, expected, atol=1e-8, rtol=1e-6)
print(f"Python camera parity passed: {len(points)} points; NumPy {np.__version__}")
