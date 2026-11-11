#!/usr/bin/env python3
"""Gazebo kamera konusundan tek kare alip PNG olarak kaydeder (gz CLI + PIL)."""
import re
import subprocess
import sys

from PIL import Image

topic = "/world/default/model/toy_0/link/camera_link/sensor/camera/image"
out = sys.argv[1]
txt = subprocess.run(["gz", "topic", "-e", "-n", "1", "-t", topic], capture_output=True, text=True, timeout=15).stdout
w = int(re.search(r"^width: (\d+)", txt, re.M).group(1))
h = int(re.search(r"^height: (\d+)", txt, re.M).group(1))
raw = re.search(r'^data: "(.*)"$', txt, re.M | re.S).group(1)
data = raw.encode("latin-1").decode("unicode_escape").encode("latin-1")
Image.frombytes("RGB", (w, h), data[: w * h * 3]).save(out)
print(out, w, h)
