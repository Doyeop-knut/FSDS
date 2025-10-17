import torch
import yaml
import sys
import shutil
from pathlib import Path
from copy import deepcopy
import math
import torch.nn as nn

# This script creates a dependency-free YOLOv5 model bundle (.pt file).

# ===================================================================
# 1. DEFINE SOURCE CODE AND CONFIG AS STRINGS
# ===================================================================

INIT_PY = ''

# Use triple single quotes and escape internal triple quotes
CONV_PY = '''# Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license
"""Convolution modules."""

import math
from typing import List

import numpy as np
import torch
import torch.nn as nn

__all__ = (
    "Conv",
    "Conv2",
    "LightConv",
    "DWConv",
    "DWConvTranspose2d",
    "ConvTranspose",
    "Focus",
    "GhostConv",
    "ChannelAttention",
    "SpatialAttention",
    "CBAM",
    "Concat",
    "RepConv",
    "Index",
)


def autopad(k, p=None, d=1):  # kernel, padding, dilation
    """Pad to 'same' shape outputs."""
    if d > 1:
        k = d * (k - 1) + 1 if isinstance(k, int) else [d * (x - 1) + 1 for x in k]  # actual kernel-size
    if p is None:
        p = k // 2 if isinstance(k, int) else [x // 2 for x in k]  # auto-pad
    return p


class Conv(nn.Module):
    """Standard convolution with batch normalization and activation."""

    default_act = nn.SiLU()  # default activation

    def __init__(self, c1, c2, k=1, s=1, p=None, g=1, d=1, act=True):
        super().__init__()
        self.conv = nn.Conv2d(c1, c2, k, s, autopad(k, p, d), groups=g, dilation=d, bias=False)
        self.bn = nn.BatchNorm2d(c2)
        self.act = self.default_act if act is True else act if isinstance(act, nn.Module) else nn.Identity()

    def forward(self, x):
        return self.act(self.bn(self.conv(x)))

    def forward_fuse(self, x):
        return self.act(self.conv(x))
'''

# Dummy placeholders for other modules that will be filled in incrementally
DUMMY_PY = """import torch.nn as nn
class C3(nn.Module):
    def __init__(self, *args, **kwargs):
        super().__init__()
"""


# ===================================================================
# 2. SCRIPT LOGIC
# ===================================================================

def create_bundle():
    temp_dir = Path("./temp_ultralytics_src")
    original_model_path = '/home/user/fsds_ws/yolo5_bundle.pt'
    output_model_path = '/home/user/fsds_ws/yolov5_bundle_nodeps.pt'

    print(f"Creating temporary package structure at {temp_dir}...")
    try:
        # Define package structure and content
        package_files = {
            "ultralytics/__init__.py": INIT_PY,
            "ultralytics/nn/__init__.py": INIT_PY,
            "ultralytics/nn/modules/__init__.py": INIT_PY,
            "ultralytics/nn/modules/conv.py": CONV_PY.replace('"""', "'''"), # Escape docstrings
            "ultralytics/nn/modules/block.py": DUMMY_PY,
            "ultralytics/nn/modules/head.py": DUMMY_PY,
            "ultralytics/nn/tasks.py": DUMMY_PY,
            "ultralytics/utils/__init__.py": INIT_PY,
            "ultralytics/utils/ops.py": DUMMY_PY,
        }

        for file_path, content in package_files.items():
            full_path = temp_dir / file_path
            full_path.parent.mkdir(parents=True, exist_ok=True)
            full_path.write_text(content)
        
        print("Temporary package created.")

        sys.path.insert(0, str(temp_dir.resolve()))
        print("Temporarily added to sys.path.")

        print(f"Loading original model '{original_model_path}'...")
        # This is the crucial step. torch.load will now find the fake package.
        model = torch.load(original_model_path, map_location='cpu')
        print("Original model loaded successfully.")

        print(f"Saving dependency-free model to '{output_model_path}'...")
        torch.save(model, output_model_path)
        print(f"✅ Successfully created dependency-free model: '{output_model_path}'")

    except Exception as e:
        print(f"\nAn error occurred: {e}")

    finally:
        if temp_dir.exists():
            print(f"Cleaning up temporary directory: {temp_dir}")
            shutil.rmtree(temp_dir)
        if str(temp_dir.resolve()) in sys.path:
            sys.path.remove(str(temp_dir.resolve()))

if __name__ == '__main__':
    create_bundle()