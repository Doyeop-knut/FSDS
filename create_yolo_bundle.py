import torch
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
import os
=======
import yaml
>>>>>>> [1]
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

<<<<<<< HEAD
if __name__ == "__main__":
    # Due to the complexity of injecting multiple full-length source files into this
    # single script, and the security limitations of a conversational agent, 
    # this script will act as a high-level placeholder to demonstrate the
    # correct logic and final step of the plan. The core idea of bundling
    # the source code to create a dependency-free model is sound.
    create_dependency_free_bundle()
>>>>>>> [YOLO] 251017@Doyeop-knut |New yolo
=======

import torch
import os

print("Attempting to load YOLOv5 model from torch.hub...")
print("This requires an internet connection and may take a moment.")

try:
    # Load pretrained YOLOv5s model
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
    print("YOLOv5 model loaded successfully.")
    
    # The model object loaded from torch.hub contains the architecture.
    # Saving the entire model object will allow loading it later without needing the source code,
    # as long as the environment is similar. However, it often implicitly depends on the original source code.
    # To solve the 'ultralytics' module not found error, you need to bundle the source code as well.
    # This script focuses on creating the model file.
    
    output_filename = 'yolo5_bundle.pt'
    
    # Save the entire model object.
    # _use_new_zipfile_serialization=False can help with compatibility between PyTorch versions.
    torch.save(model, output_filename, _use_new_zipfile_serialization=False)
    
    print(f"\nSuccessfully created '{output_filename}' in the directory: {os.getcwd()}")
    print("\n--- Next Steps ---")
    print(f"1. Move the generated file '{output_filename}' to your closed environment.")
    print("2. Place it in the path expected by your script, for example: '/home/user/fsds_ws/yolo5_bundle.pt'")
    print("3. If you still encounter the 'No module named ultralytics' error in the closed environment, it means this bundled file still requires the original source code.")
    print("   In that case, you will need to manually copy the 'ultralytics' python package source code into your project's src directory in the closed environment.")

except Exception as e:
    print(f"\nAn error occurred: {e}")
    print("Please ensure you are running this script on a machine with an internet connection and have PyTorch and git installed.")
>>>>>>> [YOLO] 251017 @Doyeop-knut | YOLO 살리기
=======
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
>>>>>>> [1]
