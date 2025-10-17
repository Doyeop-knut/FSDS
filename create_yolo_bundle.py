
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
