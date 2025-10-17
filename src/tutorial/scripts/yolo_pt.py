# create_bundle_from_local.py

import torch
from models.common import DetectMultiBackend # 로컬 소스 코드에서 직접 import
from models.experimental import attempt_load # 로컬 소스 코드에서 직접 import

# --- 사용자 설정 ---
weights_path = '/home/user/fsds_ws/yolo5-cone.pt' # 학습된 가중치 파일 경로
output_path = 'yolov5_bundle_final.pt'      # 최종 결과물 파일 이름
device = 'cuda' if torch.cuda.is_available() else 'cpu'

# 1. 로컬 소스 코드를 사용해 모델 로드 (torch.hub 대신 사용)
#    이렇게 하면 버전이 완벽하게 일치합니다.
model = attempt_load(weights_path, device=device)

# 2. 순수 모델 객체를 저장합니다.
#    이 모델 객체는 `ultralytics` 의존성이 없습니다.
torch.save(model, output_path)

print(f"✅ 버전 문제가 해결된 '{output_path}' 파일이 생성되었습니다.")
print("이 파일을 실행 환경으로 가져가서 torch.load()로 사용하세요.")