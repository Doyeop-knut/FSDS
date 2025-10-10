<<<<<<< HEAD
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
=======
import torch

# 1. 인터넷과 yolov5 소스가 있는 환경에서 평소처럼 모델을 불러옵니다.
#    이 단계는 대회 환경이 아닌, 당신의 로컬 개발 환경에서 실행합니다.
yolov5_repo = 'ultralytics/yolov5'
weights_path = '/home/user/fsds_ws/yolo5-cone.pt' # 당신이 학습한 가중치 파일

# 모델 구조와 가중치가 합쳐진 완전한 객체를 메모리에 로드
model = torch.hub.load(yolov5_repo, 'custom', path=weights_path, force_reload=True)

# 2. 모델 객체 전체를 저장합니다.
#    model.state_dict()가 아닌, 'model' 자체를 저장하는 것이 핵심입니다.
output_path = 'yolov5_bundle.pt'
torch.save(model, output_path)

print(f"모델 구조와 가중치가 하나로 합쳐진 '{output_path}' 파일이 생성되었습니다.")
print("이 파일과 추론 코드(.py)를 함께 제출하세요.")
>>>>>>> [detectionModel] 251011 @Doyeop-knut | 가중치 -> 통합 model 자체를 output으로 저장
