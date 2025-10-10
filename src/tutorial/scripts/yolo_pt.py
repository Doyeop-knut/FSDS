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