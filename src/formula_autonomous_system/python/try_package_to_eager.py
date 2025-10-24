import torch

m = torch.load("retinanet_QAT.pt", map_location="cuda")
m.eval()

try:
    m.half()
    x = torch.rand(3, 540, 960, device="cuda").half()
    with torch.no_grad():
        out = m([x])
    print("[OK] CUDA FP16 경로 정상 동작")
except Exception as e:
    print("[WARN] FP16 변환에서 예외 발생:", e)
