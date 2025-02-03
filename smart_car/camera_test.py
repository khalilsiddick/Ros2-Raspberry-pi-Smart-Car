import cv2

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
if not cap.isOpened():
    print("❌ Camera failed to open")
    exit()

print("✅ Camera working - Press Ctrl+C to exit")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ Frame read error")
            break
        print(f"Frame received: {frame.shape}")
except KeyboardInterrupt:
    pass

cap.release()
print("🛑 Camera released")
