import sys
import cv2
import numpy as np

cam_index = 1
if len(sys.argv) > 1:
    cam_index = int(sys.argv[1])

cap = cv2.VideoCapture(cam_index)

WINDOW = "HSV Calibrator"
cv2.namedWindow(WINDOW)


def nothing(x):
    pass


# Trackbar buat H, S, V min-max
cv2.createTrackbar("H min", WINDOW, 0, 179, nothing)
cv2.createTrackbar("H max", WINDOW, 179, 179, nothing)
cv2.createTrackbar("S min", WINDOW, 0, 255, nothing)
cv2.createTrackbar("S max", WINDOW, 255, 255, nothing)
cv2.createTrackbar("V min", WINDOW, 0, 255, nothing)
cv2.createTrackbar("V max", WINDOW, 255, 255, nothing)

# Preset starting point buat orange (biar gampang mulai geser dari sini)
cv2.setTrackbarPos("H min", WINDOW, 10)
cv2.setTrackbarPos("H max", WINDOW, 20)
cv2.setTrackbarPos("S min", WINDOW, 150)
cv2.setTrackbarPos("V min", WINDOW, 80)

print("=== HSV Calibrator ===")
print("Geser trackbar sampai cuma objek target yang keliatan putih di window 'Mask'.")
print("Tekan 's' buat print range HSV yang lagi aktif ke console.")
print("Tekan 'q' buat keluar.")
print()
print("Panduan hue (skala OpenCV 0-179):")
print("  Merah   : ~0-8 dan ~172-179")
print("  Orange  : ~10-20")
print("  Kuning  : ~25-35")
print("  Hijau   : ~40-80")
print("  Biru    : ~100-130")
print()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    h_min = cv2.getTrackbarPos("H min", WINDOW)
    h_max = cv2.getTrackbarPos("H max", WINDOW)
    s_min = cv2.getTrackbarPos("S min", WINDOW)
    s_max = cv2.getTrackbarPos("S max", WINDOW)
    v_min = cv2.getTrackbarPos("V min", WINDOW)
    v_max = cv2.getTrackbarPos("V max", WINDOW)

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(hsv, lower, upper)

    kernel = np.ones((7, 7), np.uint8)
    mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)

    # highlight area yang lolos filter, biar keliatan konteksnya di frame asli
    result = cv2.bitwise_and(frame, frame, mask=mask_clean)

    # kasih bounding box biar keliatan objek yang bakal ke-capture kontur
    contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 1500:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, f"area:{int(area)}", (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow("Original", frame)
    cv2.imshow("Mask", mask_clean)
    cv2.imshow("Result", result)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('s'):
        print(f"lower = np.array([{h_min}, {s_min}, {v_min}])")
        print(f"upper = np.array([{h_max}, {s_max}, {v_max}])")
        print()

cap.release()
cv2.destroyAllWindows()