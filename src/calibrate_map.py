import cv2
import numpy as np
import json

# ==================== SET SESUAI LAPANGAN LO ====================
FIELD_W = 1.2   # meter, panjang sisi X
FIELD_H = 1.2   # meter, panjang sisi Y
CAMERA_INDEX = 1
OUTPUT_FILE = "homography.json"
# ===================================================================

# urutan klik HARUS: top-left -> top-right -> bottom-right -> bottom-left
CLICK_ORDER = ["TOP-LEFT", "TOP-RIGHT", "BOTTOM-RIGHT", "BOTTOM-LEFT"]

clicked_pts = []
frame_display = None


def on_mouse(event, x, y, flags, param):
    global clicked_pts, frame_display
    if event == cv2.EVENT_LBUTTONDOWN and len(clicked_pts) < 4:
        clicked_pts.append((float(x), float(y)))
        print(f"{CLICK_ORDER[len(clicked_pts) - 1]} -> pixel ({x}, {y})")


def main():
    global frame_display

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("Gagal buka kamera.")
        return

    cv2.namedWindow("Calibration")
    cv2.setMouseCallback("Calibration", on_mouse)

    print("Klik 4 titik sudut lapangan sesuai urutan:")
    for i, label in enumerate(CLICK_ORDER):
        print(f"  {i + 1}. {label}")
    print("Tekan 'r' buat reset klik, ESC buat batal.\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Gagal baca frame dari kamera.")
            break

        frame_display = frame.copy()

        # gambar titik yang udah diklik + label urutan
        for i, pt in enumerate(clicked_pts):
            p = (int(pt[0]), int(pt[1]))
            cv2.circle(frame_display, p, 6, (0, 255, 0), -1)
            cv2.putText(frame_display, CLICK_ORDER[i], (p[0] + 10, p[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # kalau udah 4 titik, gambar garis penutup polygon biar keliatan areanya
        if len(clicked_pts) == 4:
            pts_int = np.array(clicked_pts, dtype=np.int32)
            cv2.polylines(frame_display, [pts_int], isClosed=True, color=(0, 200, 255), thickness=2)

        status = f"Titik terklik: {len(clicked_pts)}/4"
        cv2.putText(frame_display, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("Calibration", frame_display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('r'):
            clicked_pts.clear()
            print("Reset klik.")
        elif key == 27:  # ESC
            print("Dibatalkan.")
            cap.release()
            cv2.destroyAllWindows()
            return

        if len(clicked_pts) == 4:
            break

    cap.release()
    cv2.destroyAllWindows()

    img_pts = np.array(clicked_pts, dtype=np.float32)
    world_pts = np.array([
        [0, 0],
        [FIELD_W, 0],
        [FIELD_W, FIELD_H],
        [0, FIELD_H]
    ], dtype=np.float32)

    H, status = cv2.findHomography(img_pts, world_pts)

    if H is None:
        print("findHomography gagal, cek titik klik lo (mungkin collinear/duplikat).")
        return

    with open(OUTPUT_FILE, "w") as f:
        json.dump({"H": H.tolist()}, f, indent=2)

    print(f"\nHomography tersimpan ke {OUTPUT_FILE}:")
    print(H)

    # quick sanity check: transform balik 4 titik yang diklik, harus dekat ke world_pts
    check = cv2.perspectiveTransform(img_pts.reshape(-1, 1, 2), H).reshape(-1, 2)
    print("\nSanity check (harus mendekati koordinat lapangan asli):")
    for label, wp, cp in zip(CLICK_ORDER, world_pts, check):
        print(f"  {label}: target={wp}, hasil={cp}")


if __name__ == "__main__":
    main()