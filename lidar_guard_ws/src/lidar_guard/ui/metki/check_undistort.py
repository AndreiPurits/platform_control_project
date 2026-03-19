# check_undistort.py
import cv2
import numpy as np
import glob
import os

CALIB = "camera_calib.npz"
IMG_GLOB = "calib_imgs/*.jpg"   # <-- поменяй путь под свои фотки

def main():
    data = np.load(CALIB)
    K = data["K"]
    D = data["D"]

    paths = sorted(glob.glob(IMG_GLOB))
    if not paths:
        print("No images found:", IMG_GLOB)
        return

    for p in paths[:30]:
        img = cv2.imread(p)
        if img is None:
            continue
        h, w = img.shape[:2]

        newK, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1.0, (w, h))
        und = cv2.undistort(img, K, D, None, newK)

        # split view
        vis = np.hstack([img, und])
        cv2.putText(vis, "RAW", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,255,0), 2)
        cv2.putText(vis, "UNDIST", (w+20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,255,0), 2)

        cv2.imshow("raw | undist", vis)
        key = cv2.waitKey(0) & 0xFF
        if key in (27, ord('q')):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()