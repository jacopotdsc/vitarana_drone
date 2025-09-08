import cv2
import numpy as np

# === 1. Carica immagine ===
img_path = "/home/vboxuser/Pictures/Screenshot from 2025-08-08 15-06-48.png"
img = cv2.imread(img_path)

# === 2. Parametri intrinseci camera ===
camera_matrix = np.array([
    [580.77518, 0.0, 724.75002], 
    [0.0, 580.77518, 570.98956], 
    [0.0, 0.0, 1.0]
])
dist_coeffs = np.array([
    0.927077, 0.141438, 0.000196, -8.7e-05, 
    0.001695, 1.257216, 0.354688, 0.015954
])

# === 3. Rileva marker ===
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()

corners, ids, _ = cv2.aruco.detectMarkers(img, dictionary, parameters=parameters)

# === 4. Stima della posa ===
marker_size = 0.36  # in metri (36 cm)

if ids is not None and len(corners) > 0:
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners, marker_size, camera_matrix, dist_coeffs
    )

    for i in range(len(ids)):
        rvec = rvecs[i][0]
        tvec = tvecs[i][0]

        # Distanza in metri → converti in cm
        distance_cm = np.linalg.norm(tvec) * 100
        x_cm, y_cm, z_cm = tvec * 100

        print(f"[INFO] Marker ID: {ids[i][0]}")
        print(f"[INFO] Distanza stimata: {distance_cm:.2f} cm")
        print(f"[INFO] Coordinate (X, Y, Z): ({x_cm:.2f}, {y_cm:.2f}, {z_cm:.2f}) cm")

        # Opzionale: disegna asse sul marker e salva immagine
        cv2.aruco.drawAxis(img, camera_matrix, dist_coeffs, rvec, tvec, 0.1)  # asse lungo 10 cm

    cv2.imwrite("output_with_pose.png", img)
    print("[INFO] Immagine salvata come output_with_pose.png")
else:
    print("[ERRORE] Nessun marker rilevato.")
