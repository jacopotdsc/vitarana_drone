#!/usr/bin/env python3

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy

class image_proc():

    # Initialise everything
    def __init__(self):
        rospy.init_node('qr_detection') 

        ###### ----------- Ros topics ---------- ######
        self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber("/edrone/camera/camera_info", CameraInfo, self.camera_info_callback)
        self.qr_pub = rospy.Publisher('/qr_position', Point,queue_size=1)
        self.rate = rospy.Rate(10)

        ###### ----------- Meta Data ---------- ######    
        self.marker_size = 0.36

        ###### ----------- Data ---------- ######
        self.img = np.empty([])
        self.bridge = CvBridge()
        self.qcd = cv2.QRCodeDetector()

        rospy.spin()

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape((3, 3))
        self.dist_coeffs = np.array(msg.D)

    def estimate_marker_pose(self, img):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters_create()

        corners, ids, _ = cv2.aruco.detectMarkers(img, dictionary, parameters=parameters)

        if ids is not None and len(corners) > 0:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )

            for i in range(len(ids)):
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                distance_cm = np.linalg.norm(tvec) * 100
                x_cm, y_cm, z_cm = tvec * 100

                print(f"[INFO] Marker ID: {ids[i][0]}")
                print(f"[INFO] Distanza stimata: {distance_cm:.2f} cm")
                print(f"[INFO] Coordinate (X, Y, Z): ({x_cm:.2f}, {y_cm:.2f}, {z_cm:.2f}) cm")

                # Opzionale: disegna asse sul marker e salva immagine
                cv2.aruco.drawAxis(img, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)  # asse lungo 10 cm

            #cv2.imwrite("output_with_pose.png", img)
            #print("[INFO] Immagine salvata come output_with_pose.png")
        else:
            print("[ERRORE] Nessun marker rilevato.")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            rospy.logerr("CV Bridge Error: %s", e)
            return
        
        cv2.imshow("Camera View", frame)
        #cv2.waitKey(1)
            

if __name__ == '__main__':
    try:
        image_proc_obj = image_proc()
    except rospy.ROSInterruptException:
        pass
