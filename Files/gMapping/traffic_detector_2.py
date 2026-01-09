#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

class TrafficSignDetector:
    def __init__(self):
        rospy.init_node('traffic_sign_detector', anonymous=True)
        rospy.loginfo("--- TRAFIK DEDEKTORU V9.0: STANDART GORUS (Z=1) ---")

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/rgb_camera/image_raw", Image, self.image_callback)
        self.sign_pub = rospy.Publisher("/traffic_sign", String, queue_size=1)
        
        # Dosya yolu (Senin yolun)
        base_path = "/home/drcoriolis/catkin_ws/src/amr-ros-config/gazebo/models"
        self.templates = {}
        sign_names = ['LEFT', 'RIGHT', 'STRAIGHT', 'PARK']
        
        for name in sign_names:
            try:
                # Z=1 oldugu icin jpg/jpeg uzantisina dikkat et
                path = f"{base_path}/sign_{name.lower()}/materials/textures/{name.lower()}.jpeg"
                img = cv2.imread(path, 0)
                if img is not None:
                    # Uzaktan net gorsun diye 45x45 yaptik
                    self.templates[name] = cv2.resize(img, (45, 45))
                    rospy.loginfo(f"[OK] {name} sablonu yuklendi.")
            except Exception as e:
                rospy.logerr(f"Hata: {e}")

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        except Exception:
            return

        detected_sign = "NONE"
        max_val_all = 0
        
        for name, template in list(self.templates.items()):
            res = cv2.matchTemplate(gray_image, template, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, max_loc = cv2.minMaxLoc(res)
            
            # Guven Esigi: Z=1'de levhalar daha net cikar, 0.65 idealdir.
            if max_val > 0.65:
                if max_val > max_val_all:
                    # --- FILTRELER KALDIRILDI ---
                    # Artik Z=1 oldugu icin yukseklik kisitlamasi yok.
                    # Sadece ekranda gormesi yeterli.
                    
                    max_val_all = max_val
                    detected_sign = name
                    
                    # Gorsellestirme
                    h, w = template.shape
                    top_left = max_loc
                    bottom_right = (top_left[0] + w, top_left[1] + h)
                    cv2.rectangle(cv_image, top_left, bottom_right, (0, 255, 0), 2)
                    cv2.putText(cv_image, f"{name} ({max_val:.2f})", (top_left[0], top_left[1]-5), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if detected_sign != "NONE":
            self.sign_pub.publish(detected_sign)
            
        cv2.imshow("Robot Gozu (Z=1)", cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    TrafficSignDetector()
    rospy.spin()
