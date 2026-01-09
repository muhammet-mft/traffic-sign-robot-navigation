#!/usr/bin/env python3

import rospy
import tf
from gazebo_msgs.msg import ModelStates
import math
import sys

class CoordinateViewerFinalRMSE:
    def __init__(self):
        rospy.init_node('coordinate_viewer_rmse', anonymous=True)
        
        self.tf_listener = tf.TransformListener()
        self.gazebo_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        
        
        self.robot_name = "pioneer3at"
        
        
        self.real_x = 0.0
        self.real_y = 0.0
        
        
        self.offset_x = None
        self.offset_y = None
        self.initialized = False
        self.gazebo_ready = False
        
        
        self.sum_sq_error = 0.0 
        self.sample_count = 0   

        print("\n--- FINAL KOORDINAT IZLEYICI (RMSE VE OFSET DUZELTMELI) ---")
        print("Kalibrasyon bekleniyor... Robot hareket edebilir.")
        print("-" * 95)
        
        print(f"{'SLAM X':<10} | {'SLAM Y':<10} || {'GERCEK X':<10} | {'GERCEK Y':<10} || {'ANLIK HATA':<10} || {'GENEL RMSE'}")
        print("-" * 95)

    def gazebo_callback(self, msg):
        try:
            if self.robot_name in msg.name:
                idx = msg.name.index(self.robot_name)
                self.real_x = msg.pose[idx].position.x
                self.real_y = msg.pose[idx].position.y
                self.gazebo_ready = True
        except ValueError:
            pass

    def run(self):
        rate = rospy.Rate(5) 
        rospy.sleep(1.0) 
        
        while not rospy.is_shutdown():
            try:
                
                (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                raw_slam_x = trans[0]
                raw_slam_y = trans[1]
                
                if self.gazebo_ready:
                    
                    if not self.initialized:
                        self.offset_x = self.real_x - raw_slam_x
                        self.offset_y = self.real_y - raw_slam_y
                        self.initialized = True
                        continue

                    
                    adj_slam_x = raw_slam_x + self.offset_x
                    adj_slam_y = raw_slam_y + self.offset_y
                    
                    
                    error = math.sqrt((adj_slam_x - self.real_x)**2 + (adj_slam_y - self.real_y)**2)
                    
                    
                    self.sum_sq_error += error ** 2
                    self.sample_count += 1
                    rmse = math.sqrt(self.sum_sq_error / self.sample_count)
                    
                    
                    sys.stdout.write(f"\r{adj_slam_x:8.3f}   | {adj_slam_y:8.3f}   || {self.real_x:8.3f}   | {self.real_y:8.3f}   || {error:8.3f} m   || \033[92m{rmse:8.3f} m\033[0m")
                    sys.stdout.flush()
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
            
            rate.sleep()

if __name__ == '__main__':
    try:
        viewer = CoordinateViewerFinalRMSE()
        viewer.run()
    except rospy.ROSInterruptException:
        pass
