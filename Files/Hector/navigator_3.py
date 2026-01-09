#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class RobotNavigator:
    def __init__(self):
        rospy.init_node('robot_navigator', anonymous=True)
        
        rospy.loginfo("--------------------------------------------")
        rospy.loginfo(" NAVIGATOR V13: STRAIGHT OVERRIDE + PARK 1.4m")
        rospy.loginfo("--------------------------------------------")

        self.cmd_pub = rospy.Publisher('/sim_p3at/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/front_laser/scan', LaserScan, self.scan_callback)
        self.sign_sub = rospy.Subscriber('/traffic_sign', String, self.sign_callback)

        self.current_sign = "NONE"      
        self.stored_command = "NONE"    
        self.lidar_data = None
        self.is_turning = False
        self.parking_mode = False # Park modu bayragi eklendi
        self.last_log_state = ""
        
        # Baslangic zamani
        self.start_time = rospy.Time.now().to_sec()

    def sign_callback(self, msg):
        self.current_sign = msg.data
        
        # --- DEGISTIRILEN KISIM: PARK MANTIGI ---
        if self.current_sign == "PARK":
            # Hemen durmak yerine park modunu aktif et
            if not self.parking_mode:
                self.parking_mode = True
                self.stored_command = "PARK"
                rospy.logwarn(">>> PARK LEVHASI GORULDU: 1.4m mesafeye ilerleniyor... <<<")
        
        elif self.current_sign != "NONE" and not self.is_turning and not self.parking_mode:
            # Eger tabela degisirse hafizayi guncelle
            if self.stored_command != self.current_sign:
                self.stored_command = self.current_sign
                rospy.logwarn(f"!!! [HAFIZA] YENI KOMUT: {self.stored_command} !!!")

    def scan_callback(self, msg):
        ranges = list(msg.ranges)
        self.lidar_data = [x if (x > 0.1 and x != float('inf')) else 10.0 for x in ranges]

    def log_once(self, msg):
        if self.last_log_state != msg:
            rospy.loginfo(msg)
            self.last_log_state = msg

    def move_robot(self):
        rospy.sleep(1.0)
        self.start_time = rospy.Time.now().to_sec()
        
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            if self.lidar_data is None: continue
            if self.is_turning: 
                rate.sleep()
                continue

            current_time = rospy.Time.now().to_sec()
            
            # --- LIDAR ANALIZI ---
            num = len(self.lidar_data)
            mid = num // 2
            
            front_wide = min(self.lidar_data[mid-30 : mid+30]) 
            front_right = min(self.lidar_data[(num//4)-25 : (num//4)+25])
            front_left  = min(self.lidar_data[(num*3//4)-25 : (num*3//4)+25])
            right_dist = min(self.lidar_data[0:45]) 
            left_dist  = min(self.lidar_data[-45:]) 

            twist = Twist()

            # --- YENI EKLENEN PARK KONTROLU (En basa eklendi) ---
            if self.parking_mode:
                # 1.4 metre mesafe kontrolu
                if front_wide <= 1.4:
                    rospy.logwarn(f">>> HEDEF MESAFE ({front_wide:.2f}m) ULASILDI. PARK EDILDI. <<<")
                    self.stop_robot()
                    rospy.signal_shutdown("Park Edildi")
                    break
                else:
                    # Yavasca ilerle
                    twist.linear.x = 0.15
                    twist.angular.z = 0.0
                    self.cmd_pub.publish(twist)
                    self.log_once(f"Park yerine yanlasiliyor... Mesafe: {front_wide:.2f}m")
                    continue
            # ----------------------------------------------------

            # --- 0. SOFT START ---
            if (current_time - self.start_time) < 2.0:
                twist.linear.x = 0.2
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                continue

            # --- 0.5. ADAPTIF HIZ KONTROLU ---
            closest_obj = min(front_wide, right_dist, left_dist, front_right, front_left)
            calc_speed = 0.15 + (closest_obj * 0.20)
            
            if calc_speed > 0.55: target_speed = 0.55
            elif calc_speed < 0.15: target_speed = 0.15
            else: target_speed = calc_speed

            # --- 1. FAIL-SAFE ---
            if self.stored_command == "LEFT" and front_wide < 0.75:
                self.perform_turn("LEFT")
                continue 
            elif self.stored_command == "RIGHT" and front_wide < 0.75:
                self.perform_turn("RIGHT")
                continue

            # --- 2. ACIL FREN ---
            if front_wide < 0.50:
                self.log_once(f"!!! ACIL DURUM ({front_wide:.2f}m) !!!")
                twist.linear.x = -0.15
                self.cmd_pub.publish(twist)
                continue
            
            # --- 3. KAVSAK KONTROLU (Sadece LEFT ve RIGHT icin calisir) ---
            should_turn = False
            turn_dir = "NONE"

            if self.stored_command == "LEFT":
                if left_dist > 1.5 and front_wide < 1.4:
                    should_turn = True
                    turn_dir = "LEFT"
                else:
                    self.log_once(f"Sol Bekleniyor... (L:{left_dist:.1f}m)")

            elif self.stored_command == "RIGHT":
                if right_dist > 1.5 and front_wide < 1.4:
                    should_turn = True
                    turn_dir = "RIGHT"
                else:
                    self.log_once(f"Sag Bekleniyor... (R:{right_dist:.1f}m)")

            if should_turn:
                rospy.loginfo(f"Kavsak KILIDI Acildi. {turn_dir} Donuluyor.")
                self.perform_turn(turn_dir)
                continue 

            # --- 4. AKTIF ENGELDEN KACIS ---
            if (front_wide < 0.90) or (front_right < 0.75) or (front_left < 0.75):
                self.log_once("Engel Manevrasi (Yavas).")
                twist.linear.x = 0.10 
                if (left_dist + front_left) > (right_dist + front_right): 
                    twist.angular.z = 0.50 
                else: 
                    twist.angular.z = -0.50
                self.cmd_pub.publish(twist)
                continue

            # ================================================================
            # *** YENI EKLEME: STRAIGHT OVERRIDE MODU ***
            # Eger tabela STRAIGHT ise, asagidaki duvar takibini (bolum 5) iptal et.
            # ================================================================
            if self.stored_command == "STRAIGHT":
                twist.linear.x = target_speed # Hiz normal devam etsin
                
                # Sadece duvara carpmasini onleyecek minimal refleks (Duvar takibi degil, carpma onleyici)
                if right_dist < 0.50:
                    twist.angular.z = 0.2 # Cok hafif sola kac
                elif left_dist < 0.50:
                    twist.angular.z = -0.2 # Cok hafif saga kac
                else:
                    twist.angular.z = 0.0 # DUMDUZ GIT (Kurallari yik)
                
                self.log_once(f">>> STRAIGHT MODU: Duz Gidiliyor (Hiz: {target_speed:.2f}) <<<")
                self.cmd_pub.publish(twist)
                continue # Dongunun basina don (Bolum 5'i isleme)
            # ================================================================

            # --- 5. DUVAR TAKIBI & YUMUSAK ORTALAMA (Varsayilan Mod) ---
            twist.linear.x = target_speed 
            
            align_limit = 2.5 
            c_left = min(left_dist, align_limit)
            c_right = min(right_dist, align_limit)
            error = c_left - c_right 

            if right_dist < 0.60:
                twist.angular.z = 0.35 
                self.log_once("Sag Duvar Tehlikesi -> Sola")
            elif left_dist < 0.60:
                twist.angular.z = -0.35 
                self.log_once("Sol Duvar Tehlikesi -> Saga")
            else:
                base_gain = 0.25 
                correction = error * base_gain
                
                if abs(correction) < 0.05: correction = 0
                if correction > 0.4: correction = 0.4
                if correction < -0.4: correction = -0.4

                twist.angular.z = correction

                if abs(error) > 0.15:
                    dir_s = "Saga" if error > 0 else "Sola"
                    self.log_once(f"Ortalaniyor: {dir_s} (Fark:{abs(error):.2f}m)")
                else:
                    self.log_once(f"Stabil (Hiz: {target_speed:.2f})")

            self.cmd_pub.publish(twist)
            rate.sleep()

    def perform_turn(self, direction):
        self.is_turning = True
        twist = Twist()
        twist.linear.x = 0.25 
        twist.angular.z = 0.55 if direction == "LEFT" else -0.55
        
        t_duration = 3.0 
        t0 = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - t0) < t_duration:
            self.cmd_pub.publish(twist)
            rospy.sleep(0.1)
            
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        
        self.is_turning = False
        self.stored_command = "NONE" 
        rospy.loginfo(f"{direction} Donusu Tamamlandi. Hafiza Temizlendi.")
        rospy.sleep(0.5)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

if __name__ == '__main__':
    try:
        RobotNavigator()
        nav = RobotNavigator() 
        nav.move_robot()
    except rospy.ROSInterruptException:
        pass
