#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def main():
    # ROS node başlat
    rospy.init_node('move_robot', anonymous=True)
    
    # /cmd_vel topic'ine publish edecek publisher
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    # Loop hızı
    rate = rospy.Rate(10)  # 10 Hz

    # Twist mesajını oluştur
    twist = Twist()
    twist.linear.x = 1.0    # İleri hız 1 m/s
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.5   # Dönme hızı 0.5 rad/s

    # ROS çalışırken sürekli publish et
    while not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

