#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Twist
import time

def reset_robot():
    rospy.init_node('robot_resetter', anonymous=True)
    
    robot_name = 'pioneer3at' 
    cmd_topic = '/sim_p3at/cmd_vel' 
    
    
    start_x = -20.799
    start_y = 14.3
    start_z = 0.03
    start_yaw = 0.0
    

    
    
    pub = rospy.Publisher(cmd_topic, Twist, queue_size=10)
    stop_msg = Twist()
    stop_msg.linear.x = 0.0
    stop_msg.angular.z = 0.0
    
    rospy.loginfo("Motorlar durduruluyor...")
    for _ in range(10): 
        pub.publish(stop_msg)
        time.sleep(0.05)

    
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        state = ModelState()
        state.model_name = robot_name
        
        state.pose.position.x = start_x
        state.pose.position.y = start_y
        state.pose.position.z = start_z
        
        import tf.transformations
        q = tf.transformations.quaternion_from_euler(0, 0, start_yaw)
        state.pose.orientation.x = q[0]
        state.pose.orientation.y = q[1]
        state.pose.orientation.z = q[2]
        state.pose.orientation.w = q[3]
        
        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0
        
        resp = set_state(state)
        if resp.success:
            rospy.loginfo("Konum sifirlandi.")
        else:
            rospy.logwarn("Isinlanma basarisiz!")
            
    except rospy.ServiceException as e:
        print(f"Hata: {e}")

    
    rospy.loginfo("Yeni konumda sabitleniyor...")
    for _ in range(20): 
        pub.publish(stop_msg)
        time.sleep(0.05)
        
    rospy.loginfo("ROBOT HAZIR! Simdi navigator.py'i baslatabilirsin.")

if __name__ == '__main__':
    reset_robot()
