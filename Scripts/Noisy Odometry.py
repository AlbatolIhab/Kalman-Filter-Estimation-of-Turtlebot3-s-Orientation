#!/ usr/bin/env python

#Importing required Messages
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import Float32MultiArray

#Defining the callback function
def getOdom(msg):
    #Robot orientation (theta)
    global yaw
    explicit_quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(explicit_quat) #converting from quaterion to euler angles 
    #print (yaw)
    
    #Angular Rotation (theta Dot)
    angRot=msg.twist.twist.angular.z
    #adding gaussian noise to the readings
    noisy_yaw=np.random.normal(yaw,0.25,1)[0] 
    noisy_angRot=np.random.normal(angRot,0.3,1)[0]
    #print (noisy_angRot)

    #Publishing the noisy readings
    noisy_states = []
    noisy_states.append(noisy_yaw)
    noisy_states.append(noisy_angRot)
    noisy_pub.publish(Float32MultiArray(data = noisy_states))


#Initiating a node
rospy.init_node('velocity', anonymous=True)
pub=rospy.Publisher('/cmd_vel', Twist ,queue_size=10)
sub=rospy.Subscriber('/odom', Odometry, getOdom)
noisy_pub=rospy.Publisher('/noisy_state', Float32MultiArray ,queue_size=10)

rate=rospy.Rate(10)
#rospy.spin()
#Publishing velocities to move the robot in circular path
move=Twist()
move.linear.x = 0.4
move.angular.z = 0.5
    
    
while not rospy.is_shutdown():
    pub.publish(move)
    #rospy.loginfo('move')
    rate.sleep()         
        