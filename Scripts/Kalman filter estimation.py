#Importing required Messages
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

#Defining the callback function when the noisy readings are recieved
def odometry_recieved(msg):
    global A, P0, Q, R, C, I, noisy_yaw, noisy_angRot
    noisy_yaw = msg.data[0]
    noisy_angRot = msg.data[1]
    #Kalman Filter
    #PREDICT
    turtle_states = np.array([noisy_yaw, noisy_angRot])
    P = (A.dot(P0).dot((A.T))) + Q #Prediciton uncertainty
    
    #UPDATE:
    G = P.dot((C.T)).dot(np.linalg.inv((C.dot(P).dot((C.T)))+R)) # Kalman Gain
    turtle_states = turtle_states + G.dot((robot_orientation - C.dot(turtle_states))) # estimate correction 
    P = (I - G.dot(C)).dot(P) #corrrection of uncertainty 

    #filtered data from kalman filter
    filtered_states = []
    filtered_states.append(turtle_states[0])
    filtered_states.append(turtle_states[1])
    filtered_pub.publish(Float32MultiArray(data = filtered_states))

#Defining the callback function to recieve imu readings
def imu_recieved(msg):
    global robot_orientation
    quat_orientation=[msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
    roll, pitch, imu_yaw = euler_from_quaternion(quat_orientation)
    angVel_imu=msg.angular_velocity.z
    robot_orientation=np.array([imu_yaw, angVel_imu])

def main():
    #Initiating a node
    rospy.init_node('kalman_filter', anonymous=True)
    global A, P0, Q, R, C, I,filtered_pub

    #Defining system, process, and measurment matrices 
    I = np.eye(2)
    dt = 0.01
    sigma_th = 0.3
    sigma_yaw=0.25
    sigma_yawdot=0.3
    A =np.array([[1, dt],
                 [0, 1]])

    P0 =np.array([[500, 0],
                  [0, 500]])

    Q = np.array([[sigma_yaw**2, 0]
                ,[0, sigma_yawdot**2]])
    x0 = np.array([[0, 0]])

    R = np.array([[sigma_th**2,0],
                [0,sigma_th**2]])

    C = np.array([[1, 0],
                [0,1]])

    rospy.Subscriber('/imu', Imu, imu_recieved)
    rospy.Subscriber('/noisy_state',Float32MultiArray, odometry_recieved)
    filtered_pub = rospy.Publisher('/filtered_heading', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == "__main__":
    main()