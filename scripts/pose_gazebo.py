import rospy
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import math

def callback(data):

    pose_x = data.pose.pose.position.x
    pose_y = data.pose.pose.position.y

    orientation = data.pose.pose.orientation

    (roll, pitch,theta) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    print("THETA : " + str(math.degrees(theta)))
    print("X : " + str(pose_x))
    print("Y : " + str(pose_y))
    
def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/odom", Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()