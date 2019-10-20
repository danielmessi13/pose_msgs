#!/usr/bin/env python3

from pose_msgs.msg import TransformStampedCertainty
import rospy
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
import math

l_pose_x = 0
l_pose_y = 0
pose_x = 0
pose_y = 0
theta = 0

def callback(data):

    # print("ID: " + data.transforms[0].child_frame_id)
    # print("X: " + str(data.transforms[0].transform.translation.x))
    # print("Y: " + str(data.transforms[0].transform.translation.y))
    # print("W (Theta): " + str(data.transforms[0].transform.rotation.w))

    global l_pose_x 
    global pose_x
    global l_pose_y
    global pose_y
    global theta


    if data.transforms[0].child_frame_id == "marker_id0":
        l_pose_x = data.transforms[0].transform.translation.x
        l_pose_y = data.transforms[0].transform.translation.y

        orientation = data.transforms[0].transform.rotation
        (roll, pitch, theta) = quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)


        if l_pose_y > 0:
            l_pose_y = (-1 * l_pose_y)
        else:
            l_pose_y = abs(l_pose_y)

    else:
        pose_x = data.transforms[0].transform.translation.x
        pose_y = data.transforms[0].transform.translation.y

        if pose_y > 0:
            pose_y = (-1 * pose_y)

        else:
            pose_y = abs(pose_y)

    K_ANGLE = 20

    pub = rospy.Publisher('front_back', Float64, queue_size=512)

    pub2 = rospy.Publisher('left_right', Float64, queue_size=512)

    distancia = abs(math.sqrt((l_pose_x - pose_x) ** 2 + (l_pose_y - pose_y) ** 2))

    arc_to_move = math.atan2(l_pose_y - pose_y, l_pose_x - pose_x)

    angle = (arc_to_move - theta) * K_ANGLE
    
    # print("Arco a se mover: " + str(arc_to_move))
    # print("Theta: " + str(theta))
    # print("Angulo: " + str(angle))

    # print(theta)
    
    pub.publish(Float64(distancia))
    pub2.publish(Float64(angle))


def listener_joy():

    l_pose_x = 0
    l_pose_y = 0
    pose_x = 0
    pose_y = 0

    rospy.init_node('pose_aruco', anonymous=True)
    rospy.Subscriber("/tf", TFMessage, callback)
    rospy.spin()
    

def quaternion_to_euler(x, y, z, w):

        import math
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.atan2(t3, t4)

        return X, Y, Z

if __name__ == '__main__':
    try:
        listener_joy()
    except rospy.ROSInterruptException:
        pass
