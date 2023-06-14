

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

previous_data = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

x = 0.0
y = 0.0
th = 0.0

last_time = rospy.Time.now()


def getData(data):
    global x
    global y
    global th
    global previous_data
    global last_time
    global odom_pub
    global odom_broadcaster

    
    current_time = rospy.Time.now()

    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()

    vx = previous_data.linear.x
    vy = previous_data.linear.y

    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt

    vth = previous_data.angular.z
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "robot",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "robot"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    previous_data = data

rospy.Subscriber("/dynamixel_workbench/cmd_vel", Twist, getData)
rospy.spin()
