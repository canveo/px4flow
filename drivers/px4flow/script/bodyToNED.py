#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from px_comm.msg import OpticalFlow
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
from numpy import *

q = array([0,0,0,1])
v = array([0,0,0])
vx_arr = [0, 0, 0, 0, 0]
vy_arr = [0, 0, 0, 0, 0]
z_prev = 0
t_prev = 0
z_now = 0
t_now = 0

def imucallback(msg):
    global q
    q = array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

def toNED(msg):
    global vx_arr, vy_arr
    vx_arr.pop(0)
    vx_arr.append(msg.velocity_x)
    vy_arr.pop(0)
    vy_arr.append(msg.velocity_y)

    vx = 0.5*vx_arr[4]+0.2*vx_arr[3]+0.15*vx_arr[2]+0.1*vx_arr[1]+0.05*vx_arr[0]
    vy = 0.5*vy_arr[4]+0.2*vy_arr[3]+0.15*vy_arr[2]+0.1*vy_arr[1]+0.05*vy_arr[0]
    v_body = array([vx, vy, 0]) * 1.2

    global q
    [qx, qy, qz, qw] = [q[0], q[1], q[2], q[3]]
    Tned = array([[1-2*qy*qy-2*qz*qz,   2*qx*qy-2*qz*qw,   2*qx*qz+2*qy*qw],
                  [2*qx*qy + 2*qz*qw, 1-2*qx*qx-2*qz*qz,   2*qy*qz-2*qx*qw],
                  [2*qx*qz-2*qy*qw  , 2*qy*qz + 2*qx*qw, 1-2*qx*qx-2*qy*qy]])

    v = dot(Tned, v_body)
    
    global z_now, t_now, z_prev, t_prev
    z_now = msg.ground_distance
    t_now = msg.header.stamp.to_sec()
    if ((t_now - t_prev) < 1):
        vz = (z_now-z_prev)/(t_now-t_prev)
    else:
        vz = 0

    twist = TwistStamped()
    twist.header = Header()
    twist.header.frame_id = "ned"
    twist.header.stamp = rospy.Time.now()
    twist.twist.linear.x = v[0]
    twist.twist.linear.y = v[1]
    twist.twist.linear.z = -vz
    pub.publish(twist)

    z_prev = z_now
    t_prev = t_now

if __name__ == '__main__':
    rospy.init_node('bodyToNED')
    rospy.Subscriber('/mavros/imu/data', Imu, imucallback)
    pub = rospy.Publisher('velocity', TwistStamped, queue_size=0)	
#    rate = rospy.Rate(1)
#    for i in xrange(5):
#        rate.sleep()

    rospy.Subscriber('/px4flow/opt_flow', OpticalFlow, toNED)
    rospy.spin()


