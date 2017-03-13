#!/usr/bin/env python
# license removed for brevity
import rospy
from px_comm.msg import OpticalFlow
from viconxbee.msg import viconPoseMsg

filename = 'ground_truth.txt'
vx=0
vy=0
z=0

def flowcallback(data):
    t = data.header.stamp
    z_px  = data.ground_distance
    vx_px = data.velocity_x
    vy_px = data.velocity_y
    global vx,vy,z

    f = open(filename,'a')
    f.write('%d %d %f %f %f %f %f %f \n' % (t.secs, t.nsecs, vx, vy, z, vx_px, vy_px, z_px,))
    f.close()

def viconcallback(data):
    global vx,vy,z
    vx = data.dx
    vy = data.dy
    z  = data.z

def listener():

    rospy.init_node('totxt', anonymous=True)

    rospy.Subscriber("/px4flow/opt_flow", OpticalFlow, flowcallback)
    rospy.Subscriber("/viconxbee/viconPoseTopic", viconPoseMsg, viconcallback)

    rospy.spin()

if __name__ == '__main__':
    listener()
    print "*******"
