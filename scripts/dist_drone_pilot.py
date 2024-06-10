#!/usr/bin/python3
import rospy
import time
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Float64

distance = 0

def callback(data):
    global distance
    pos_drone = data.pose[data.name.index("aerotrain::base_link")].position
    pos_cam = data.pose[data.name.index("aerotrain::camera_pilot")].position
    distance = ((pos_drone.x - pos_cam.x)**2 + (pos_drone.y - pos_cam.y)**2 + (pos_drone.z - pos_cam.z)**2)**0.5
    
def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/gazebo/link_states", LinkStates, callback)
    pub = rospy.Publisher('/camera_pilot/drone_distance', Float64, queue_size=1)
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        if distance != 0:
            pub.publish(distance)
        rate.sleep()

if __name__ == '__main__':
    time.sleep(2)
    listener()