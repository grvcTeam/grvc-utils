#!/usr/bin/env python
# license removed for brevity
import rospy, tf
from geometry_msgs.msg import PoseStamped

pose = PoseStamped()

def callback(data):
    global pose
    pose = data


def talker():
    rospy.init_node('bridge', anonymous=True)

    pub = rospy.Publisher('mavros/mocap/pose', PoseStamped, queue_size=1)
    #pub = rospy.Publisher('mavros/vision_pose/pose', PoseStamped, queue_size=1)
    rospy.Subscriber("mocap_client/Pliego_apm_2/pose", PoseStamped, callback)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
