#!/usr/bin/env python
import argparse
import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped

class PoseRepublisher:
    def __init__(self, link_name, topic):
        self.link_name = link_name
        self.pub = rospy.Publisher(topic, PoseStamped, queue_size=1)

    def link_states_callback(self, data):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "fcu"  # TODO: Check!
        link_index = data.name.index(self.link_name)
        pose_stamped.pose = data.pose[link_index]
        self.pub.publish(pose_stamped)

def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description="Republish any link pose from Gazebo simulator")
    parser.add_argument("-link_name", type=str, default="mbzirc_1::base_link",
                        help="name of the link whose pose is wanted, as in gazebo/link_states")
    parser.add_argument('-topic', type=str, default="gazebo_pose_server",
                        help='name of the topic to republish pose')
    args, unknown = parser.parse_known_args()
    for arg in unknown:
        if arg[0] == '-':
            raise SyntaxWarning("Unexpected argument " + arg)

    rospy.init_node("gazebo_pose_server", anonymous=True)
    pose_republisher = PoseRepublisher(args.link_name, args.topic)
    rospy.Subscriber("gazebo/link_states", LinkStates, pose_republisher.link_states_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
