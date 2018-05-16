#!/usr/bin/env python
import argparse
import time
import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped

class PoseRepublisher:
    def __init__(self, link_name, topic, only_position):
        self.link_name = link_name
        self.pub = rospy.Publisher(topic, PoseStamped, queue_size=1)
        self.only_position = only_position

    def link_states_callback(self, data):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "fcu"  # TODO: Check!
        if self.link_name not in data.name:
            rospy.logwarn("Link %s not found", self.link_name)
            time.sleep(1)
            return
        link_index = data.name.index(self.link_name)
        pose_stamped.pose = data.pose[link_index]
        if self.only_position:
            pose_stamped.pose.orientation.x = 0
            pose_stamped.pose.orientation.y = 0
            pose_stamped.pose.orientation.z = 0
            pose_stamped.pose.orientation.w = 1
        self.pub.publish(pose_stamped)

def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description="Republish any link pose from Gazebo simulator")
    parser.add_argument("-link_name", type=str, default="mbzirc_1::base_link",
                        help="name of the link whose pose is wanted, as in gazebo/link_states")
    parser.add_argument('-topic', type=str, default="uav_1/mavros/vision_pose/pose",
                        help='name of the topic to republish pose')
    parser.add_argument('-only_position', action="store_true", default=False,
                        help='republish only position or also orientation')
    args, unknown = parser.parse_known_args()
    for arg in unknown:
        if arg[0] == '-':
            raise SyntaxWarning("Unexpected argument " + arg)

    rospy.init_node("gazebo_pose_server", anonymous=True)
    pose_republisher = PoseRepublisher(args.link_name, args.topic, args.only_position)
    rospy.Subscriber("gazebo/link_states", LinkStates, pose_republisher.link_states_callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()
