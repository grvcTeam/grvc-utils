#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import PoseStamped

def position_distance(pose_a, pose_b):
    diff_x = pose_a.position.x - pose_b.position.x
    diff_y = pose_a.position.y - pose_b.position.y
    diff_z = pose_a.position.z - pose_b.position.z
    return math.sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z)

def vision_timer_callback(event):
    rospy.logwarn("Vision pose timeout!")

def local_timer_callback(event):
    rospy.logwarn("Local pose timeout!")

class PoseWatchdog:
    def __init__(self):
        self.vision_to_local_th = 0.5
        self.repeat_th = 3
        self.timeout_th = 3
        self.repeat_count = 0
        self.vision_pose = PoseStamped()
        self.local_pose  = PoseStamped()
        self.vision_timer = rospy.Timer(rospy.Duration(self.timeout_th), vision_timer_callback)
        self.local_timer  = rospy.Timer(rospy.Duration(self.timeout_th), local_timer_callback)

    def check_distance(self):
        distance = position_distance(self.vision_pose.pose, self.local_pose.pose)
        if distance > self.vision_to_local_th:
            rospy.logwarn("Vision to local pose distance =  %f", distance)

    def vision_pose_callback(self, data):
        frozen_distance = position_distance(self.vision_pose.pose, data.pose)
        if frozen_distance == 0:
            self.repeat_count += 1
        else:
            self.repeat_count = 0
 
        if self.repeat_count > self.repeat_th:
            rospy.logwarn("Vision pose is frozen!")

        self.vision_pose = data
        self.check_distance()

        self.vision_timer.shutdown()
        self.vision_timer = rospy.Timer(rospy.Duration(self.timeout_th), vision_timer_callback)

    def local_pose_callback(self, data):
        self.local_pose = data
        self.check_distance()
        self.local_timer.shutdown()
        self.local_timer = rospy.Timer(rospy.Duration(self.timeout_th), local_timer_callback)

def main():
    rospy.init_node("pose_watchdog", anonymous=True)
    pose_watchdog = PoseWatchdog()
    rospy.Subscriber("uav_1/mavros/vision_pose/pose", PoseStamped, pose_watchdog.vision_pose_callback, queue_size=1)
    rospy.Subscriber("uav_1/mavros/local_position/pose", PoseStamped, pose_watchdog.local_pose_callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()
