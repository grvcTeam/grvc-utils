#!/usr/bin/env python
#
# Original code: https://github.com/grvcTeam/grvc-ual/blob/master/uav_abstraction_layer/scripts/track_waypoints.py

import argparse, sys, yaml, rospy, rospkg

from dji_sdk_demo.srv import *
from geographic_msgs.msg import GeoPoint

def track_waypoints():
    # Parse arguments
    parser = argparse.ArgumentParser(description='Track waypoints defined in a yaml file')
    parser.add_argument('-plan_package', type=str, default='handy_tools',
                        help='Name of the package where plan to track is stored')
    parser.add_argument('-plan_file', type=str, default='wp_default.yaml',
                        help='Name of the file inside plan_package/plans')
    parser.add_argument('-wait_for', type=str, default='path',
                        help='Wait for human response: [none]/[path]/[wp]')
    args, unknown = parser.parse_known_args()
    # utils.check_unknown_args(unknown)

    rospy.init_node('waypoint_tracker')

    file_name = args.plan_file
    # Autocomplete file extension
    if not file_name.endswith('.yaml'):
        file_name = file_name + '.yaml'

    file_url = rospkg.RosPack().get_path(args.plan_package) + '/plans/' + file_name
    with open(file_url, 'r') as wp_file:
        wp_data = yaml.load(wp_file)

    if 'frame_id' not in wp_data:
        rospy.logerr("Must specify frame_id in waypoints file")  # TODO: default to ''?
        return

    wp_list = []

    for wp_id in range(1000):
    # for wp_id in range(int(n_wp)):
        if 'wp_' + str(wp_id) in wp_data:
            print 'WP '+ str(wp_id+1)
            wp_raw = wp_data['wp_' + str(wp_id)]

            wp = GeoPoint()
            wp.latitude = wp_raw[0]
            wp.longitude = wp_raw[1]
            wp.altitude = wp_raw[2]

            print wp

            wp_list.append(wp)

    rospy.wait_for_service('/inspector/run_mission')

    try:
        run_mission = rospy.ServiceProxy('/inspector/run_mission', RunMission)
         # TODO: Check we're flying!
        print "Ready to track " + str(len(wp_list)) + " waypoints from " + file_url
        res = run_mission(wp_list)
        return res.ack
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    track_waypoints()
