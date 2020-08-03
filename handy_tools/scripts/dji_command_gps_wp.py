#!/usr/bin/env python
import rospy

from dji_sdk_demo.srv import *
from geographic_msgs.msg import GeoPoint

def run_mission_srv(wp_list):
    rospy.wait_for_service('/inspector/run_mission')
    try:
        run_mission = rospy.ServiceProxy('/inspector/run_mission', RunMission)
        res = run_mission(wp_list)
        return res.ack
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    wp_list = []

    n_wp = raw_input('Num wp?')
    for i in range(int(n_wp)):
        print 'WP '+ str(i+1)

        wp = GeoPoint()
        wp.latitude = float(raw_input('Lat: '))
        wp.longitude = float(raw_input('Lon: '))
        wp.altitude = float(raw_input('Alt: '))

        wp_list.append(wp)
    print wp_list
    print "ACK: " + str(run_mission_srv(wp_list))
