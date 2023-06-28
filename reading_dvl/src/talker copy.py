#!/usr/bin/env python

import rospy
import libsbp
from libsbp import sbp_msg
from libsbp.system import sbp_state, sbp_startup, sbp_piksi, sbp_gps
from sensor_msgs.msg import NavSatFix

def callback(msg):
    # Extract GPS data from the SBP message
    latitude = msg.lat
    longitude = msg.lon
    altitude = msg.height

    # Create a NavSatFix message
    navsat_fix = NavSatFix()
    navsat_fix.header.stamp = rospy.Time.now()
    navsat_fix.header.frame_id = 'gps'
    navsat_fix.latitude = latitude
    navsat_fix.longitude = longitude
    navsat_fix.altitude = altitude

    # Publish the NavSatFix message
    pub.publish(navsat_fix)

def swift_node():
    rospy.init_node('swift_navigation_node', anonymous=True)
    rospy.Subscriber('/sbp/pos_llh', sbp_gps.MsgPosLLH, callback)
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('gps/fix', NavSatFix, queue_size=10)
    swift_node()
