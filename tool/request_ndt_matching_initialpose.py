#!/usr/bin/env python
# coding: utf-8

import rospy
from lidar_localizer.srv import Initialpose
from geometry_msgs.msg import PoseWithCovarianceStamped


if __name__ == '__main__':
    rospy.init_node('gnss_poser_request_ndt_matching_initialpose', anonymous=True)

    gnss_pose = rospy.wait_for_message('/gnss_pose_cov', PoseWithCovarianceStamped)

    rospy.wait_for_service('ndt_matching_initialpose')
    try:
        client = rospy.ServiceProxy('ndt_matching_initialpose', Initialpose)
        req = Initialpose()
        req.pose = gnss_pose
        res = client(req.pose)
        rospy.loginfo('ndt_matching response: {}'.format(res.result))
    except rospy.ServiceException as e:
        rospy.loginfo(e)
