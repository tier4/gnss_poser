#!/usr/bin/env python
# coding: utf-8

from argparse import ArgumentParser

import rospy
from lidar_localizer.srv import Initialpose
from geometry_msgs.msg import PoseWithCovarianceStamped


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('-T', '--topic', type=str, default='/gnss_pose_cov', help='topic')
    parser.add_argument('-S', '--service', type=str, default='ndt_matching_initialpose', help='service')
    args = parser.parse_args()

    rospy.init_node('gnss_poser_request_ndt_matching_initialpose', anonymous=True)

    gnss_pose = rospy.wait_for_message(args.topic, PoseWithCovarianceStamped)

    rospy.wait_for_service(args.service)
    try:
        client = rospy.ServiceProxy(args.service, Initialpose)
        req = Initialpose()
        req.pose = gnss_pose
        res = client(req.pose)
        rospy.loginfo('ndt_matching response: {}'.format(res.result))
    except rospy.ServiceException as e:
        rospy.loginfo(e)
