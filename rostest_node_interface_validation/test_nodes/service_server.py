#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (c) 2020, Anthony Remazeilles
# All rights reserved.
# Author: Anthony Remazeilles <anthony.remazeilles@tecnalia.com>
# inspired from https://github.com/ros/ros_comm/blob/noetic-devel/tools/rostest/test_nodes/service_server.py

import rospy
from std_srvs.srv import Empty, EmptyResponse
from std_srvs.srv import SetBool, SetBoolResponse
from std_srvs.srv import Trigger, TriggerResponse


def empty_cb(req):
    return EmptyResponse()

def set_bool_cb(req):
    response = SetBoolResponse()
    response.success = True
    response.message = str(req.data)
    return response

def trigger_cb(req):
    return TriggerResponse()

def trigger2_cb(req):
    response = TriggerResponse()
    response.success = True
    response.message = 'well done!'
    return response

def main():
    rospy.init_node('service_server')

    empty_service = rospy.Service('empty', Empty, empty_cb)
    set_bool_service = rospy.Service('set_bool', SetBool, set_bool_cb)
    trigger_service = rospy.Service('trigger', Trigger, trigger_cb)
    trigger2_service = rospy.Service('trigger_spec', Trigger, trigger2_cb)

    rospy.spin()


if __name__ == '__main__':
    main()
