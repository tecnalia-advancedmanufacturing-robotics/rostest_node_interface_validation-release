#!/usr/bin/env python
"""
Perform a unitest on a node acting as a service server.

Verify that, given a service request, the answer provided is the one expected.

Test files should set the following parameters:

<test test-name="test_service" pkg="rostest_node_interface_validation" type="test_service" >
    <rosparam>
        calls:
            - name: service name
              input: input parameter of the message, in dictionary format
              output: output parameter of the message, in dictionary format
    </rosparam>
</test>

Several calls can be added.
If the input or the output should be empty, place None.

"""

__author__ = "Anthony Remazeilles"
__email__ = "anthony.remazeilles@tecnalia.com"
__copyright__ = "Copyright (C) 2020 Tecnalia Research and Innovation"
__licence__ = "Apache 2.0"

import sys
import time
import unittest
import rospy
import rosservice
import rostest
from rospy_message_converter import message_converter

CLASSNAME = 'servicetest'


class ServiceTest(unittest.TestCase):
    def __init__(self, *args):
        super(ServiceTest, self).__init__(*args)
        rospy.init_node(CLASSNAME)

        self.calls = list()

        try:
            calls = rospy.get_param('~calls')

            for call in calls:

                keys = ['name', 'input', 'output']
                for item in keys:
                    if item not in call:
                        self.fail("{} field required, but not specified in {}".format(item, call))

                srv_data = dict()

                srv_data['name'] = call['name']
                srv_data['input'] = call['input']
                srv_data['output'] = call['output']

                if srv_data['input'] == 'None':
                    rospy.logwarn('None input converted to empty input')
                    srv_data['input'] = dict()
                if srv_data['output'] == 'None':
                    rospy.logwarn('None output converted to empty output')
                    srv_data['output'] = dict()

                self.calls.append(srv_data)
        except KeyError as err:
            msg_err = "service_test not initialized properly"
            msg_err += " Parameter [%s] not set." % (str(err))
            msg_err += " Caller ID: [%s] Resolved name: [%s]" % (
                rospy.get_caller_id(),
                rospy.resolve_name(err.args[0]))
            self.fail(msg_err)

    def setUp(self):
        # warn on /use_sim_time is true
        use_sim_time = rospy.get_param('/use_sim_time', False)
        self.t_start = time.time()
        while not rospy.is_shutdown() and \
                use_sim_time and (rospy.Time.now() == rospy.Time(0)):
            rospy.logwarn_throttle(
                1, '/use_sim_time is specified and rostime is 0, /clock is published?')
            if time.time() - self.t_start > 10:
                self.fail('Timed out (10s) of /clock publication.')
            # must use time.sleep because /clock isn't yet published, so rospy.sleep hangs.
            time.sleep(0.1)

    def test_advertise_service(self):
        """Test services are advertised"""
        t_start = self.t_start
        s_name_set = set([ item['name'] for item in self.calls])
        t_timeout_max = 10.0

        while not rospy.is_shutdown():
            t_now = time.time()
            t_elapsed = t_now - t_start
            if not s_name_set:
                break
            if t_elapsed > t_timeout_max:
                break

            for s_name in rosservice.get_service_list():
                if s_name in s_name_set:
                    s_name_set.remove(s_name)
            time.sleep(0.05)

        # All services should have been detected
        assert not s_name_set, \
            'services [%s] not advertized on time' % (s_name_set)

        rospy.loginfo("All services advertized on time")

    def test_service_call(self):

        for item in self.calls:
            rospy.loginfo("Testing service {} with input parameters {}".format(
                item['name'],
                item['input']))
            self._test_service(item['name'], item['input'], item['output'])
            rospy.loginfo("So far so good")

    def _test_service(self, srv_name, srv_input, srv_output):
        self.assert_(srv_name)
        all_services = rosservice.get_service_list()
        self.assertIn(srv_name, all_services)
        srv_class = rosservice.get_service_class_by_name(srv_name)

        try:
            srv_proxy = rospy.ServiceProxy(srv_name, srv_class)
        except KeyError as err:
            msg_err = "Service proxy could not be created"
            self.fail(msg_err)

        try:
            if srv_input:
                srv_resp = srv_proxy(**srv_input)
            else:
                srv_resp = srv_proxy()

        except (genpy.SerializationError, rospy.ROSException), err:
            msg_err = "Service proxy error: {}".format(err.message)
            self.fail(msg_err)
        srv_dic = message_converter.convert_ros_message_to_dictionary(srv_resp)

        self.assertDictEqual(srv_dic, srv_output)


def main():
    try:
        rostest.run('rostest', CLASSNAME, ServiceTest, sys.argv)
    except KeyboardInterrupt:
        pass
    print("{} exiting".format(CLASSNAME))
