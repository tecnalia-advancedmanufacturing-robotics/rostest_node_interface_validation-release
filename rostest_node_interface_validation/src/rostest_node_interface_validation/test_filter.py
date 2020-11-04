#!/usr/bin/env python
"""
Perform a unittest on a ROS message filter-like behavior.

Verify that a component, when receiving a specific message,
generates the expected topic.

Example of test file using this component:

    <test test-name="filter_test" pkg="rostest_node_interface_validation" type="test_filter">
        <rosparam>
            filters:
                - topic_in: topic to which the filter node is subscribed to
                  topic_out: topic to which the filter node will pubish filter output
                  msg_in: message to publish on topic_in, in python dictionary format
                  msg_out: message to be received on topic_out, in python dictionary format
                  timeout: limit time [s] for receiving the filter output
        </rosparam>
    </test>

Several filter calls can be added.
If the input or the output message should be empty, place None.
"""

__author__ = "Anthony Remazeilles"
__email__ = "anthony.remazeilles@tecnalia.com"
__copyright__ = "Copyright (C) 2020 Tecnalia Research and Innovation"
__licence__ = "Apache 2.0"

import sys
import time
import unittest
import rospy
import rostopic
import rostest
from rospy_message_converter import message_converter

CLASSNAME = 'filterTest'


class FilterMsgTest(unittest.TestCase):
    def __init__(self, *args):
        super(FilterMsgTest, self).__init__(*args)

        self.filter_msg = None
        self.filter_stamp = None
        self.is_received = False

    def setUp(self):
        rospy.init_node(CLASSNAME)

        # warn on /use_sim_time is true
        use_sim_time = rospy.get_param('/use_sim_time', False)
        t_start = time.time()
        while not rospy.is_shutdown() and \
                use_sim_time and (rospy.Time.now() == rospy.Time(0)):
            rospy.logwarn_throttle(
                1, '/use_sim_time is specified and rostime is 0, /clock is published?')
            if time.time() - t_start > 10:
                self.fail('Timed out (10s) of /clock publication.')
            # must use time.sleep because /clock isn't yet published, so rospy.sleep hangs.
            time.sleep(0.1)

    def test_filter(self):
        self.filters = list()
        try:

            filters = rospy.get_param("~filters")

            for one_filter in filters:
                keys = ['topic_in', 'topic_out', 'msg_in', 'msg_out', 'timeout']

                for item in keys:
                    if item not in keys:
                        self.fail("{} field required, but not specified in {}".format(item, call))

            if one_filter['topic_in'] == 'None':
                rospy.logwarn('None input converted to empty input')
                one_filter['topic_in'] = dict()
            if one_filter['topic_out'] == 'None':
                rospy.logwarn('None output converted to empty output')
                one_filter['topic_out'] = dict()

            self.filters = filters

        except KeyError as err:
            msg_err = "filter_test not initialized properly \n"
            msg_err += " Parameter [%s] not set. \n" % (str(err))
            msg_err += " Caller ID: [%s] Resolved name: [%s]\n" % (
                rospy.get_caller_id(),
                rospy.resolve_name(err.args[0]))
            self.fail(msg_err)

        for item in self.filters:
            rospy.loginfo("Testing filtering {}-{}".format(item['topic_in'], item['topic_out']))
            self._test_filter(item['topic_in'], item['topic_out'], item['msg_in'], item['msg_out'], item['timeout'])
            rospy.loginfo("So far so good")

    def _filter_cb(self, msg):
        self.filter_msg = msg
        self.filter_stamp = rospy.get_time()
        rospy.loginfo("Message received!")
        self.is_received = True

    def _test_filter(self, topic_in, topic_out, msg_in, msg_out, timeout):
        self.assert_(topic_in)
        self.assert_(topic_out)
        self.assert_(msg_in)
        self.assert_(msg_out)

        l_pub = rospy.get_published_topics()
        rospy.loginfo("Detected topics: {}".format(l_pub))

        # getting message to send
        msg_in_type = rostopic.get_topic_type(topic_in)[0]
        rospy.loginfo("Type transiting on {}: {}".format(topic_in, msg_in_type))
        self.assert_(msg_in_type is not None)

        rospy.loginfo("Converting {} into {}".format(msg_in, msg_in_type))
        try:
            ros_msg_in = message_converter.convert_dictionary_to_ros_message(msg_in_type, msg_in)
            rospy.loginfo("Generated message: [%s]" % (ros_msg_in))
        except ValueError as err:
            msg_err = "Prb in message in contruction \n"
            msg_err += "Expected type: [%s]\n" % (msg_in_type)
            msg_err += "dictionary: [%s]\n" % (msg_in)
            msg_err += "Erorr: [%s]\n" % (str(err))
            self.fail(msg_err)

        # getting message to receive
        msg_out_type = rostopic.get_topic_type(topic_out)[0]
        rospy.loginfo("Type transiting on {}: {}".format(topic_out, msg_out_type))
        self.assert_(msg_out_type is not None)

        rospy.loginfo("Converting {} into {}".format(msg_out, msg_out_type))
        try:
            ros_msg_out = message_converter.convert_dictionary_to_ros_message(msg_out_type, msg_out)
            rospy.loginfo("Generated message: [%s]" % (ros_msg_out))
        except ValueError as err:
            msg_err = "Prb in message in contruction \n"
            msg_err += "Expected type: [%s]\n" % (msg_out_type)
            msg_err += "dictionary: [%s]\n" % (msg_out)
            msg_err += "Erorr: [%s]\n" % (str(err))
            self.fail(msg_err)

        # subscription
        # make sure no reception done before publication
        self.is_received = False
        pub = rospy.Publisher(topic_in, ros_msg_in.__class__, queue_size=1)
        sub = rospy.Subscriber(topic_out, ros_msg_out.__class__, self._filter_cb, queue_size=1)
        rospy.sleep(0.5)
        self.assert_(not self.is_received, "No message should be received before publication!")
        time_pub = rospy.get_time()
        pub.publish(ros_msg_in)
        # make sure one message gets received in the defined time.
        is_too_long = False
        while not self.is_received:
            if timeout is not None:
                if rospy.get_time() - time_pub > timeout:
                    is_too_long = True
                    break
            rospy.sleep(0.1)

        self.assert_(not is_too_long, 'filter out not received within indicated duration: [%s]' % timeout)

        rospy.loginfo("A message has been received!")
        rospy.loginfo("{} at {}".format(self.filter_msg, self.filter_stamp))
        duration = self.filter_stamp - time_pub
        rospy.loginfo("Filter duration: {}".format(duration))
        # check the receive message
        self.assertEqual(self.filter_msg, ros_msg_out)


def main():
    try:
        rostest.run('rostest', CLASSNAME, FilterMsgTest, sys.argv)
    except KeyboardInterrupt:
        pass
    print("{} exiting".format(CLASSNAME))
