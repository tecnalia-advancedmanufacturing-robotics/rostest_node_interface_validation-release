[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

# rostest_node_interface_validation

The package `rostest_node_interface_validation` extends `rostest` to add more testing functionalities at node level.

**Author & Maintainer**: [Anthony Remazeilles](https://github.com/aremazeilles), anthony.remazeilles@tecnalia.com

**Affiliation** : Tecnalia Research and Innovation, Spain

**License**: This project is under the Apache 2.0 License.
See [LICENSE.md](LICENSE.md) for more details.

## Getting started

### Prerequisites

We assume [`ROS`][ros] is installed on the machine.
Code developed and tested so far under `ROS kinetic`.

[ros]: http://www.ros.org/

### Installing

```shell
# Assuming ~/catkin_ws is the workspace in which the repository has been downloaded
cd ~/catkin_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

### Use

This package contains two scripts, `test_service` and `test_filter`.
We just highlight the typical usage:

#### test_service

An example is provided in [service_call.test](rostest_node_interface_validation/test/service_call.test):

```xml
<launch>
  <!-- Following node is a basic service server -->
  <node pkg="rostest_node_interface_validation" type="service_server.py" name="service_server"/>
  <!-- configuration of the test-->
  <test test-name="test_service" pkg="rostest_node_interface_validation" type="test_service" >
      <rosparam>
        calls:
          - name: /trigger_spec
            input: None
            output: {'success': True, 'message': 'well done!'}
      </rosparam>
  </test>
</launch>
```

where:

* `name`: is the service to test
* `input`: input parameters, defined as a dictionary
* `output`: expected output, defined as a dictionary

The test will connect to the service indicated, call it with the provided parameters, and compare the output received with the one defined.
The test succeeds if all went well.

Services with no input messages should use term `None` for the input value, as illustrated above.

Another example can be generated if `rospy_tutorial` is installed.
If one creates the following example `example_srv.test` file:

```xml
<launch>
  <test test-name="test_service" pkg="rostest_node_interface_validation" type="test_service" >
    <rosparam>
      calls:
        - name: /add_two_ints
          input: {'a': 0, 'b': 5.0}
          output: {'sum': 5}
    </rosparam>
  </test>
</launch>
```

```shell
# in shell 1
roscore
# in shell 2
rosrun rospy_tutorials add_two_ints_server
# in shell 3 (adjust rostest_node_interface_validation to the package where the test file is placed)
rostest rostest_node_interface_validation example_srv.test
```

### test_filter


`test_filter` enables testing a filter-like node, that is supposed to publish a message after having processed a received message.

An example is provided in [msg_filter.test](rostest_node_interface_validation/test/msg_filter.test):

```xml
<launch>
    <!-- Following node is a basic filter node that multiply received value by 2, and publish it. -->
    <!-- parameter "wait" can  be used to wait a given time before publishing the result. -->
    <node name="dummy_node" pkg="rostest_node_interface_validation" type="dummy_filter.py">
        <param name="wait" value="0.2" />
    </node>

    <test test-name="filter_test" pkg="rostest_node_interface_validation" type="test_filter">
        <rosparam>
          filters:
            - topic_in: /filter_in
              topic_out: /filter_out
              msg_in: {'data': 2.0}
              msg_out: {'data': 4.0}
              timeout: 1.0
        </rosparam>
    </test>
</launch>
```

`dummy_node` is a simple filter node, which doubles a received value.
Parameter `wait` enables emulating the processing time before publishing the result.

The parameters of `test_filter` node are:

* `topic_in`: topic to which the filter expects the message
* `topic_out`: topic to which the filter will publish the filter result
* `msg_in`; input message to be transmitted to the filter, written as a python dictionary
* `msg_out`: output message that should be published by the filter
* `timeout`: maximum time expected for the generation of the filter output

## Acknowledgements

This development is supported by the European Union’s Horizon 2020 project [ROSIN][rosin_website].
This project has received funding from the European Union’s Horizon 2020 research and innovation programme under
grant agreement No 732287.

The opinions and arguments expressed reflect only the author‘s view and reflect in no way the European Commission‘s opinions.
The European Commission is not responsible for any use that may be made of the information it contains.

[![ROSIN website][rosin_logo]][rosin_website]

[rosin_logo]: http://rosin-project.eu/wp-content/uploads/2017/03/Logo_ROSIN_CMYK-Website.png
[rosin_website]: http://rosin-project.eu/ "Go to website"
