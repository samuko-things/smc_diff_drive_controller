# Copyright 2021 AUTHORS
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the AUTHORS nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.



from rclpy.node import Node


class NodeParameters:
  """
  ROS2 Node Parameter Handling.

  https://index.ros.org/doc/ros2/Tutorials/Parameters/Understanding-ROS2-Parameters
  https://index.ros.org/doc/ros2/Tutorials/Using-Parameters-In-A-Class-Python/

  Start the node with parameters from yml file:
  ros2 run smc_diff_drive_controller smc_diff_drive_controller

  with the following arguments:
  --ros-args --params-file ~/<workspace>/src/smc_diff_drive_controller/config/smc_diff_drive_controller_params.yaml
  """

  def __init__(self, node: Node):
    node.get_logger().info('Initializing parameters')
    # Declare parameters of the ROS2 node and their default values:
    # if no parameters are given, use the default value

    node.declare_parameter('serial_port', value='')

    node.declare_parameter('base_frame_id', value='base_footprint')
    node.declare_parameter('odom_frame_id', value='odom')
    node.declare_parameter('odom_tf_freq', value=30)
    node.declare_parameter('odom_topic', value='/odom')

    node.declare_parameter('wheel_radius', value=0.0)
    node.declare_parameter('wheel_seperation', value=0.0)

    node.declare_parameter('linear_x_max_velocity', value=2.0)
    node.declare_parameter('linear_x_min_velocity', value=-2.0)

    node.declare_parameter('angular_z_max_velocity', value=4.0)
    node.declare_parameter('angular_z_min_velocity', value=-4.0)
    

    # get the parameters - requires CLI arguments '--ros-args --params-file <parameter file>'
    node.get_logger().info('Parameters set to:')

    try:
      self.serial_port = node.get_parameter('serial_port')
      node.get_logger().info('\tserial_port:\t\t"%s"' % self.serial_port.value)


      self.base_frame_id = node.get_parameter('base_frame_id')
      node.get_logger().info('\tbase_frame_id:\t\t"%s"' % self.base_frame_id.value)

      self.odom_frame_id = node.get_parameter('odom_frame_id')
      node.get_logger().info('\todom_frame_id:\t\t"%s"' % self.odom_frame_id.value)

      self.odom_tf_freq = node.get_parameter('odom_tf_freq')
      node.get_logger().info('\todom_tf_freq:\t\t"%s"' % self.odom_tf_freq.value)

      self.odom_topic = node.get_parameter('odom_topic')
      node.get_logger().info('\todom_topic:\t\t"%s"' % self.odom_topic.value)


      self.wheel_radius = node.get_parameter('wheel_radius')
      node.get_logger().info('\twheel_radius:\t\t"%s"' % self.wheel_radius.value)

      self.wheel_seperation = node.get_parameter('wheel_seperation')
      node.get_logger().info('\twheel_seperation:\t\t"%s"' % self.wheel_seperation.value)


      self.linear_x_max_velocity = node.get_parameter('linear_x_max_velocity')
      node.get_logger().info('\tlinear_x_max_velocity:\t\t"%s"' % self.linear_x_max_velocity.value)

      self.linear_x_min_velocity = node.get_parameter('linear_x_min_velocity')
      node.get_logger().info('\tlinear_x_min_velocity:\t\t"%s"' % self.linear_x_min_velocity.value)


      self.angular_z_max_velocity = node.get_parameter('angular_z_max_velocity')
      node.get_logger().info('\tangular_z_max_velocity:\t\t"%s"' % self.angular_z_max_velocity.value)

      self.angular_z_min_velocity = node.get_parameter('angular_z_min_velocity')
      node.get_logger().info('\tangular_z_min_velocity:\t\t"%s"' % self.angular_z_min_velocity.value)


    except Exception as e:  # noqa: B902
      node.get_logger().warn('Could not get parameters...setting variables to default')
      node.get_logger().warn('Error: "%s"' % e)
