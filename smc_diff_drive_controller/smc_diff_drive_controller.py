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

from time import sleep, time

from smc_diff_drive_controller.modules.NodeParameters import NodeParameters
from smc_diff_drive_controller.modules.smc_arduino_pyserial_comm import SMCArduinoSerialComm


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster

from nav_msgs.msg import Odometry




###############################################################################################################
# This program converts Euler angles to a quaternion.
# Author: AutomaticAddison.com
 
import numpy as np # Scientific computing library for Python
import math
 
def quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]



# This program converts a Quaternion to Euler angles.
# Author: AutomaticAddison.com

def euler_from_quaternion(x, y, z, w):
  """
  Convert a quaternion into euler angles (roll, pitch, yaw)
  roll is rotation around x in radians (counterclockwise)
  pitch is rotation around y in radians (counterclockwise)
  yaw is rotation around z in radians (counterclockwise)
  """

  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + y * y)
  roll_x = math.atan2(t0, t1)

  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  pitch_y = math.asin(t2)

  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (y * y + z * z)
  yaw_z = math.atan2(t3, t4)

  return roll_x, pitch_y, yaw_z # in radians

###################################################################################################################





class DiffDriveController(Node):

    def __init__(self):
        # Initialize parent (ROS Node)
        super().__init__('smc_diff_drive_controller')
        
        # Initialize ROS2 Node Parameters:
        self.param = NodeParameters(self)

        self.serial_port = self.param.serial_port.value

        self.base_frame_id = self.param.base_frame_id.value
        self.odom_frame_id = self.param.odom_frame_id.value
        self.odom_tf_freq = self.param.odom_tf_freq.value
        self.odom_topic = self.param.odom_topic.value

        self.wheel_radius = self.param.wheel_radius.value
        self.wheel_seperation = self.param.wheel_seperation.value

        self.linear_x_max_velocity = self.param.linear_x_max_velocity.value
        self.linear_x_min_velocity = self.param.linear_x_min_velocity.value

        self.angular_z_max_velocity = self.param.angular_z_max_velocity.value
        self.angular_z_min_velocity = self.param.angular_z_min_velocity.value


        # odometry pose calculation parameters
        self.thetaL = 0.00
        self.thetaR = 0.00
        self.wL = 0.00
        self.wR = 0.00

        self.prevThetaL = self.thetaL
        self.prevThetaR = self.thetaR

        self.poseX = 0.00
        self.poseY = 0.00
        self.poseTheta = 0.00

        self.prevPoseX = 0.00
        self.prevPoseY = 0.00
        self.prevPoseTheta = 0.00

        self.prevTime = time()
        self.newTime = time()



        self.ser = SMCArduinoSerialComm(self.serial_port, 115200, 0.1)
        for i in range(5):
          sleep(1.0)
          self.get_logger().info('configuring controller: %d sec' %(i))
        self.get_logger().info('configuration complete')
        

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.diff_drive_callback,
            10)
        self.cmd_vel_sub  # prevent unused variable warning

        # create publisher to publish odometry message:
        QoSProf = QoSProfile(depth=10)
        self.pub_odom_msg = self.create_publisher(Odometry, self.odom_topic, QoSProf)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize timer to broadcast odom frame transform
        self.odom_tf_period = 1.0/self.odom_tf_freq  # seconds
        self.timer = self.create_timer(self.odom_tf_period, self.broadcast_odom_tf)

        


    def diff_drive_callback(self, cmd):
      """Get cmd velocity and send it to the smc controller and """

      V_robot = cmd.linear.x
      W_robot = cmd.angular.z

      if V_robot > self.linear_x_max_velocity:
         V_robot = self.linear_x_max_velocity
      elif V_robot < self.linear_x_min_velocity:
         V_robot = self.linear_x_min_velocity

      if W_robot > self.angular_z_max_velocity:
         W_robot = self.angular_z_max_velocity
      elif W_robot < self.angular_z_min_velocity:
         W_robot = self.angular_z_min_velocity

      wl = ( (2*V_robot) - (W_robot*self.wheel_seperation) ) / (2*self.wheel_radius)
      wr = ( (2*V_robot) + (W_robot*self.wheel_seperation) ) / (2*self.wheel_radius)

      self.ser.sendTargetVel(wl, wr)


    def publish_odometry_msg(self):
      odom_msg = Odometry()

      odom_msg.header.frame_id = self.odom_frame_id
      odom_msg.child_frame_id = self.base_frame_id

      odom_msg.pose.pose.position.x = self.poseX
      odom_msg.pose.pose.position.y = self.poseY
      odom_msg.pose.pose.position.z = 0.0

      qx,qy,qz,qw = quaternion_from_euler(0.00, 0.00, self.poseTheta); # convert rpy to quaternion
      odom_msg.pose.pose.orientation.x = qx
      odom_msg.pose.pose.orientation.y = qy
      odom_msg.pose.pose.orientation.z = qz
      odom_msg.pose.pose.orientation.w = qw

      v = (self.wheel_radius/2.00)*(self.wL + self.wR)
      w = (self.wheel_radius/self.wheel_seperation)*(self.wL - self.wR)

      odom_msg.twist.twist.linear.x = v
      odom_msg.twist.twist.linear.y = 0.00
      odom_msg.twist.twist.linear.z = 0.00

      odom_msg.twist.twist.angular.x = 0.00
      odom_msg.twist.twist.angular.y = 0.00
      odom_msg.twist.twist.angular.z = w

      self.pub_odom_msg.publish(odom_msg)
         


    def broadcast_odom_tf(self):
      # self.newTime = time()
      # dt = self.newTime - self.prevTime
      try:
        self.thetaL, self.thetaR = self.ser.getMotorsPos()
        self.wL, self.wR = self.ser.getMotorsVel()
        # self.get_logger().info('\nthetaL= %f m\nthetaR= %f m\n' %(self.thetaL, self.thetaR))
        # self.thetaL = self.thetaL+(self.wL*dt)
        # self.thetaR = self.thetaR+(self.wR*dt)

        self.poseX = self.prevPoseX + ( (self.wheel_radius/2)*((self.thetaR-self.prevThetaR)+(self.thetaL-self.prevThetaL))*math.cos(self.prevPoseTheta) )
        self.poseY = self.prevPoseY + ( (self.wheel_radius/2)*((self.thetaR-self.prevThetaR)+(self.thetaL-self.prevThetaL))*math.sin(self.prevPoseTheta) )
        self.poseTheta = self.prevPoseTheta + (self.wheel_radius/self.wheel_seperation)*((self.thetaR-self.prevThetaR)-(self.thetaL-self.prevThetaL))

        ###########################################################################
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id

        t.transform.translation.x = self.poseX
        t.transform.translation.y = self.poseY
        t.transform.translation.z = 0.0

        qx,qy,qz,qw = quaternion_from_euler(0.00, 0.00, self.poseTheta); # convert rpy to quaternion
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
        #############################################################################

        self.publish_odometry_msg()


        self.prevThetaL = self.thetaL
        self.prevThetaR = self.thetaR

        self.prevPoseX = self.poseX
        self.prevPoseY = self.poseY
        self.prevPoseTheta = self.poseTheta

        self.wL = 0.00
        self.wR = 0.00
      except:
        pass

      # self.get_logger().info('\nposeX= %f m\nposeY= %f m\ntheta= %f rad\n' %(self.poseX, self.poseY, self.poseTheta))
      # self.prevTime = self.newTime

        



def main(args=None):
    rclpy.init(args=args)

    smc_diff_drive_controller_node = DiffDriveController()

    rclpy.spin(smc_diff_drive_controller_node)

    smc_diff_drive_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()