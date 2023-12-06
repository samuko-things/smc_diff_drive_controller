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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='smc_diff_drive_controller' #<--- CHANGE ME


    ######################################################################################

    # static transform from base_footprint (parent frame) to imu (child frame)
    tf_base_footprint_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0.0', '--y', '0.0', '--z', '0.1', 
                     '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0', 
                     '--frame-id', 'base_footprint', '--child-frame-id', 'imu']
    )

    # static transform from base_footprint (parent frame) to sonar (child frame)
    tf_base_footprint_to_sonar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0.09', '--y', '0.0', '--z', '0.1', 
                     '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0', 
                     '--frame-id', 'base_footprint', '--child-frame-id', 'sonar']
    )

    ########################################################################################
  



    ##########################################################################################

    # the samuko_mpu9250_imu publishes the imu data msg on the imu frame
    # it publishes the imu message on the /imu_data topic
    imu_config_file = os.path.join(get_package_share_directory(package_name),'config', 'samuko_mpu9250_imu_params.yaml')
    samuko_mpu9250_imu_node = Node(
        package = 'samuko_mpu9250_imu',
        executable = 'samuko_mpu9250_imu',
        parameters = [imu_config_file]
    )

    # the smc_diff_drive_controller broadcasts
    # dynamic transform from odom (parent frame) to base_footprint (child frame)
    # it also publishes the odometry message on the /odom topic
    diff_drive_config_file = os.path.join(get_package_share_directory(package_name),'config', 'smc_diff_drive_controller_params.yaml')
    smc_diff_drive_controller_node = Node(
        package = 'smc_diff_drive_controller',
        executable = 'smc_diff_drive_controller',
        parameters = [diff_drive_config_file]
    )

    # Start robot localization using an Extended Kalman filter
    # fuse imu and odometry together and publishes on /odometry/filtered topic
    ekf_config_file = os.path.join(get_package_share_directory(package_name),'config', 'ekf_fuse_imu_odom.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': 'false'}])

    ################################################################################################




    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the nodes to the launch description
    ld.add_action(tf_base_footprint_to_imu)
    ld.add_action(tf_base_footprint_to_sonar)
    
    ld.add_action(smc_diff_drive_controller_node)
    ld.add_action(samuko_mpu9250_imu_node)
    ld.add_action(ekf_node)
        
    return ld      # return (i.e send) the launch description for excecution
