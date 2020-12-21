#!/usr/bin/env python2
import numpy as np
import argparse
import rospy
import sys
import os


def generate_singlecam_sdf_function(pitch_angle_deg=0.0, ros_update_rate=10.0,
                                  yaw_angle_left=-2.3562, yaw_angle_right=2.3562,
                                  include_cam_visualisation=True,
                                  fov_horizontal_rad=0.73675,
                                  cam_w=752,
                                  cam_h=480,
                                  ):

    pitch_angle_rad = (np.pi/2.0) - np.deg2rad(pitch_angle_deg)

    # get environmental variable of model location
    px4_dir = (os.environ.get('PX4_SRC_DIR'))
    if not px4_dir:
        raise ('need to set the PX4_SRC_DIR environmental variable for this script to work')
    else:
        save_dir = os.path.join(px4_dir, 'Tools/sitl_gazebo/models/typhoon_2cam/typhoon_2cam.sdf')
        print ('saving output to: {}'.format(save_dir))

    # main camera string
    main_camera_bit = """
    <sensor name='camera' type='camera'>
          <pose>-0.051 0 -0.162 0 """ + str(pitch_angle_rad) + """ 3.14159</pose>
          <camera name='__default__'>   
            <horizontal_fov>""" + str(fov_horizontal_rad) + """</horizontal_fov>    <!--1.0471975511965976    or 100 degs 1.7453292519943295  or 90 deg 1.5707963267948966 -->     <!--  <horizontal_fov>1.0122909661567112</horizontal_fov>    1.0471975511965976    or 100 degs 1.7453292519943295  or 90 deg 1.5707963267948966 -->
            <image>
              <width>""" + str(cam_w) + """</width>
              <height>""" + str(cam_h) + """</height>
            </image>
            <clip>
              <near>0.1</near>
              <far>100</far>
            </clip>
          </camera>

          <always_on>1</always_on>
          <update_rate>10.0</update_rate>
          <visualize>true</visualize>

          <plugin name='camera_plugin' filename='libgazebo_ros_camera.so'>
              <!--<robotNamespace></robotNamespace>-->
            <alwaysOn>true</alwaysOn>
            <imageTopicName>image_raw</imageTopicName>
            <cameraInfoTopicName>camera_info</cameraInfoTopicName>
            <updateRate>10.0</updateRate>
            <!--<updateRate>30.0</updateRate>-->
            <cameraName>usb_cam</cameraName>      <!--resize_img -->
            <frameName>/robot_camera_link</frameName>
          </plugin>
        </sensor>
         """

    preliminary_str = """    
    <sdf version='1.5'>
  <model name='typhoon_2cam'>
    <!-- Typhoon H body -->
    <pose>0 0 0.26 0 0 3.1415927</pose>
    <link name='base_link'>
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <!-- rear reference point X: -100.044mm, CAD offset: 1001.049mm -->
        <!-- top reference point Z: 33.8663mm, CAD offset: 42.8698mm -->
        <pose>0.001005 0 -0.0090035 0 0 0</pose>
        <mass>2.02</mass>
        <inertia>
          <ixx>0.011</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.015</iyy>
          <iyz>0</iyz>
          <izz>0.021</izz>
        </inertia>
      </inertial>
      <collision name='base_link_collision'>
        <pose>0 0 0.0 0 0 0</pose>
        <geometry>
          <box>
            <size>0.67 0.67 0.15</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode/>
          </friction>
        </surface>
      </collision>
      <visual name='base_link_visual'>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>0.001 0.001 0.001</scale>
            <uri>model://typhoon_2cam/meshes/main_body_remeshed_v3.stl</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>

      <gravity>1</gravity>
      <velocity_decay/>
      <self_collide>0</self_collide>
    </link>

    <link name="cgo3_mount_link">
      <inertial>
        <!-- place holder -->
        <pose>-0.041 0 -0.162 0 0 0</pose>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <visual name='cgo3_mount_visual'>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>0.001 0.001 0.001</scale>
            <uri>model://typhoon_2cam/meshes/cgo3_mount_remeshed_v1.stl</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
    </link>

    <joint name='cgo3_mount_joint' type='revolute'>
      <child>cgo3_mount_link</child>
      <parent>base_link</parent>
      <pose>0 0 0 0 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>0</lower>
          <upper>0</upper>
          <effort>100</effort>
          <velocity>-1</velocity>
        </limit>
        <dynamics>
          <damping>1</damping>
        </dynamics>
        <use_parent_model_frame>1</use_parent_model_frame>
      </axis>
      <physics>
        <ode>
          <implicit_spring_damper>1</implicit_spring_damper>
        </ode>
      </physics>
    </joint>

    <link name="cgo3_vertical_arm_link">
      <inertial>
        <!-- place holder -->
        <pose>-0.041 0 -0.162 0 0 0</pose>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <visual name='cgo3_vertical_arm_visual'>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>0.001 0.001 0.001</scale>
            <uri>model://typhoon_2cam/meshes/cgo3_vertical_arm_remeshed_v1.stl</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
    </link>

    <joint name='cgo3_vertical_arm_joint' type='revolute'>
      <child>cgo3_vertical_arm_link</child>
      <parent>cgo3_mount_link</parent>
      <pose>-0.026 0 -0.10 0 0 0</pose>
      <!--
      <controlIndex>6</controlIndex>
      -->
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1e16</lower>
          <upper>1e16</upper>
          <effort>100</effort>
          <velocity>-1</velocity>
        </limit>
        <dynamics>
          <damping>0.1</damping>
        </dynamics>
        <use_parent_model_frame>1</use_parent_model_frame>
      </axis>
      <physics>
        <ode>
          <implicit_spring_damper>1</implicit_spring_damper>
          <limit>
            <!-- testing soft limits -->
            <cfm>0.1</cfm>
            <erp>0.2</erp>
          </limit>
        </ode>
      </physics>
    </joint>

    <link name="cgo3_horizontal_arm_link">
      <inertial>
        <!-- place holder -->
        <pose>-0.041 0 -0.081 0 0 0</pose>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <visual name='cgo3_horizontal_arm_visual'>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>0.001 0.001 0.001</scale>
            <uri>model://typhoon_2cam/meshes/cgo3_horizontal_arm_remeshed_v1.stl</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
    </link>
    <joint name='cgo3_horizontal_arm_joint' type='revolute'>
      <child>cgo3_horizontal_arm_link</child>
      <parent>cgo3_vertical_arm_link</parent>
      <pose>0.026 0 -0.162 0 0 0</pose>
      <!--
      <controlIndex>7</controlIndex>
      -->
      <axis>
        <xyz>-1 0 0</xyz>
        <limit>
          <lower>-0.785398</lower>
          <upper>0.785398</upper>
          <effort>100</effort>
          <velocity>-1</velocity>
        </limit>
        <dynamics>
          <damping>0.1</damping>
        </dynamics>
        <use_parent_model_frame>1</use_parent_model_frame>
      </axis>
      <physics>
        <ode>
          <implicit_spring_damper>1</implicit_spring_damper>
          <limit>
            <!-- testing soft limits -->
            <cfm>0.1</cfm>
            <erp>0.2</erp>
          </limit>
        </ode>
      </physics>
    </joint>

    <link name="cgo3_camera_link">
      <inertial>
        <!-- place holder -->
        <pose>-0.041 0 -0.162 0 0 0</pose>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <collision name='cgo3_camera_collision'>
        <pose>-0.041 0 -0.162 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>0.035</radius>
          </sphere>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1</mu>
              <mu2>1</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>1e+8</kp>
              <kd>1</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
      <visual name='cgo3_camera_visual'>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>0.001 0.001 0.001</scale>
            <uri>model://typhoon_2cam/meshes/cgo3_camera_remeshed_v1.stl</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>

      <sensor name="camera_imu" type="imu">
        <always_on>1</always_on>
      </sensor>
    """

    final_str = """       
        
        </link>
    
        <joint name='cgo3_camera_joint' type='revolute'>
          <child>cgo3_camera_link</child>
          <parent>cgo3_horizontal_arm_link</parent>
          <pose>-0.041 0.03 -0.162 0 0 0</pose>
          <axis>
            <xyz>0 -1 0</xyz>
            <limit>
              <lower>-1.05</lower>
              <upper>2.09</upper>
              <effort>100</effort>
              <velocity>-1</velocity>
            </limit>
            <dynamics>
              <damping>0.1</damping>
            </dynamics>
            <use_parent_model_frame>1</use_parent_model_frame>
          </axis>
          <physics>
            <ode>
              <implicit_spring_damper>1</implicit_spring_damper>
              <limit>
                <!-- testing soft limits -->
                <cfm>0.1</cfm>
                <erp>0.2</erp>
              </limit>
            </ode>
          </physics>
        </joint>
    
        <link name="left_leg">
          <inertial>
            <!-- place holder -->
            <pose>0 -0.14314 -0.207252 0 0 0</pose>
            <mass>0.1</mass>
            <inertia>
              <ixx>0.001</ixx>
              <ixy>0</ixy>
              <ixz>0</ixz>
              <iyy>0.001</iyy>
              <iyz>0</iyz>
              <izz>0.001</izz>
            </inertia>
          </inertial>
          <collision name='collision'>
            <pose>-0.005 -0.14314 -0.207252 0 1.56893 0</pose>
            <geometry>
              <cylinder>
                <radius>0.012209</radius>
                <length>0.3</length>
              </cylinder>
            </geometry>
            <surface>
              <friction>
                <ode>
                  <mu>1</mu>
                  <mu2>1</mu2>
                </ode>
              </friction>
              <contact>
                <ode>
                  <kp>1e+8</kp>
                  <kd>1</kd>
                  <max_vel>0.01</max_vel>
                  <min_depth>0.001</min_depth>
                </ode>
              </contact>
            </surface>
          </collision>
          <collision name='collision_bar'>
            <pose>0.00052 -0.08503 -0.121187 -0.501318 0 0</pose>
            <geometry>
              <cylinder>
                <radius>0.00914984</radius>
                <length>0.176893</length>
              </cylinder>
            </geometry>
            <surface>
              <friction>
                <ode>
                  <mu>1</mu>
                  <mu2>1</mu2>
                </ode>
              </friction>
              <contact>
                <ode>
                  <kp>1e+8</kp>
                  <kd>1</kd>
                  <max_vel>0.01</max_vel>
                  <min_depth>0.001</min_depth>
                </ode>
              </contact>
            </surface>
          </collision>
          <visual name='base_link_left_leg'>
            <pose>0 0 0 0 0 0</pose>
            <geometry>
              <mesh>
                <scale>0.001 0.001 0.001</scale>
                <uri>model://typhoon_2cam/meshes/leg2_remeshed_v3.stl</uri>
              </mesh>
            </geometry>
            <material>
              <script>
                <name>Gazebo/DarkGrey</name>
                <uri>file://media/materials/scripts/gazebo.material</uri>
              </script>
            </material>
          </visual>
        </link>
        <joint name='left_leg_joint' type='revolute'>
          <child>left_leg</child>
          <parent>base_link</parent>
          <pose>0.00026 -0.040515 -0.048 0 0 0</pose>
          <axis>
            <xyz>-1 0 0</xyz>
            <limit>
              <lower>0</lower>
              <upper>1</upper>
              <effort>100</effort>
              <velocity>-1</velocity>
              <stiffness>100000000</stiffness>
              <dissipation>1</dissipation>
            </limit>
            <dynamics>
              <damping>0.1</damping>
            </dynamics>
            <use_parent_model_frame>1</use_parent_model_frame>
          </axis>
          <physics>
            <ode>
              <implicit_spring_damper>1</implicit_spring_damper>
            </ode>
          </physics>
        </joint>
    
        <link name="right_leg">
          <pose>0 0 0 0 0 0</pose>
          <inertial>
            <!-- place holder -->
            <pose>0 0.14314 -0.207252 0 0 0</pose>
            <mass>0.1</mass>
            <inertia>
              <ixx>0.001</ixx>
              <ixy>0</ixy>
              <ixz>0</ixz>
              <iyy>0.001</iyy>
              <iyz>0</iyz>
              <izz>0.001</izz>
            </inertia>
          </inertial>
          <collision name='collision'>
            <pose>-0.005 0.14314 -0.207252 0 1.56893 0</pose>
            <geometry>
              <cylinder>
                <radius>0.012209</radius>
                <length>0.3</length>
              </cylinder>
            </geometry>
            <surface>
              <friction>
                <ode>
                  <mu>1</mu>
                  <mu2>1</mu2>
                </ode>
              </friction>
              <contact>
                <ode>
                  <kp>1e+8</kp>
                  <kd>1</kd>
                  <max_vel>0.01</max_vel>
                  <min_depth>0.001</min_depth>
                </ode>
              </contact>
            </surface>
          </collision>
          <collision name='collision_bar'>
            <pose>0.00052 0.08503 -0.121187 0.501318 0 0</pose>
            <geometry>
              <cylinder>
                <radius>0.00914984</radius>
                <length>0.176893</length>
              </cylinder>
            </geometry>
            <surface>
              <friction>
                <ode>
                  <mu>1</mu>
                  <mu2>1</mu2>
                </ode>
              </friction>
              <contact>
                <ode>
                  <kp>1e+8</kp>
                  <kd>1</kd>
                  <max_vel>0.01</max_vel>
                  <min_depth>0.001</min_depth>
                </ode>
              </contact>
            </surface>
          </collision>
          <visual name='base_link_right_leg'>
            <pose>0 0 0 0 0 0</pose>
            <geometry>
              <mesh>
                <scale>0.001 0.001 0.001</scale>
                <uri>model://typhoon_2cam/meshes/leg1_remeshed_v3.stl</uri>
              </mesh>
            </geometry>
            <material>
              <script>
                <name>Gazebo/DarkGrey</name>
                <uri>file://media/materials/scripts/gazebo.material</uri>
              </script>
            </material>
          </visual>
        </link>
        <joint name='right_leg_joint' type='revolute'>
          <child>right_leg</child>
          <parent>base_link</parent>
          <pose>0.00026 0.040515 -0.048 0 0 0</pose>
          <axis>
            <xyz>1 0 0</xyz>
            <limit>
              <lower>0</lower>
              <upper>1</upper>
              <effort>100</effort>
              <velocity>-1</velocity>
              <stiffness>100000000</stiffness>
              <dissipation>1</dissipation>
            </limit>
            <dynamics>
              <damping>0.1</damping>
            </dynamics>
            <use_parent_model_frame>1</use_parent_model_frame>
          </axis>
          <physics>
            <ode>
              <implicit_spring_damper>1</implicit_spring_damper>
            </ode>
          </physics>
        </joint>
    
        <link name='typhoon_2cam/imu_link'>
          <pose>0 0 0 0 0 3.1415927</pose>
          <inertial>
            <pose>0 0 0 0 0 0</pose>
            <mass>0.015</mass>
            <inertia>
              <ixx>1e-05</ixx>
              <ixy>0</ixy>
              <ixz>0</ixz>
              <iyy>1e-05</iyy>
              <iyz>0</iyz>
              <izz>1e-05</izz>
            </inertia>
          </inertial>
        </link>
        <joint name='typhoon_2cam/imu_joint' type='revolute'>
          <child>typhoon_2cam/imu_link</child>
          <parent>base_link</parent>
          <axis>
            <xyz>1 0 0</xyz>
            <limit>
              <lower>0</lower>
              <upper>0</upper>
              <effort>0</effort>
              <velocity>0</velocity>
            </limit>
            <dynamics>
              <spring_reference>0</spring_reference>
              <spring_stiffness>0</spring_stiffness>
            </dynamics>
            <use_parent_model_frame>1</use_parent_model_frame>
          </axis>
        </joint>
        <link name='rotor_3'>
          <pose>0.211396 0.119762 0.082219 0 0 0</pose>
          <inertial>
            <pose>0 0 0 0 0 0</pose>
            <mass>0.005</mass>
            <inertia>
              <ixx>9.75e-07</ixx>
              <ixy>0</ixy>
              <ixz>0</ixz>
              <iyy>0.000273104</iyy>
              <iyz>0</iyz>
              <izz>0.000274004</izz>
            </inertia>
          </inertial>
          <collision name='rotor_3_collision'>
            <pose>0 0 0 0 0 0</pose>
            <geometry>
              <cylinder>
                <length>0.005</length>
                <radius>0.128</radius>
              </cylinder>
            </geometry>
            <surface>
              <contact>
                <ode/>
              </contact>
              <friction>
                <ode/>
              </friction>
            </surface>
          </collision>
          <visual name='rotor_3_visual'>
            <pose>-0.211396 -0.119762 -0.082219 0 0 0</pose>
            <geometry>
              <mesh>
                <scale>0.001 0.001 0.001</scale>
                <uri>model://typhoon_2cam/meshes/prop_ccw_assembly_remeshed_v3.stl</uri>
              </mesh>
            </geometry>
            <material>
              <script>
                <name>Gazebo/Blue</name>
                <uri>file://media/materials/scripts/gazebo.material</uri>
              </script>
            </material>
          </visual>
          <gravity>1</gravity>
          <velocity_decay/>
          <self_collide>0</self_collide>
        </link>
        <joint name='rotor_3_joint' type='revolute'>
          <child>rotor_3</child>
          <parent>base_link</parent>
          <axis>
            <xyz>0.0446 -0.0825 1.8977</xyz>
            <limit>
              <lower>-1e+16</lower>
              <upper>1e+16</upper>
              <effort>10</effort>
              <velocity>-1</velocity>
            </limit>
            <dynamics>
              <damping>0.005</damping>
            </dynamics>
            <use_parent_model_frame>1</use_parent_model_frame>
          </axis>
          <physics>
            <ode>
              <implicit_spring_damper>1</implicit_spring_damper>
            </ode>
          </physics>
        </joint>
        <link name='rotor_0'>
          <pose>-0.209396 0.122762 0.082219 0 0 2.09439510239</pose>
          <inertial>
            <pose>0 0 0 0 0 0</pose>
            <mass>0.005</mass>
            <inertia>
              <ixx>9.75e-07</ixx>
              <ixy>0</ixy>
              <ixz>0</ixz>
              <iyy>0.000273104</iyy>
              <iyz>0</iyz>
              <izz>0.000274004</izz>
            </inertia>
          </inertial>
          <collision name='rotor_0_collision'>
            <pose>0 0 0 0 0 0</pose>
            <geometry>
              <cylinder>
                <length>0.005</length>
                <radius>0.128</radius>
              </cylinder>
            </geometry>
            <surface>
              <contact>
                <ode/>
              </contact>
              <friction>
                <ode/>
              </friction>
            </surface>
          </collision>
          <visual name='rotor_0_visual'>
            <pose>-0.211396 -0.119762 -0.082219 0 0 0</pose>
            <geometry>
              <mesh>
                <scale>0.001 0.001 0.001</scale>
                <uri>model://typhoon_2cam/meshes/prop_ccw_assembly_remeshed_v3.stl</uri>
              </mesh>
            </geometry>
            <material>
              <script>
                <name>Gazebo/Blue</name>
                <uri>file://media/materials/scripts/gazebo.material</uri>
              </script>
            </material>
          </visual>
          <gravity>1</gravity>
          <velocity_decay/>
          <self_collide>0</self_collide>
        </link>
        <joint name='rotor_0_joint' type='revolute'>
          <child>rotor_0</child>
          <parent>base_link</parent>
          <axis>
            <xyz>0.046 0.0827 1.8977</xyz>
            <limit>
              <lower>-1e+16</lower>
              <upper>1e+16</upper>
              <effort>10</effort>
              <velocity>-1</velocity>
            </limit>
            <dynamics>
              <damping>0.005</damping>
            </dynamics>
            <use_parent_model_frame>1</use_parent_model_frame>
          </axis>
          <physics>
            <ode>
              <implicit_spring_damper>1</implicit_spring_damper>
            </ode>
          </physics>
        </joint>
        <link name='rotor_4'>
          <pose>-0.00187896 0.242705 0.0822169 0 0 0</pose>
          <inertial>
            <pose>0 0 0 0 0 0</pose>
            <mass>0.005</mass>
            <inertia>
              <ixx>9.75e-07</ixx>
              <ixy>0</ixy>
              <ixz>0</ixz>
              <iyy>0.000273104</iyy>
              <iyz>0</iyz>
              <izz>0.000274004</izz>
            </inertia>
          </inertial>
          <collision name='rotor_4_collision'>
            <pose>0 0 0 0 0 0</pose>
            <geometry>
              <cylinder>
                <length>0.005</length>
                <radius>0.128</radius>
              </cylinder>
            </geometry>
            <surface>
              <contact>
                <ode/>
              </contact>
              <friction>
                <ode/>
              </friction>
            </surface>
          </collision>
          <visual name='rotor_4_visual'>
            <pose>0.00187896 -0.242705 -0.0822169 0 0 0</pose>
            <geometry>
              <mesh>
                <scale>0.001 0.001 0.001</scale>
                <uri>model://typhoon_2cam/meshes/prop_cw_assembly_remeshed_v3.stl</uri>
              </mesh>
            </geometry>
            <material>
              <script>
                <name>Gazebo/DarkGrey</name>
                <uri>file://media/materials/scripts/gazebo.material</uri>
              </script>
            </material>
          </visual>
          <gravity>1</gravity>
          <velocity_decay/>
          <self_collide>0</self_collide>
        </link>
        <joint name='rotor_4_joint' type='revolute'>
          <child>rotor_4</child>
          <parent>base_link</parent>
          <axis>
            <xyz>-0.09563 -0.0003 1.8976</xyz>
            <limit>
              <lower>-1e+16</lower>
              <upper>1e+16</upper>
              <effort>10</effort>
              <velocity>-1</velocity>
            </limit>
            <dynamics>
              <damping>0.005</damping>
            </dynamics>
            <use_parent_model_frame>1</use_parent_model_frame>
          </axis>
          <physics>
            <ode>
              <implicit_spring_damper>1</implicit_spring_damper>
            </ode>
          </physics>
        </joint>
        <link name='rotor_1'>
          <pose>0.211396 -0.119762 0.082219 0 0 -2.09439510239</pose>
          <inertial>
            <pose>0 0 0 0 0 0</pose>
            <mass>0.005</mass>
            <inertia>
              <ixx>9.75e-07</ixx>
              <ixy>0</ixy>
              <ixz>0</ixz>
              <iyy>0.000273104</iyy>
              <iyz>0</iyz>
              <izz>0.000274004</izz>
            </inertia>
          </inertial>
          <collision name='rotor_1_collision'>
            <pose>0 0 0 0 0 0</pose>
            <geometry>
              <cylinder>
                <length>0.005</length>
                <radius>0.128</radius>
              </cylinder>
            </geometry>
            <surface>
              <contact>
                <ode/>
              </contact>
              <friction>
                <ode/>
              </friction>
            </surface>
          </collision>
          <visual name='rotor_1_visual'>
            <pose>0.00187896 -0.242705 -0.0822169 0 0 0</pose>
            <geometry>
              <mesh>
                <scale>0.001 0.001 0.001</scale>
                <uri>model://typhoon_2cam/meshes/prop_cw_assembly_remeshed_v3.stl</uri>
              </mesh>
            </geometry>
            <material>
              <script>
                <name>Gazebo/DarkGrey</name>
                <uri>file://media/materials/scripts/gazebo.material</uri>
              </script>
            </material>
          </visual>
          <gravity>1</gravity>
          <velocity_decay/>
          <self_collide>0</self_collide>
        </link>
        <joint name='rotor_1_joint' type='revolute'>
          <child>rotor_1</child>
          <parent>base_link</parent>
          <axis>
            <xyz>0.0486 0.0811 1.8976</xyz>
            <limit>
              <lower>-1e+16</lower>
              <upper>1e+16</upper>
              <effort>10</effort>
              <velocity>-1</velocity>
            </limit>
            <dynamics>
              <damping>0.005</damping>
            </dynamics>
            <use_parent_model_frame>1</use_parent_model_frame>
          </axis>
          <physics>
            <ode>
              <implicit_spring_damper>1</implicit_spring_damper>
            </ode>
          </physics>
        </joint>
    
        <link name='rotor_5'>
          <pose>-0.00187896 -0.242705 0.0822169 0 0 -2.09439510239</pose>
          <inertial>
            <pose>0 0 0 0 0 0</pose>
            <mass>0.005</mass>
            <inertia>
              <ixx>9.75e-07</ixx>
              <ixy>0</ixy>
              <ixz>0</ixz>
              <iyy>0.000273104</iyy>
              <iyz>0</iyz>
              <izz>0.000274004</izz>
            </inertia>
          </inertial>
          <collision name='rotor_5_collision'>
            <pose>0 0 0 0 0 0</pose>
            <geometry>
              <cylinder>
                <length>0.005</length>
                <radius>0.128</radius>
              </cylinder>
            </geometry>
            <surface>
              <contact>
                <ode/>
              </contact>
              <friction>
                <ode/>
              </friction>
            </surface>
          </collision>
          <visual name='rotor_5_visual'>
            <pose>-0.211396 -0.119762 -0.082219 0 0 0</pose>
            <geometry>
              <mesh>
                <scale>0.001 0.001 0.001</scale>
                <uri>model://typhoon_2cam/meshes/prop_ccw_assembly_remeshed_v3.stl</uri>
              </mesh>
            </geometry>
            <material>
              <script>
                <name>Gazebo/Blue</name>
                <uri>file://media/materials/scripts/gazebo.material</uri>
              </script>
            </material>
          </visual>
          <gravity>1</gravity>
          <velocity_decay/>
          <self_collide>0</self_collide>
        </link>
        <joint name='rotor_5_joint' type='revolute'>
          <child>rotor_5</child>
          <parent>base_link</parent>
          <axis>
            <xyz>-0.033996 -0.0006 0.68216</xyz>
            <limit>
              <lower>-1e+16</lower>
              <upper>1e+16</upper>
              <effort>10</effort>
              <velocity>-1</velocity>
            </limit>
            <dynamics>
              <damping>0.005</damping>
            </dynamics>
            <use_parent_model_frame>1</use_parent_model_frame>
          </axis>
          <physics>
            <ode>
              <implicit_spring_damper>1</implicit_spring_damper>
            </ode>
          </physics>
        </joint>
        <link name='rotor_2'>
          <pose>-0.209396 -0.122762 0.082219 0 0 2.09439510239</pose>
          <inertial>
            <pose>0 0 0 0 0 0</pose>
            <mass>0.005</mass>
            <inertia>
              <ixx>9.75e-07</ixx>
              <ixy>0</ixy>
              <ixz>0</ixz>
              <iyy>0.000273104</iyy>
              <iyz>0</iyz>
              <izz>0.000274004</izz>
            </inertia>
          </inertial>
          <collision name='rotor_2_collision'>
            <pose>0 0 0 0 0 0</pose>
            <geometry>
              <cylinder>
                <length>0.005</length>
                <radius>0.128</radius>
              </cylinder>
            </geometry>
            <surface>
              <contact>
                <ode/>
              </contact>
              <friction>
                <ode/>
              </friction>
            </surface>
          </collision>
          <visual name='rotor_2_visual'>
            <pose>0.00187896 -0.242705 -0.0822169 0 0 0</pose>
            <geometry>
              <mesh>
                <scale>0.001 0.001 0.001</scale>
                <uri>model://typhoon_2cam/meshes/prop_cw_assembly_remeshed_v3.stl</uri>
              </mesh>
            </geometry>
            <material>
              <script>
                <name>Gazebo/DarkGrey</name>
                <uri>file://media/materials/scripts/gazebo.material</uri>
              </script>
            </material>
          </visual>
          <gravity>1</gravity>
          <velocity_decay/>
          <self_collide>0</self_collide>
        </link>
        <joint name='rotor_2_joint' type='revolute'>
          <child>rotor_2</child>
          <parent>base_link</parent>
          <axis>
            <xyz>0.0404 -0.0876 1.8976</xyz>
            <limit>
              <lower>-1e+16</lower>
              <upper>1e+16</upper>
              <effort>10</effort>
              <velocity>-1</velocity>
            </limit>
            <dynamics>
              <damping>0.005</damping>
            </dynamics>
            <use_parent_model_frame>1</use_parent_model_frame>
          </axis>
          <physics>
            <ode>
              <implicit_spring_damper>1</implicit_spring_damper>
            </ode>
          </physics>
        </joint>
    
    <!--
        <include>
          <uri>model://sonar</uri>
        </include>
        <joint name="sonar_joint" type="revolute">
          <child>sonar_model::link</child>
          <parent>typhoon_2cam::base_link</parent>
          <axis>
            <xyz>0 0 1</xyz>
            <limit>
              <upper>0</upper>
              <lower>0</lower>
            </limit>
          </axis>
        </joint>
    -->
    
    
        <plugin name='rosbag' filename='libgazebo_multirotor_base_plugin.so'>
          <robotNamespace></robotNamespace>
          <linkName>base_link</linkName>
          <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
        </plugin>
    
        <plugin name='front_right_motor_model' filename='libgazebo_motor_model.so'>
          <robotNamespace></robotNamespace>
          <jointName>rotor_0_joint</jointName>
          <linkName>rotor_0</linkName>
          <turningDirection>ccw</turningDirection>
          <timeConstantUp>0.0125</timeConstantUp>
          <timeConstantDown>0.025</timeConstantDown>
          <maxRotVelocity>1500</maxRotVelocity>
          <motorConstant>8.54858e-06</motorConstant>
          <momentConstant>0.06</momentConstant>
          <commandSubTopic>/gazebo/command/motor_speed</commandSubTopic>
          <motorNumber>4</motorNumber>
          <rotorDragCoefficient>0.000806428</rotorDragCoefficient>
          <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
          <motorSpeedPubTopic>/motor_speed/4</motorSpeedPubTopic>
          <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
          <!--
          <joint_control_pid>
            <p>0.1</p>
            <i>0</i>
            <d>0</d>
            <iMax>0</iMax>
            <iMin>0</iMin>
            <cmdMax>3</cmdMax>
            <cmdMin>-3</cmdMin>
          </joint_control_pid>
          -->
        </plugin>
        <plugin name='back_left_motor_model' filename='libgazebo_motor_model.so'>
          <robotNamespace></robotNamespace>
          <jointName>rotor_1_joint</jointName>
          <linkName>rotor_1</linkName>
          <turningDirection>cw</turningDirection>
          <timeConstantUp>0.0125</timeConstantUp>
          <timeConstantDown>0.025</timeConstantDown>
          <maxRotVelocity>1500</maxRotVelocity>
          <motorConstant>8.54858e-06</motorConstant>
          <momentConstant>0.06</momentConstant>
          <commandSubTopic>/gazebo/command/motor_speed</commandSubTopic>
          <motorNumber>5</motorNumber>
          <rotorDragCoefficient>0.000806428</rotorDragCoefficient>
          <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
          <motorSpeedPubTopic>/motor_speed/5</motorSpeedPubTopic>
          <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
          <!--
          <joint_control_pid>
            <p>0.1</p>
            <i>0</i>
            <d>0</d>
            <iMax>0</iMax>
            <iMin>0</iMin>
            <cmdMax>3</cmdMax>
            <cmdMin>-3</cmdMin>
          </joint_control_pid>
          -->
        </plugin>
        <plugin name='front_left_motor_model' filename='libgazebo_motor_model.so'>
          <robotNamespace></robotNamespace>
          <jointName>rotor_2_joint</jointName>
          <linkName>rotor_2</linkName>
          <turningDirection>cw</turningDirection>
          <timeConstantUp>0.0125</timeConstantUp>
          <timeConstantDown>0.025</timeConstantDown>
          <maxRotVelocity>1500</maxRotVelocity>
          <motorConstant>8.54858e-06</motorConstant>
          <momentConstant>0.06</momentConstant>
          <commandSubTopic>/gazebo/command/motor_speed</commandSubTopic>
          <motorNumber>2</motorNumber>
          <rotorDragCoefficient>0.000806428</rotorDragCoefficient>
          <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
          <motorSpeedPubTopic>/motor_speed/2</motorSpeedPubTopic>
          <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
          <!--
          <joint_control_pid>
            <p>0.1</p>
            <i>0</i>
            <d>0</d>
            <iMax>0</iMax>
            <iMin>0</iMin>
            <cmdMax>3</cmdMax>
            <cmdMin>-3</cmdMin>
          </joint_control_pid>
          -->
        </plugin>
        <plugin name='back_right_motor_model' filename='libgazebo_motor_model.so'>
          <robotNamespace></robotNamespace>
          <jointName>rotor_3_joint</jointName>
          <linkName>rotor_3</linkName>
          <turningDirection>ccw</turningDirection>
          <timeConstantUp>0.0125</timeConstantUp>
          <timeConstantDown>0.025</timeConstantDown>
          <maxRotVelocity>1500</maxRotVelocity>
          <motorConstant>8.54858e-06</motorConstant>
          <momentConstant>0.06</momentConstant>
          <commandSubTopic>/gazebo/command/motor_speed</commandSubTopic>
          <motorNumber>3</motorNumber>
          <rotorDragCoefficient>0.000806428</rotorDragCoefficient>
          <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
          <motorSpeedPubTopic>/motor_speed/3</motorSpeedPubTopic>
          <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
          <!--
          <joint_control_pid>
            <p>0.1</p>
            <i>0</i>
            <d>0</d>
            <iMax>0</iMax>
            <iMin>0</iMin>
            <cmdMax>3</cmdMax>
            <cmdMin>-3</cmdMin>
          </joint_control_pid>
          -->
        </plugin>
        <plugin name='back_left_motor_model' filename='libgazebo_motor_model.so'>
          <robotNamespace></robotNamespace>
          <jointName>rotor_4_joint</jointName>
          <linkName>rotor_4</linkName>
          <turningDirection>cw</turningDirection>
          <timeConstantUp>0.0125</timeConstantUp>
          <timeConstantDown>0.025</timeConstantDown>
          <maxRotVelocity>1500</maxRotVelocity>
          <motorConstant>8.54858e-06</motorConstant>
          <momentConstant>0.06</momentConstant>
          <commandSubTopic>/gazebo/command/motor_speed</commandSubTopic>
          <motorNumber>0</motorNumber>
          <rotorDragCoefficient>0.000806428</rotorDragCoefficient>
          <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
          <motorSpeedPubTopic>/motor_speed/0</motorSpeedPubTopic>
          <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
          <!--
          <joint_control_pid>
            <p>0.1</p>
            <i>0</i>
            <d>0</d>
            <iMax>0</iMax>
            <iMin>0</iMin>
            <cmdMax>3</cmdMax>
            <cmdMin>-3</cmdMin>
          </joint_control_pid>
          -->
        </plugin>
        <plugin name='front_left_motor_model' filename='libgazebo_motor_model.so'>
          <robotNamespace></robotNamespace>
          <jointName>rotor_5_joint</jointName>
          <linkName>rotor_5</linkName>
          <turningDirection>ccw</turningDirection>
          <timeConstantUp>0.0125</timeConstantUp>
          <timeConstantDown>0.025</timeConstantDown>
          <maxRotVelocity>1500</maxRotVelocity>
          <motorConstant>8.54858e-06</motorConstant>
          <momentConstant>0.06</momentConstant>
          <commandSubTopic>/gazebo/command/motor_speed</commandSubTopic>
          <motorNumber>1</motorNumber>
          <rotorDragCoefficient>0.000806428</rotorDragCoefficient>
          <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
          <motorSpeedPubTopic>/motor_speed/1</motorSpeedPubTopic>
          <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
          <!--
          <joint_control_pid>
            <p>0.1</p>
            <i>0</i>
            <d>0</d>
            <iMax>0</iMax>
            <iMin>0</iMin>
            <cmdMax>3</cmdMax>
            <cmdMin>-3</cmdMin>
          </joint_control_pid>
          -->
        </plugin>
        <plugin name="gps_plugin" filename="libgazebo_gps_plugin.so">
            <robotNamespace></robotNamespace>
    
            <gpsNoise>false</gpsNoise>
    
    
        </plugin>
        <plugin name='magnetometer_plugin' filename='libgazebo_magnetometer_plugin.so'>
          <robotNamespace/>
          <pubRate>20</pubRate>
          <!--<noiseDensity>0.0004</noiseDensity>
          <randomWalk>6.4e-06</randomWalk>-->
          <noiseDensity>0.0</noiseDensity>
          <randomWalk>0.0</randomWalk>
          <biasCorrelationTime>600</biasCorrelationTime>
          <magTopic>/mag</magTopic>
        </plugin>
        <plugin name='barometer_plugin' filename='libgazebo_barometer_plugin.so'>
          <robotNamespace/>
          <pubRate>10</pubRate>
          <baroTopic>/baro</baroTopic>
        </plugin>
        <plugin name='mavlink_interface' filename='libgazebo_mavlink_interface.so'>
          <robotNamespace></robotNamespace>
          <imuSubTopic>/imu</imuSubTopic>
          <gpsSubTopic>/gps</gpsSubTopic>
          <magSubTopic>/mag</magSubTopic>
          <baroSubTopic>/baro</baroSubTopic>
          <lidarSubTopic>/link/lidar</lidarSubTopic>        # JS 
        <!--  <lidarSubTopic>/sf10a/link/lidar</lidarSubTopic>      # JS-->
          <mavlink_addr>INADDR_ANY</mavlink_addr>
          <mavlink_udp_port>14560</mavlink_udp_port>
          <serialEnabled>false</serialEnabled>
          <serialDevice>/dev/ttyACM0</serialDevice>
          <baudRate>921600</baudRate>
          <qgc_addr>INADDR_ANY</qgc_addr>
          <qgc_udp_port>14550</qgc_udp_port>
          <sdk_addr>INADDR_ANY</sdk_addr>
          <sdk_udp_port>14540</sdk_udp_port>
          <hil_mode>false</hil_mode>
          <hil_state_level>false</hil_state_level>
          <enable_lockstep>true</enable_lockstep>      # JS
          <use_tcp>true</use_tcp>
          <motorSpeedCommandPubTopic>/gazebo/command/motor_speed</motorSpeedCommandPubTopic>
          <control_channels>
            <channel name="rotor0">
              <input_index>0</input_index>
              <input_offset>0</input_offset>
              <input_scaling>1500</input_scaling>
              <zero_position_disarmed>0</zero_position_disarmed>
              <zero_position_armed>100</zero_position_armed>
              <joint_control_type>velocity</joint_control_type>
              <!-- gazebo_motor_model has the joint_control_pid active in this model
              <joint_control_pid>
                <p>0.1</p>
                <i>0</i>
                <d>0</d>
                <iMax>0</iMax>
                <iMin>0</iMin>
                <cmdMax>3</cmdMax>
                <cmdMin>-3</cmdMin>
              </joint_control_pid>
              -->
              <joint_name>rotor_4_joint</joint_name>
            </channel>
            <channel name="rotor1">
              <input_index>1</input_index>
              <input_offset>0</input_offset>
              <input_scaling>1500</input_scaling>
              <zero_position_disarmed>0</zero_position_disarmed>
              <zero_position_armed>100</zero_position_armed>
              <joint_control_type>velocity</joint_control_type>
              <!-- gazebo_motor_model has the joint_control_pid active in this model
              <joint_control_pid>
                <p>0.1</p>
                <i>0</i>
                <d>0</d>
                <iMax>0</iMax>
                <iMin>0</iMin>
                <cmdMax>3</cmdMax>
                <cmdMin>-3</cmdMin>
              </joint_control_pid>
              -->
              <joint_name>rotor_5_joint</joint_name>
            </channel>
            <channel name="rotor2">
              <input_index>2</input_index>
              <input_offset>0</input_offset>
              <input_scaling>1500</input_scaling>
              <zero_position_disarmed>0</zero_position_disarmed>
              <zero_position_armed>100</zero_position_armed>
              <joint_control_type>velocity</joint_control_type>
              <!-- gazebo_motor_model has the joint_control_pid active in this model
              <joint_control_pid>
                <p>0.1</p>
                <i>0</i>
                <d>0</d>
                <iMax>0</iMax>
                <iMin>0</iMin>
                <cmdMax>3</cmdMax>
                <cmdMin>-3</cmdMin>
              </joint_control_pid>
              -->
              <joint_name>rotor_2_joint</joint_name>
            </channel>
            <channel name="rotor3">
              <input_index>3</input_index>
              <input_offset>0</input_offset>
              <input_scaling>1500</input_scaling>
              <zero_position_disarmed>0</zero_position_disarmed>
              <zero_position_armed>100</zero_position_armed>
              <joint_control_type>velocity</joint_control_type>
              <!-- gazebo_motor_model has the joint_control_pid active in this model
              <joint_control_pid>
                <p>0.1</p>
                <i>0</i>
                <d>0</d>
                <iMax>0</iMax>
                <iMin>0</iMin>
                <cmdMax>3</cmdMax>
                <cmdMin>-3</cmdMin>
              </joint_control_pid>
              -->
              <joint_name>rotor_3_joint</joint_name>
            </channel>
            <channel name="rotor4">
              <input_index>4</input_index>
              <input_offset>0</input_offset>
              <input_scaling>1500</input_scaling>
              <zero_position_disarmed>0</zero_position_disarmed>
              <zero_position_armed>100</zero_position_armed>
              <joint_control_type>velocity</joint_control_type>
              <!-- gazebo_motor_model has the joint_control_pid active in this model
              <joint_control_pid>
                <p>0.1</p>
                <i>0</i>
                <d>0</d>
                <iMax>0</iMax>
                <iMin>0</iMin>
                <cmdMax>3</cmdMax>
                <cmdMin>-3</cmdMin>
              </joint_control_pid>
              -->
              <joint_name>rotor_0_joint</joint_name>
            </channel>
            <channel name="rotor5">
              <input_index>5</input_index>
              <input_offset>0</input_offset>
              <input_scaling>1500</input_scaling>
              <zero_position_disarmed>0</zero_position_disarmed>
              <zero_position_armed>100</zero_position_armed>
              <joint_control_type>velocity</joint_control_type>
              <!-- gazebo_motor_model has the joint_control_pid active in this model
              <joint_control_pid>
                <p>0.1</p>
                <i>0</i>
                <d>0</d>
                <iMax>0</iMax>
                <iMin>0</iMin>
                <cmdMax>3</cmdMax>
                <cmdMin>-3</cmdMin>
              </joint_control_pid>
              -->
              <joint_name>rotor_1_joint</joint_name>
            </channel>
            <channel name="gimbal_roll">
              <input_index>6</input_index>
              <input_offset>0</input_offset>
              <input_scaling>-3.1415</input_scaling>
              <zero_position_disarmed>0</zero_position_disarmed>
              <zero_position_armed>0</zero_position_armed>
              <joint_control_type>position_gztopic</joint_control_type>
              <gztopic>/gimbal_roll_cmd</gztopic>
              <joint_name>typhoon_2cam::cgo3_camera_joint</joint_name>
            </channel>
            <channel name="gimbal_pitch">
              <input_index>7</input_index>
              <input_offset>0</input_offset>
              <input_scaling>3.1415</input_scaling>
              <zero_position_disarmed>0</zero_position_disarmed>
              <zero_position_armed>0</zero_position_armed>
              <joint_control_type>position_gztopic</joint_control_type>
              <gztopic>/gimbal_pitch_cmd</gztopic>
              <joint_name>typhoon_2cam::cgo3_camera_joint</joint_name>
            </channel>
            <channel name="gimbal_yaw">
              <input_index>8</input_index>
              <input_offset>0</input_offset>
              <input_scaling>-3.1415</input_scaling>
              <zero_position_disarmed>0</zero_position_disarmed>
              <zero_position_armed>0</zero_position_armed>
              <joint_control_type>position_gztopic</joint_control_type>
              <gztopic>/gimbal_yaw_cmd</gztopic>
              <joint_name>typhoon_2cam::cgo3_vertical_arm_joint</joint_name>
            </channel>
            <channel name="left_leg">
              <input_index>9</input_index>
              <input_offset>1</input_offset>
              <input_scaling>0.5</input_scaling>
              <zero_position_disarmed>0</zero_position_disarmed>
              <zero_position_armed>0</zero_position_armed>
              <joint_control_type>position</joint_control_type>
              <joint_control_pid>
                <p>3.5</p>
                <i>0.5</i>
                <d>0</d>
                <iMax>4</iMax>
                <iMin>-4</iMin>
                <cmdMax>6</cmdMax>
                <cmdMin>-6</cmdMin>
              </joint_control_pid>
              <joint_name>left_leg_joint</joint_name>
            </channel>
            <channel name="right_leg">
              <input_index>10</input_index>
              <input_offset>1</input_offset>
              <input_scaling>0.5</input_scaling>
              <zero_position_disarmed>0</zero_position_disarmed>
              <zero_position_armed>0</zero_position_armed>
              <joint_control_type>position</joint_control_type>
              <joint_control_pid>
                <p>3.5</p>
                <i>0.5</i>
                <d>0</d>
                <iMax>4</iMax>
                <iMin>-4</iMin>
                <cmdMax>6</cmdMax>
                <cmdMin>-6</cmdMin>
              </joint_control_pid>
              <joint_name>right_leg_joint</joint_name>
            </channel>
          </control_channels>
        </plugin>
        <static>0</static>
        <plugin name='gazebo_imu_plugin' filename='libgazebo_imu_plugin.so'>
          <robotNamespace></robotNamespace>
          <linkName>typhoon_2cam/imu_link</linkName>
          <imuTopic>/imu</imuTopic>
    
          <gyroscopeNoiseDensity>0.0</gyroscopeNoiseDensity>
          <gyroscopeRandomWalk>0.0</gyroscopeRandomWalk>
          <gyroscopeTurnOnBiasSigma>0.0</gyroscopeTurnOnBiasSigma>
          <accelerometerNoiseDensity>0.0</accelerometerNoiseDensity>
          <accelerometerRandomWalk>0.0</accelerometerRandomWalk>
          <accelerometerTurnOnBiasSigma>0.0</accelerometerTurnOnBiasSigma>
    
    <!--
          <gyroscopeNoiseDensity>0.0003394</gyroscopeNoiseDensity>
          <gyroscopeRandomWalk>3.8785e-05</gyroscopeRandomWalk>
          <gyroscopeBiasCorrelationTime>1000.0</gyroscopeBiasCorrelationTime>
          <gyroscopeTurnOnBiasSigma>0.0087</gyroscopeTurnOnBiasSigma>
          <accelerometerNoiseDensity>0.004</accelerometerNoiseDensity>
          <accelerometerRandomWalk>0.006</accelerometerRandomWalk>
          <accelerometerBiasCorrelationTime>300.0</accelerometerBiasCorrelationTime>
          <accelerometerTurnOnBiasSigma>0.196</accelerometerTurnOnBiasSigma>
    -->
    
    
        </plugin>
        <plugin name='gimbal_controller' filename='libgazebo_gimbal_controller_plugin.so'>
          <joint_yaw>typhoon_2cam::cgo3_vertical_arm_joint</joint_yaw>
          <joint_roll>typhoon_2cam::cgo3_horizontal_arm_joint</joint_roll>
          <joint_pitch>typhoon_2cam::cgo3_camera_joint</joint_pitch>
          <control_gimbal_channels>
    
            <channel>
              <joint_control_pid>
                <p>0.5</p>
                <i>0.01245</i>
                <d>0.01</d>
                <iMax>0</iMax>
                <iMin>0</iMin>
                <cmdMax>1.0</cmdMax>
                <cmdMin>-1.0</cmdMin>
              </joint_control_pid>
              <joint_axis>joint_yaw</joint_axis>
            </channel>
    <!--
            <channel>
              <joint_control_pid>
                <p>0.5</p>
                <i>0.01245</i>
                <d>0.01</d>
                <iMax>0</iMax>
                <iMin>0</iMin>
                <cmdMax>1.0</cmdMax>
                <cmdMin>-1.0</cmdMin>
              </joint_control_pid>
              <joint_axis>joint_yaw</joint_axis>
            </channel>
    
      
            <channel>
              <joint_control_pid>
                <p>0.8</p>
                <i>0.035</i>
                <d>0.02</d>
                <iMax>0</iMax>
                <iMin>0</iMin>
                <cmdMax>0.3</cmdMax>
                <cmdMin>-0.3</cmdMin>
              </joint_control_pid>
              <joint_axis>joint_roll</joint_axis>
            </channel>
    
            ## untouched
            <channel>
              <joint_control_pid>
                <p>2.068</p>
                <i>0.01245</i>
                <d>0.01</d>
                <iMax>0</iMax>
                <iMin>0</iMin>
                <cmdMax>1.0</cmdMax>
                <cmdMin>-1.0</cmdMin>
              </joint_control_pid>
              <joint_axis>joint_yaw</joint_axis>
            </channel>
            <channel>
              <joint_control_pid>
                <p>2.068</p>
                <i>0.01245</i>
                <d>0.01</d>
                <iMax>0</iMax>
                <iMin>0</iMin>
                <cmdMax>0.3</cmdMax>
                <cmdMin>-0.3</cmdMin>
              </joint_control_pid>
              <joint_axis>joint_roll</joint_axis>
            </channel>
            <channel>
              <joint_control_pid>
                <p>2.068</p>
                <i>0.01245</i>
                <d>0.01</d>
                <iMax>0</iMax>
                <iMin>0</iMin>
                <cmdMax>0.3</cmdMax>
                <cmdMin>-0.3</cmdMin>
              </joint_control_pid>
              <joint_axis>joint_pitch</joint_axis>
            </channel>
    
    -->
            <channel>
              <joint_control_pid>
                <p>15</p>
                <i>0.01245</i>
                <d>0.01</d>
                <iMax>0.5</iMax>
                <iMin>-0.5</iMin>
                <cmdMax>0.9</cmdMax>
                <cmdMin>-0.9</cmdMin>
              </joint_control_pid>
              <joint_axis>joint_roll</joint_axis>
            </channel>
    
            <channel>
              <joint_control_pid>
                <p>7.5</p>
                <i>0.001245</i>
                <d>0.01</d>
                <iMax>0.5</iMax>
                <iMin>-0.5</iMin>
                <cmdMax>0.9</cmdMax>
                <cmdMin>-0.9</cmdMin>
              </joint_control_pid>
              <joint_axis>joint_pitch</joint_axis>
            </channel>
    
          </control_gimbal_channels>
          <gimbal_imu>camera_imu</gimbal_imu>
        </plugin>
    
    
    
        JS <!--  
        <include>
          <uri>model://sf10a</uri>
          <pose>0.08 0 -0.04 0 0 0</pose>
        </include>
        <joint name="lidar_joint" type="fixed">
          <child>sf10a::link</child>      
          <parent>base_link</parent>
          <axis>
            <xyz>0 0 1</xyz>
            <limit>
              <upper>0</upper>
              <lower>0</lower>
            </limit>
          </axis>
        </joint>
    
    
    -->
    
    THis lidar was too short range
    
        <include>
          <uri>model://lidar_long</uri>
          <pose>0.08 0 -0.08 0 3.1415 0</pose>
        </include>
    
    <!---->
        <joint name="lidar_joint" type="fixed">
          <child>lidar_long::link</child>      
          <parent>base_link</parent>
          <axis>
            <xyz>0 0 1</xyz>
            <limit>
              <upper>0</upper>
              <lower>0</lower>
            </limit>
          </axis>
        </joint>
    
    
        <plugin name="p3d_base_controller" filename="libgazebo_ros_p3d.so">
            <bodyName>cgo3_camera_link</bodyName> 
            <topicName>camera_gt</topicName> 
            <!--<frameName>test_link</frameName> -->  leave blank so that the world frame is used here
            <updateRate>100.0</updateRate>   
        </plugin>
    
        <plugin name="p3d_base_controller" filename="libgazebo_ros_p3d.so">
            <bodyName>base_link</bodyName> 
            <topicName>body_ground_truth</topicName> 
            <!--<frameName>test_link</frameName> -->  leave blank so that the world frame is used here
            <updateRate>100.0</updateRate>   
        </plugin>
    
        <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
          <alwaysOn>true</alwaysOn>
          <bodyName>cgo3_camera_link</bodyName>
          <topicName>camera_imu_ros</topicName>
          <serviceName>cam_imu_service</serviceName>
          <gaussianNoise>0.0</gaussianNoise>
          <updateRate>100.0</updateRate>
        </plugin>
      </model>
    </sdf>
    """


    full_str = preliminary_str + main_camera_bit + final_str

    # print (full_str)
    # print main_camera_bit

    # savefile
    with open(save_dir, 'w') as filehandle:
        filehandle.write(full_str)

if __name__ == '__main__':



    parser = argparse.ArgumentParser(description="This parses instructions for autorun.")
    # todo - can this be combined with argparse cx?
    parser.add_argument('-p', '--pitch_offset_deg', type=float, default=0, help='pitch angle w.r.t the ground - 0 is at'
                        ' the ground, 90 is horizontal in the direction of motion')

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    print (args)

    generate_singlecam_sdf_function(pitch_angle_deg=args.pitch_offset_deg)