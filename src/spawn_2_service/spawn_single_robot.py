#! /usr/bin/env python3

import rospy
import rospkg
from initial_db import Initial
from gazebo_msgs.srv import SpawnModel


# get data
# single_sys single_custome multi_sys multi_custome
init_obj = Initial('single_custome', '')
robot_initial = init_obj.robots_initial
id = init_obj.id

# ros init
rospy.init_node("spawn_robot")
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('init_sim')

# spawn service
print("Waiting for gazebo services...")
rospy.wait_for_service("gazebo/spawn_urdf_model")
print("Got it.")
spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)

# robot file
with open(pkg_path+'/models/TB3_model.urdf', 'r') as file:
    robot_file = file.read()

# spawn robot
robot_namespace = '/t1'
model_name = 'robot1'
reference_frame = 'world'
rospy.set_param('tf_prefix', '/t'+'1')
spawn_model(model_name, robot_file, robot_namespace,
            robot_initial, reference_frame)
