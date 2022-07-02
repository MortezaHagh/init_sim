#! /usr/bin/env python3

import rospy
from initial_db import Initial
from spawn_robot import SpawnModel

# # get data
# Initial(method, stype)
# method (single_sys single_custome multi_sys multi_custome)
# stype(sb, sql)
init_obj = Initial('single_custome', '')
robot_initial = init_obj.robots_initial
id = init_obj.id

# ros init
rospy.init_node('spawn_single_robot')

# robot model
model_name = 'turtlebot3_'+str(id)
with open('/home/morteza/catkin_ws/src/my_worlds/src/spawn_1_script/TB3_model.urdf', 'r') as file:
    robot_file = file.read()

# spawn parameters
param_name = 'robot_description_'+str(id)
rospy.set_param(param_name, robot_file)
robot_namespace = ''
# rospy.set_param('tf_prefix', '/t1')


# spawn robot
sm = SpawnModel(model_name, param_name, robot_initial, robot_namespace)
sm.get_parameters()
sm.callSpawnService()
if sm.bond:
    rospy.on_shutdown(sm.callDeleteService)
    rospy.spin()
