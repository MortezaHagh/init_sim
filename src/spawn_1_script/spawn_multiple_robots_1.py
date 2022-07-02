#! /usr/bin/env python3

import rospy
from initial_db import Initial
from spawn_robot import SpawnModel


# # get data
# Initial(method, stype)
# method (single_sys single_custome multi_sys multi_custome)
# stype(sb, sql)
init_obj = Initial('multi_custome', '')
robots_initial = init_obj.robots_initial
robots_count = init_obj.robots_count

# start node
rospy.init_node('spawn_multi_robots')

# robot model
with open('/home/morteza/catkin_ws/src/init_sim/models/TB3_model.urdf', 'r') as file:
    robot_file = file.read()

# spawn robots
for rd in range(robots_count):
    id = init_obj.id[rd]
    model_name = 'turtlebot3_'+str(id)
    param_name = 'robot_description_'+str(id)
    rospy.set_param(param_name, robot_file)
    rospy.set_param('tf_prefix', '/t'+str(id))
    robot_namespace = '/t'+str(id)
    smc = SpawnModel(model_name, param_name,
                     robots_initial[rd], robot_namespace)
    smc.get_parameters()
    smc.callSpawnService()
    rospy.sleep(0.5)

# on shutdown hook
if smc.bond:
    rospy.on_shutdown(smc.callDeleteService)
    rospy.spin()
