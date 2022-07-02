#! /usr/bin/env python3

import rospy
import rospkg
from initial_db import Initial
from gazebo_msgs.srv import SpawnModel


# # get data
# single_sys single_custome multi_sys multi_custome
init_obj = Initial('multi_custome', '')
robots_initial = init_obj.robots_initial
robots_count = init_obj.robots_count

# ros init
rospy.init_node("spawn_multi_robots")
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

# spawn robots
name = 'robot'
reference_frame = 'world'

for rd in range(robots_count):
    id = init_obj.id[rd]
    model_name = name+str(id)
    robot_namespace = '/t'+str(id)
    rospy.set_param('tf_prefix', '/t'+str(id))
    spawn_model(model_name, robot_file, robot_namespace,
                robots_initial[rd], reference_frame)
    rospy.sleep(0.5)
