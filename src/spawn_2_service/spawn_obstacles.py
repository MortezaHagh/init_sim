#! /usr/bin/env python3

# import tf
import json
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion


rospy.init_node("spawn_obstacles")

# custom map
obst_count = 3
obst_x = [4, 5, 6]
obst_y = [1, 2, 3]

# # map from json
# with open('/home/morteza/catkin_ws/src/my_worlds/src/spawn_2_service/jsonObst.json', 'r') as f:
#     obstacles = json.load(f)

# obst_count = obstacles['count']
# obst_x = obstacles['x']
# obst_y = obstacles['y']

# spawn sercice
print("Waiting for gazebo services...")
rospy.wait_for_service("gazebo/spawn_sdf_model")
print("Got it.")
spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

# model file
with open('/home/morteza/catkin_ws/src/my_worlds/models/cylinder1/model.sdf', 'r') as file:
    model_file = file.read()

# spawn models
name = 'cylinder'
robot_namespace = ''
reference_frame = 'world'

for i in range(obst_count):
    pose = Point(x=obst_x[i], y=obst_y[i], z=0)
    init_pose = Pose(pose, Quaternion())
    model_name = name+str(i)
    spawn_model(model_name, model_file, robot_namespace,
                init_pose, reference_frame)
