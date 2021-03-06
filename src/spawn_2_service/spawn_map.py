#! /usr/bin/env python3
import sys
import rospy
import rospkg
from initial_db import Initial
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

# # settings
# method: single_sys, single_custome, multi_sys, multi_custome, multi_json
# stype: db, sql
stype = ''
path_unit = 0.5
method = 'multi_json'

# # input args
# args: from_database, warehouse, path_unit, method
for i in range(0, len(sys.argv)):
    if sys.argv[i] == '-from_database':
        if sys.argv[i+1] == 'True':
            method = 'multi_sys'

    if sys.argv[i] == '-warehouse':
        warehouse = sys.argv[i+1]

    if sys.argv[i] == '-path_unit':
        path_unit = float(sys.argv[i+1])

    if sys.argv[i] == '-method':
        method = sys.argv[i+1]


# get data
init_obj = Initial(method, stype, path_unit)
robots_initial = init_obj.robots_initial
robots_count = init_obj.robots_count
obst = init_obj.obst

# ros init
rospy.init_node("spawn_multi_robots")
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('init_sim')

# spawn_urdf_model service
print("Waiting for gazebo spawn_urdf_model services...")
rospy.wait_for_service("gazebo/spawn_urdf_model")
print("Got it.")
spawn_robots = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)

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
    spawn_robots(model_name, robot_file, robot_namespace,
                 robots_initial[rd], reference_frame)
    rospy.sleep(0.5)

# spawn obstacles

# spawn_sdf_model sercice
print("Waiting for gazebo spawn_sdf_model services...")
rospy.wait_for_service("gazebo/spawn_sdf_model")
print("Got it.")
spawn_obst = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

# model file
with open('/home/morteza/catkin_ws/src/init_sim/models/cylinder1/model2.sdf', 'r') as file2:
    model_file = file2.read()

# spawn models
name = 'cylinder'
model_namespace = ''
reference_frame = 'world'

for i in range(obst['count']):
    pose = Point(x=obst['x'][i], y=obst['y'][i], z=0)
    init_pose = Pose(pose, Quaternion())
    model_name = name+str(i)
    spawn_obst(model_name, model_file, model_namespace,
               init_pose, reference_frame)
    rospy.sleep(0.5)
