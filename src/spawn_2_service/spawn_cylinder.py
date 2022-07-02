#! /usr/bin/env python3

# import tf
import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion


rospy.init_node("spawn_cylinder")

# spawn service
print("Waiting for gazebo services...")
# rospy.wait_for_service("gazebo/delete_model")
rospy.wait_for_service("gazebo/spawn_sdf_model")
print("Got it.")

# delete_model = rospy.ServicePoxy("gazebo/delete_model", DeleteModel)
spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

# model file
with open('/home/morteza/catkin_ws/src/init_sim/models/cylinder1/model.sdf', 'r') as file:
    model_file = file.read()

# spawn model
# orient = Quaternion(tf.transformations.quaternion_from_euler(0,0,0))
# init_pose   =   Pose(Point(), orient)
model_namespace = ''
model_name = 'cylinder'
reference_frame = 'world'
init_pose = Pose(Point(x=-1, y=0, z=0), Quaternion())
spawn_model(model_name, model_file, model_namespace,
            init_pose, reference_frame)
