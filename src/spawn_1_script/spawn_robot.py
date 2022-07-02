#! /usr/bin/env python3

import re
import os
import sys
import rospy
from std_srvs.srv import Empty
import tf.transformations as tft
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from gazebo_ros import gazebo_interface
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Wrench

model_database_template = """<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://MODEL_NAME</uri>
    </include>
  </world>
</sdf>"""


class SpawnModel():
    def __init__(self, model_name, param_name, robot_data, robot_namespace):
        self.initial_xyz = [0, 0, 0]
        self.initial_rpy = [0, 0, 0]
        self.initial_q = [0, 0, 0, 1]
        self.file_name = ""
        self.param_name = ""
        self.database_name = ""
        self.model_name = ""
        if robot_namespace == '':
            self.robot_namespace = rospy.get_namespace()
        else:
            self.robot_namespace = robot_namespace
        self.gazebo_namespace = "/gazebo"
        self.reference_frame = ""
        self.unpause_physics = False
        self.wait_for_model = ""
        self.wait_for_model_exists = False
        self.urdf_format = False
        self.sdf_format = False
        self.joint_names = []
        self.joint_positions = []
        self.package_to_model = False
        self.bond = False

        #
        self.model_name = model_name
        self.param_name = param_name
        self.robot_data = robot_data
        self.urdf_format = True

    # get initial pose
    def get_parameters(self):
        self.initial_xyz = self.robot_data['xyz']
        self.initial_rpy = self.robot_data['rpy']

    def checkForModel(self, model):
        for n in model.name:
            if n == self.wait_for_model:
                self.wait_for_model_exists = True

    # Generate a blank SDF file with an include for the model from the model database

    def createDatabaseCode(self, database_name):
        return model_database_template.replace("MODEL_NAME", database_name)

    def callSpawnService(self):

        # wait for model to exist

        if not self.wait_for_model == "":
            rospy.Subscriber(
                "%s/model_states" % (self.gazebo_namespace), ModelStates, self.checkForModel)
            r = rospy.Rate(10)
            while not rospy.is_shutdown() and not self.wait_for_model_exists:
                r.sleep()

        if rospy.is_shutdown():
            sys.exit(0)

        if self.file_name != "":
            rospy.loginfo("Loading model XML from file")
            if os.path.exists(self.file_name):
                if os.path.isdir(self.file_name):
                    rospy.logerr("Error: file name is a path? %s",
                                 self.file_name)
                    sys.exit(0)
                if not os.path.isfile(self.file_name):
                    rospy.logerr("Error: unable to open file %s",
                                 self.file_name)
                    sys.exit(0)
            else:
                rospy.logerr("Error: file does not exist %s", self.file_name)
                sys.exit(0)
            # load file
            f = open(self.file_name, 'r')
            model_xml = f.read()
            if model_xml == "":
                rospy.logerr("Error: file is empty %s", self.file_name)
                sys.exit(0)

        # ROS Parameter
        elif self.param_name != "":
            rospy.loginfo("Loading model XML from ros parameter")
            model_xml = rospy.get_param(self.param_name)
            if model_xml == "":
                rospy.logerr("Error: param does not exist or is empty")
                sys.exit(0)

        # Gazebo Model Database
        elif self.database_name != "":
            rospy.loginfo("Loading model XML from Gazebo Model Database")
            model_xml = self.createDatabaseCode(self.database_name)
            if model_xml == "":
                rospy.logerr("Error: an error occured generating the SDF file")
                sys.exit(0)
        else:
            rospy.logerr(
                "Error: user specified param or filename is an empty string")
            sys.exit(0)

        if self.package_to_model:
            model_xml = re.sub(
                "<\s*mesh\s+filename\s*=\s*([\"|'])package://", "<mesh filename=\g<1>model://", model_xml)

        # setting initial pose
        initial_pose = Pose()
        initial_pose.position.x = self.initial_xyz[0]
        initial_pose.position.y = self.initial_xyz[1]
        initial_pose.position.z = self.initial_xyz[2]
        # convert rpy to quaternion for Pose message
        tmpq = tft.quaternion_from_euler(
            self.initial_rpy[0], self.initial_rpy[1], self.initial_rpy[2])
        q = Quaternion(tmpq[0], tmpq[1], tmpq[2], tmpq[3])
        initial_pose.orientation = q

        # spawn model
        if self.urdf_format:
            success = gazebo_interface.spawn_urdf_model_client(self.model_name, model_xml, self.robot_namespace,
                                                               initial_pose, self.reference_frame, self.gazebo_namespace)
        elif self.sdf_format:
            success = gazebo_interface.spawn_sdf_model_client(self.model_name, model_xml, self.robot_namespace,
                                                              initial_pose, self.reference_frame, self.gazebo_namespace)
        else:
            rospy.logerr(
                "Error: should not be here in spawner helper script, there is a bug")
            sys.exit(0)

        # set model configuration before unpause if user requested
        if len(self.joint_names) != 0:
            try:
                success = gazebo_interface.set_model_configuration_client(self.model_name, self.param_name,
                                                                          self.joint_names, self.joint_positions, self.gazebo_namespace)
            except rospy.ServiceException as e:
                rospy.logerr(
                    "Set model configuration service call failed: %s", e)

        # unpause physics if user requested
        if self.unpause_physics:
            rospy.wait_for_service('%s/unpause_physics' %
                                   (self.gazebo_namespace))
            try:
                unpause_physics = rospy.ServiceProxy(
                    '%s/unpause_physics' % (self.gazebo_namespace), Empty)
                unpause_physics()
            except rospy.ServiceException as e:
                rospy.logerr("Unpause physics service call failed: %s", e)

        return

    def callDeleteService(self):
        try:
            delete_model = rospy.ServiceProxy(
                '%s/delete_model' % (self.gazebo_namespace), DeleteModel)
            delete_model(model_name=self.model_name)
        except rospy.ServiceException as e:
            rospy.logerr("Delete model service call failed: %s", e)
