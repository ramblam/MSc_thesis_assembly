# Copyright 2020-2023 OpenDR European Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import time
import tf
import rospy
import math
import actionlib
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose
from opendr_control.msg import PickResult, PlaceResult, PickAction, PlaceAction
from franka_msgs.msg import ErrorRecoveryActionGoal

class PickAndPlaceServer(object):
    def __init__(self, rotate_EE, rotate_EE_quat, stop_action, resume_action,
                 move_joint_space, move_cartesian_space,
                 move_cartesian_space_1D, move_cartesian_space_2D,
                 grasp, move_gripper):
        self.rotate_EE = rotate_EE
        self.rotate_EE_quat = rotate_EE_quat
        self.stop_action = stop_action
        self.resume_action = resume_action
        self.move_joint_space = move_joint_space
        self.move_cartesian_space = move_cartesian_space
        self.move_cartesian_space_1D = move_cartesian_space_1D
        self.move_cartesian_space_2D = move_cartesian_space_2D
        self.grasp = grasp
        self.move_gripper = move_gripper
        self._counter = 0
        self.pick_server = actionlib.SimpleActionServer('/opendr/pick',
                                                        PickAction,
                                                        self.pick2,
                                                        auto_start=False)

        self.place_server = actionlib.SimpleActionServer('/opendr/place',
                                                         PlaceAction,
                                                         self.place,
                                                         auto_start=False)

        self.give_server = actionlib.SimpleActionServer('/opendr/give',
                                                         PlaceAction,
                                                         self.give,
                                                         auto_start=False)

        self.speech_sub = rospy.Subscriber('/text_commands', String, self.voice_pause) #pause on comment. Change pause function statements accordingly.
        self.gesture_sub = rospy.Subscriber('/opendr/commands', String, self.gesture_pause) #pause on gesture.
        self.recovery_pub = rospy.Publisher("/franka_control/error_recovery/goal", ErrorRecoveryActionGoal, queue_size=1)

        self._pause = False
        self._table_level = 0.116

    def __del__(self):
        self.pick_server = None
        self.place_server = None
        self.give_server = None

    def start(self):
        self.pick_server.start()
        self.place_server.start()
        self.give_server.start()
        self.move_gripper(0.08)
        self.move_gripper(0.04)

    def stop(self):
        self.pick_server = None
        self.place_server = None
        self.give_server = None

    def pick(self, goal):
        success = True
        z_final = goal.pose.position.z if goal.pose.position.z > self._table_level else self._table_level
        z_intermediate = z_final + 0.15
        # Aproach
        self.move_cartesian_space_2D([goal.pose.position.x, goal.pose.position.y], False)
        self.move_cartesian_space_1D(z_intermediate, False)
        orientation_list = [goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        print("pick_rotate")
        self.rotate_EE_quat([0.9239462162078852, -0.38251904893439176, 0.0, 0.0])
        self.rotate_EE(yaw)
        # Pre-grasp
        self.move_cartesian_space_1D(z_final, True)
        # Grasp
        self.grasp(goal.width, goal.force)
        # Post-grasp
        self.move_cartesian_space_1D(z_intermediate, False)

        if success:
            result = PickResult(True)
            self.pick_server.set_succeeded(result)

    def pick2(self, goal):
        success = True
        z_final = goal.pose.position.z if goal.pose.position.z > self._table_level else self._table_level
        #z_final = self._table_level
        z_intermediate = z_final + 0.15
        pose_intermediate = goal.pose
        pose_intermediate.position.z = z_intermediate
        # Aproach
        self.move_cartesian_space(pose_intermediate)
        #orientation_list = [pose_intermediate.orientation.x, pose_intermediate.orientation.y, pose_intermediate.orientation.z, pose_intermediate.orientation.w]
        #(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        # Pre-grasp
        self.move_cartesian_space_1D(z_final, True)
        # Grasp
        self.grasp(goal.width, goal.force)
        time.sleep(1)
        # Post-grasp
        self.move_cartesian_space_1D(z_intermediate, False)

        if success:
            result = PickResult(True)
            self.pick_server.set_succeeded(result)

    def place(self, goal):
        success = True

        z_final = goal.pose.position.z
        #Original:
        #z_intermediate = goal.pose.position.z + 0.3
        #Rami edit:
        z_intermediate = goal.pose.position.z + 0.15
        #Original:
        #goal.pose.position.z = z_intermediate
        
        #Original:
        #prepose = Pose()
        #prepose.position.x = 0.4955222156145186
        #prepose.position.y = -0.11675839207592095
        #prepose.position.z = 0.3986073776445874
        #prepose.orientation.x = -0.6609967852858333
        #prepose.orientation.y = -0.31114417228114366
        #prepose.orientation.z = -0.27521515925565554
        #prepose.orientation.w = 0.6249233313080573
        
        #Rami edit:
        prepose = Pose()
        prepose.position.x = goal.pose.position.x
        prepose.position.y = goal.pose.position.y
        prepose.position.z = z_intermediate
        prepose.orientation.x = goal.pose.orientation.x
        prepose.orientation.y = goal.pose.orientation.y
        prepose.orientation.z = goal.pose.orientation.z
        prepose.orientation.w = goal.pose.orientation.w
        
        self.move_cartesian_space(prepose)
        time.sleep(1)
        self.move_cartesian_space(goal.pose)
        time.sleep(1)
        #Original:
        #self.move_cartesian_space_1D(z_final, True)
        #time.sleep(1)
        #Original:
        #self.move_gripper(0.04)
        #Rami edit:
        self.move_gripper(0.08)
        time.sleep(1)
        self.move_cartesian_space_1D(z_intermediate, True)
        if success:
            self._counter += 1
            result = PlaceResult(True)
            self.place_server.set_succeeded(result)

    def give(self, goal):
        success = True

        self.move_cartesian_space(goal.pose)
        time.sleep(1)
        self.move_gripper(0.04)

        if success:
            result = PlaceResult(True)
            self.give_server.set_succeeded(result)

    def voice_pause(self, msg):
        if msg.data == 'PAUSE' and not self._pause:
            self.stop_action()
            self._pause = True
        if msg.data == 'CONTINUE':
            self.recovery_pub.publish(ErrorRecoveryActionGoal())
        if msg.data == 'CONTINUE' and self._pause:
            self.resume_action()
            self._pause = False

    def gesture_pause(self, msg):
        if msg.data == 'PAUSE' and not self._pause:
            self.stop_action()
            self._pause = True
        if msg.data == 'CONTINUE':
            self.recovery_pub.publish(ErrorRecoveryActionGoal())
        if msg.data == 'CONTINUE' and self._pause:
            self.resume_action()
            self._pause = False
