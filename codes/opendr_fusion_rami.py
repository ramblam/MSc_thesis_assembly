#!/usr/bin/env python3
import rospy
import time
from opendr_control.srv import SetJointState, SetPoseTarget2D, SetPoseTarget, SetPoseTarget1D, MoveGripper
from geometry_msgs.msg import Pose
from opendr_control.msg import PickGoal
from opendr_control.msg import PlaceGoal
from opendr_control.pick_and_place_client import PickAndPlaceClient
from pathlib import Path
import yaml
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose, VisionInfo
from std_msgs.msg import String, Bool
from opendr_control.detections import Detections

from opendr_control.msg import PickActionResult, PlaceActionResult

import moveit_commander
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_multiply
import tf
import math


class FusionNode:

    def __init__(self, input_image__command_topic='/opendr/commands',human_pose_topic='/opendr/human_pose', input_voice__command_topic='/text_commands',input_object__command_topic='/opendr/grasp_detected', another ='/opendr/object_categories'):

        self.input_image__command_topic = input_image__command_topic
        self.input_voice__command_topic = input_voice__command_topic
        self.input_object__command_topic = input_object__command_topic
        self.human_pose_topic = human_pose_topic
        self.another = another

        self.move_joint_space = rospy.ServiceProxy("/opendr/set_joint_state", SetJointState)
        self.move_cartesian_space_2D = rospy.ServiceProxy('/opendr/set_pose_target_2D', SetPoseTarget2D)

        self.move_cartesian_space = rospy.ServiceProxy('/opendr/set_pose_target', SetPoseTarget)
        self.move_cartesian_space_1D = rospy.ServiceProxy('/opendr/set_pose_target_1D', SetPoseTarget1D)
        self.hand_over = Pose()
        self.move_gripper = rospy.ServiceProxy('/opendr/move_gripper', MoveGripper)
        self.allen_key_pose = PickGoal()
        self.screwdriver_pose = PickGoal()
        self.pipe_pose = PickGoal()
        self.elbow_pose = PickGoal()
        self.metallic_box_pose = PickGoal()
        self.cover_pose = PickGoal()
        self.big_bolt_pose = PickGoal()
        self.medium_bolt_pose = PickGoal()
        self.small_bolt_pose = PickGoal()
        self.quality_check_pose = Pose()
        self.cover_target_pose = PlaceGoal()
        self.elbow_target_pose = PlaceGoal()
        self._table_level = 0.116
        self.step_size = 0.05
        self.move_dist = 0.05
        self.dist_msg = 50
        self.next_pose = Pose()

        self.tts_pub = rospy.Publisher("/ros_to_creoir_tts", String, queue_size=10)
        self.own_commander = moveit_commander.MoveGroupCommander(rospy.get_param('/arm_control/group_name'))

        self.id_List = {}
        self.unit = ""

        self.big_bolt_ct = 0
        self.number_of_big_bolts_detected = 0
        self.small_bolt_ct = 0
        self.number_of_small_bolts_detected = 0
        self.medium_bolt_ct = 0
        self.number_of_medium_bolts_detected = 0
        self.number_of_bolt_holes_detected = 0

        self.allen_key_detected = False
        self.screwdriver_detected = False
        self.big_bolt_detected = False
        self.pipe_detected = False
        self.medium_bolt_detected = False
        self.small_bolt_detected = False
        self.bolt_hole_detected = False

        #Cover is picked from predefined point but this may be needed in other functionalities.
        self.cover_detected = False
        self.elbow_detected = False
        self.elbow_target_detected = False
        self.cover_target_detected = False

        #Poses were not read from file in this use case but may still be useful.
        filename = "data_poses"
        with open(str(Path.home() / filename)+'.yml', "r") as stream:
            self.poses = yaml.safe_load(stream)

        self.joint_pose_home = [0.03947615364372932, -0.2751773490529311, -0.19003075736228583, -2.2549331945118145, -0.01891063651608096, 1.9724511810938516, 0.6380886986905766]
        self.joint_pose_hand_over = [0.03947615364372932, -0.2751773490529311, -0.19003075736228583, -2.2549331945118145, -0.01891063651608096, 1.9724511810938516, 0.6380886986905766]
        self.joint_pose_quality = [0.03947615364372932, -0.2751773490529311, -0.19003075736228583, -2.2549331945118145, -0.01891063651608096, 1.9724511810938516, 0.6380886986905766]
        self.hand_over.position.x = 0.4605589481661214
        self.hand_over.position.y = -0.07504445782438655
        self.hand_over.position.z = 0.47245122625836516
        self.hand_over.orientation.x = -0.9259242966293841
        self.hand_over.orientation.y = 0.37728053027212066
        self.hand_over.orientation.z = 0.009888526193032033
        self.hand_over.orientation.w = 0.01502715670702321

        self.quality_check_pose.position.x = 0.4605589481661214
        self.quality_check_pose.position.y = -0.07504445782438655
        self.quality_check_pose.position.z = 0.47245122625836516
        self.quality_check_pose.orientation.x = -0.9259242966293841
        self.quality_check_pose.orientation.y = 0.37728053027212066
        self.quality_check_pose.orientation.z = 0.009888526193032033
        self.quality_check_pose.orientation.w = 0.01502715670702321

        self.cover_pose.pose.position.x = 0.550758507798
        self.cover_pose.pose.position.y = -0.253459750734
        self.cover_pose.pose.position.z = 0.1176910084203
        self.cover_pose.pose.orientation.x = 0.909466441462
        self.cover_pose.pose.orientation.y = -0.413411764929
        self.cover_pose.pose.orientation.z = -0.03966160207822
        self.cover_pose.pose.orientation.w = 0.0197094340946
        self.cover_pose.width = 0.076
        self.cover_pose.force = 20.0

        self.detections = Detections()

        self.sub2 = rospy.Subscriber(self.input_voice__command_topic, String, self.voice_callback)
        print("Speech recognition has started.")

        self.sub3 = rospy.Subscriber(self.input_object__command_topic, ObjectHypothesisWithPose, self.detections.process_detection)
        self.sub4 = rospy.Subscriber(self.another, VisionInfo, self.detections.save_categories)
        self.pick_and_place_client = PickAndPlaceClient()
        self.pick_and_place_client.start()

        rospy.loginfo("Sensor fusion node started!")

        robot_pose_at_start = self.own_commander.get_current_pose().pose
        print("robot pose at start: ", robot_pose_at_start)
        time.sleep(5)

    def listen(self):
        """
        Start the node and begin processing input data
        """

        self.allen_key_id = self.detections.find_object_by_category("allen key")
        self.allen_key_poses= self.detections.get_object_pose(self.allen_key_id)
        if self.allen_key_poses == False:
            print("Allen key not detected")
            self.allen_key_detected = False
        else:
            self.allen_key_pose.pose = self.allen_key_poses[0]
            self.allen_key_pose.pose.position.z = 0.119
            self.allen_key_pose.width = 0.005
            self.allen_key_pose.force = 20.0
            print("Allen key detected")
            self.allen_key_detected = True
        
        self.screwdriver_id = self.detections.find_object_by_category("screwdriver")
        self.screwdriver_poses = self.detections.get_object_pose(self.screwdriver_id)
        print("self.screwdriver_poses: ", self.screwdriver_poses)
        if self.screwdriver_poses == False:
            print("Screwdriver not detected!")
            self.screwdriver_detected = False
            
        else:
            self.screwdriver_pose.pose = self.screwdriver_poses[0]
            self.screwdriver_pose.pose.position.x += 0.01
            self.screwdriver_pose.pose.position.z = 0.120
            print("Screwdriver detected!")
            self.screwdriver_detected = True

        self.screwdriver_pose.width = 0.005
        self.screwdriver_pose.force = 20.0

        self.big_bolt_id = self.detections.find_object_by_category("big bolt")
        self.big_bolt_poses= self.detections.get_object_pose(self.big_bolt_id)
        print("self.big_bolt_poses: ", self.big_bolt_poses)
        if self.big_bolt_poses == False:
            print("No big bolts detected")
            self.big_bolt_detected = False
            self.number_of_big_bolts_detected = 0
        else:
            self.number_of_big_bolts_detected = len(self.big_bolt_poses)

        self.medium_bolt_id = self.detections.find_object_by_category("medium bolt")
        self.medium_bolt_poses= self.detections.get_object_pose(self.medium_bolt_id)
        print("self.medium_bolt_poses: ", self.medium_bolt_poses)
        if self.medium_bolt_poses == False:
            print("No medium bolts detected")
            self.medium_bolt_detected = False
            self.number_of_medium_bolts_detected = 0
        else:
            self.number_of_medium_bolts_detected = len(self.medium_bolt_poses)
        
        self.small_bolt_id = self.detections.find_object_by_category("small bolt")
        self.small_bolt_poses= self.detections.get_object_pose(self.small_bolt_id)
        print("self.small_bolt_poses: ", self.small_bolt_poses)
        if self.small_bolt_poses == False:
            print("No small bolts detected")
            self.small_bolt_detected = False
            self.number_of_small_bolts_detected = 0
        else:
            self.number_of_small_bolts_detected = len(self.small_bolt_poses)
        
        self.pipe_id = self.detections.find_object_by_category("pipe")
        self.pipe_poses= self.detections.get_object_pose(self.pipe_id)
        if self.pipe_poses == False:
            print("Pipe not detected")
            self.pipe_detected = False
        else:
            self.pipe_pose.pose = self.pipe_poses[0]
            self.pipe_pose.pose.position.y -= 0.06
            self.pipe_pose.pose.position.z = 0.119
            print("Pipe detected")
            self.pipe_detected = True
        self.pipe_pose.width = 0.01
        self.pipe_pose.force = 25.0
        
        self.bolt_hole_id = self.detections.find_object_by_category("bolt hole")
        self.bolt_hole_poses= self.detections.get_object_pose(self.bolt_hole_id)
        print("self.bolt_hole_poses: ", self.bolt_hole_poses)
        if self.bolt_hole_poses == False:
            print("No bolt holes detected")
            self.bolt_hole_detected = False
            self.number_of_bolt_holes_detected = 0
        else:
            self.bolt_hole_detected = True
            self.number_of_bolt_holes_detected = len(self.bolt_hole_poses)
        
        self.elbow_id = self.detections.find_object_by_category("elbow")
        self.elbow_poses= self.detections.get_object_pose(self.elbow_id)
        print("self.elbow_poses: ", self.elbow_poses)
        if self.elbow_poses == False:
            print("Elbow not detected!")
            self.elbow_detected = False
        else:
            self.elbow_pose.pose = self.elbow_poses[0]
            self.elbow_pose.pose.position.z = 0.117
            self.elbow_pose.pose.position.x += 0.015
            print("Elbow detected!")
            self.elbow_detected = True

            #Check if these are needed anymore:
            eulers = euler_from_quaternion([self.elbow_pose.pose.orientation.x, self.elbow_pose.pose.orientation.y, self.elbow_pose.pose.orientation.z, self.elbow_pose.pose.orientation.w])
            print("Roll here: ", eulers[0]-0.1)
            quaternions = quaternion_from_euler(eulers[0]-0.1, eulers[1], eulers[2])
            self.elbow_pose.pose.orientation.x = -quaternions[0]
            self.elbow_pose.pose.orientation.y = -quaternions[1]
            self.elbow_pose.pose.orientation.z = -quaternions[2]
            self.elbow_pose.pose.orientation.w = -quaternions[3]
            print("self.elbow_pose.pose.orientation after conversion: ", self.elbow_pose.pose.orientation)

        self.elbow_pose.width = 0.01
        self.elbow_pose.force = 30.0
        
        self.cover_target_id = self.detections.find_object_by_category("cover target")
        self.cover_target_poses = self.detections.get_object_pose(self.cover_target_id)
        if self.cover_target_poses == False:
            print("Cover target not detected!")
            self.cover_target_detected = False
        else:
            self.cover_target_pose.pose = self.cover_target_poses[0]
            self.cover_target_pose.pose.position.z = 0.251
            print("Cover target detected!")
            self.cover_target_detected = True

        self.elbow_target_id = self.detections.find_object_by_category("elbow target")
        self.elbow_target_poses= self.detections.get_object_pose(self.elbow_target_id)
        #print("self.elbow_target_poses: ", self.elbow_target_poses)
        if self.elbow_target_poses == False:
            print("Elbow target not detected!")
            self.elbow_target_detected = False
        else:
            self.elbow_target_pose.pose = self.elbow_target_poses[0]
            self.elbow_target_pose.pose.position.z = 0.2089
            self.elbow_target_pose.pose.orientation.w = -self.elbow_target_pose.pose.orientation.w
            print("Elbow target detected!")
            print("self.elbow_target_poses: ", self.elbow_target_poses)
            self.elbow_target_detected = True

    def object_callback(self, data):
        if data.id in self.id_List.keys():
            self.id_List[data.id] = [data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z,data.pose.pose.orientation.x,
                                        data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
        else:

            self.id_List[data.id] = [data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z,data.pose.pose.orientation.x,
                                        data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]

    def move_to(self, goal, open_gripper=True, gripper_opening=0.06):
        pose_intermediate = goal
        z_final = pose_intermediate.pose.position.z if pose_intermediate.pose.position.z > self._table_level else self._table_level
        z_intermediate = z_final + 0.15
        self.move_cartesian_space_1D(z_intermediate, False)
        time.sleep(1)
        self.move_cartesian_space_2D([pose_intermediate.pose.position.x, pose_intermediate.pose.position.y], False)
        #If we are moving to for example cover target, we probably have the cover in the gripper. Thus, we don't want to open it.
        #Meanwhile, if we are moving to for example cover_pose, we have to open gripper to avoid collisions. 
        if open_gripper:
            self.move_gripper(gripper_opening)
        time.sleep(1)
        self.move_cartesian_space_1D(z_final, True)

    def move_above(self, goal, distance):
        pose_intermediate = goal
        z_final = pose_intermediate.pose.position.z if pose_intermediate.pose.position.z > self._table_level else self._table_level
        z_intermediate = z_final + distance
        self.move_cartesian_space_1D(z_intermediate, False)
        time.sleep(1)
        self.move_cartesian_space_2D([pose_intermediate.pose.position.x, pose_intermediate.pose.position.y], False)

    def voice_callback(self, msg):
        msg = msg.data.split(" ")
        print(msg)

        if msg[0] == 'PICK':
            if msg[1] == 'SCREWDRIVER':
                if self.screwdriver_detected:
                    print("picking screwdriver")
                    self.tts_pub.publish("Picking screwdriver")
                    print("self.screwdriver_pose when picking: ", self.screwdriver_pose)
                    self.move_gripper(0.08)
                    self.pick_and_place_client.pick(self.screwdriver_pose)
                    self.tts_pub.publish("setContextMain")
                else:
                    self.tts_pub.publish("Screwdriver can not be picked as it was not detected")
            elif msg[1] == 'PIPE':
                if self.pipe_detected:
                    print("picking pipe")
                    self.tts_pub.publish("Picking pipe")
                    self.pick_and_place_client.pick(self.pipe_pose)
                    self.tts_pub.publish("setContextMain")
                else:
                    self.tts_pub.publish("Pipe can not be picked as it was not detected")
            elif msg[1] == 'COVER':
                self.tts_pub.publish("Picking cover from predefined location. Assure it is at right location")
                self.move_gripper(0.08)
                self.pick_and_place_client.pick(self.cover_pose)
                self.tts_pub.publish("setContextMain")
            elif msg[1] == 'ELBOW':
                if self.elbow_detected:
                    print("picking elbow")
                    self.tts_pub.publish("Picking elbow")
                    self.move_gripper(0.08)
                    self.pick_and_place_client.pick(self.elbow_pose)
                    self.tts_pub.publish("setContextMain")
                else:
                    self.tts_pub.publish("Elbow can not be picked as it was not detected")
            elif msg[1] == 'ALLEN' and msg[2] == 'KEY':
                if self.allen_key_detected:
                    print("picking allen key")
                    self.tts_pub.publish("Picking allen key")
                    self.pick_and_place_client.pick(self.allen_key_pose)
                    self.tts_pub.publish("setContextMain")
                else:
                    self.tts_pub.publish("Allen key can not be picked as it was not detected")
            elif msg[1] == 'BIG' and msg[2] == 'BOLT':
                if self.big_bolt_poses == False or (self.big_bolt_ct>(self.number_of_big_bolts_detected-1)):
                    print("No big bolts detected or all detected ones handed over!")
                    self.tts_pub.publish("Can not pick big bolt as no big bolts detected or all detected ones handed over")
                else:
                    self.big_bolt_pose.pose = self.big_bolt_poses[self.big_bolt_ct]
                    self.big_bolt_pose.pose.position.z = 0.118
                    self.big_bolt_pose.width = 0.005
                    self.big_bolt_pose.force = 20.0
                    self.tts_pub.publish("Picking big bolt")
                    self.move_gripper(0.04)
                    self.pick_and_place_client.pick(self.big_bolt_pose)
                    self.tts_pub.publish("setContextMain")
                    self.big_bolt_ct +=1

            elif msg[1] == 'MEDIUM' and msg[2] == 'BOLT':
                if self.medium_bolt_poses == False or (self.medium_bolt_ct>(self.number_of_medium_bolts_detected-1)):
                    print("No medium bolts detected or all detected ones handed over!")
                    self.tts_pub.publish("Can not pick medium bolt as none was detected or all detected ones handed over")
                else:
                    self.medium_bolt_pose.pose = self.medium_bolt_poses[self.medium_bolt_ct]
                    self.medium_bolt_pose.pose.position.z = 0.118
                    self.medium_bolt_pose.width = 0.005
                    self.medium_bolt_pose.force = 20.0
                    self.tts_pub.publish("Picking medium bolt")
                    self.move_gripper(0.04)
                    self.pick_and_place_client.pick(self.medium_bolt_pose)
                    self.tts_pub.publish("setContextMain")
                    self.medium_bolt_ct +=1
            elif msg[1] == 'SMALL' and msg[2] == 'BOLT':
                if self.small_bolt_poses == False or (self.small_bolt_ct>(self.number_of_small_bolts_detected-1)):
                    print("No small bolts detected or all detected ones handed over!!")
                    self.tts_pub.publish("Can not pick small bolt as none was detected or all detected ones handed over!")
                else:
                    self.small_bolt_pose.pose = self.small_bolt_poses[self.small_bolt_ct]
                    self.small_bolt_pose.pose.position.z = 0.118
                    self.small_bolt_pose.width = 0.005
                    self.small_bolt_pose.force = 20.0
                    self.tts_pub.publish("Picking small bolt")
                    self.move_gripper(0.04)
                    self.pick_and_place_client.pick(self.small_bolt_pose)
                    self.tts_pub.publish("setContextMain")
                    self.small_bolt_ct +=1
            elif msg[1] == 'METALLIC' and msg[2] == 'BOX':
                print("Can not pick this part")
                self.tts_pub.publish("Can not pick this part")

        elif msg[0] == 'GIVE':
            if msg[1] == 'SCREWDRIVER':
                if self.screwdriver_detected:
                    print("handing over screwdriver")
                    self.tts_pub.publish("Handing over screwdriver")
                    self.move_gripper(0.08)
                    self.pick_and_place_client.pick_and_give(self.screwdriver_pose, self.hand_over)
                    self.tts_pub.publish("setContextMain")
                elif self.screwdriver_detected == False and self.allen_key_detected == True:
                    self.tts_pub.publish("Can not hand over screwdriver as it was not detected. Did you mean allen key?")
                else:
                    self.tts_pub.publish("Can not hand over screwdriver as it was not detected" )
            elif msg[1] == 'PIPE':
                print("handing over pipe")
                self.pick_and_place_client.pick_and_give(self.pipe_pose, self.hand_over)
                self.tts_pub.publish("setContextMain")
            elif msg[1] == 'COVER':
                self.tts_pub.publish("Handing over cover from a predefined location. Assure it is at the right place")
                print("handing over cover")
                self.move_gripper(0.08)
                self.pick_and_place_client.pick_and_give(self.cover_pose, self.hand_over)
                self.tts_pub.publish("setContextMain")
            elif msg[1] == 'ELBOW':
                if self.elbow_detected: 
                    print("handing over elbow")
                    self.tts_pub.publish("Handing over elbow")
                    self.move_gripper(0.08)
                    self.pick_and_place_client.pick_and_give(self.elbow_pose, self.hand_over)
                    self.tts_pub.publish("setContextMain")
                else:
                    self.tts_pub.publish("Can not hand over elbow as it is not detected")
            elif msg[1] == 'ALLEN' and msg[2] == 'KEY':
                if self.allen_key_detected:
                    print("handing over allen key")
                    self.tts_pub.publish("handing over allen key")
                    self.pick_and_place_client.pick_and_give(self.allen_key_pose, self.hand_over)
                    self.tts_pub.publish("setContextMain")
                elif self.allen_key_detected == False and self.screwdriver_detected == True:
                    self.tts_pub.publish("Can not hand over allen key as it was not detected. Did you mean screwdriver?")
                else:
                    self.tts_pub.publish("Can not hand over allen key as it was not detected")
            elif msg[1] == 'BIG' and msg[2] == 'BOLT':
                if self.big_bolt_poses == False or (self.big_bolt_ct > (self.number_of_big_bolts_detected-1)):
                    print("No big bolts detected or all detected ones handed over!")
                    self.tts_pub.publish("Can not give big bolt as no big bolts were detected or all detected ones handed over")
                else:
                    self.big_bolt_pose.pose = self.big_bolt_poses[self.big_bolt_ct]
                    self.big_bolt_pose.pose.position.z = 0.119
                    self.big_bolt_pose.width = 0.005
                    self.big_bolt_pose.force = 20.0
                    self.tts_pub.publish("Handing over big bolt")
                    self.move_gripper(0.04)
                    self.pick_and_place_client.pick_and_give(self.big_bolt_pose, self.hand_over)
                    self.big_bolt_ct +=1
                    self.tts_pub.publish("setContextMain")

            elif msg[1] == 'MEDIUM' and msg[2] == 'BOLT':
                if self.medium_bolt_poses == False or (self.medium_bolt_ct>(self.number_of_medium_bolts_detected-1)):
                    print("No medium bolts detected or all detected ones handed over!")
                    self.tts_pub.publish("Can not give medium bolt as no medium bolts detected or all detected ones handed over")
                else:
                    self.medium_bolt_pose.pose = self.medium_bolt_poses[self.medium_bolt_ct]
                    self.medium_bolt_pose.pose.position.z = 0.118
                    self.medium_bolt_pose.width = 0.005
                    self.medium_bolt_pose.force = 20.0
                    self.tts_pub.publish("Handing over medium bolt")
                    self.move_gripper(0.04)
                    self.pick_and_place_client.pick_and_give(self.medium_bolt_pose, self.hand_over)
                    self.tts_pub.publish("setContextMain")
                    self.medium_bolt_ct +=1
            elif msg[1] == 'SMALL' and msg[2] == 'BOLT':
                if self.small_bolt_poses == False or (self.small_bolt_ct>(self.number_of_small_bolts_detected-1)):
                    print("Can not give small bolt as no small bolts detected or all detected ones handed over!")
                    self.tts_pub.publish("No small bolts detected or all detected ones handed over")
                else:
                    self.small_bolt_pose.pose = self.small_bolt_poses[self.small_bolt_ct]
                    self.small_bolt_pose.pose.position.z = 0.118
                    self.small_bolt_pose.width = 0.005
                    self.small_bolt_pose.force = 20.0
                    self.tts_pub.publish("Handing over small bolt")
                    self.move_gripper(0.04)
                    self.pick_and_place_client.pick_and_give(self.small_bolt_pose, self.hand_over)
                    self.tts_pub.publish("setContextMain")
                    self.small_bolt_ct +=1
            elif msg[1] == 'METALLIC' and msg[2] == 'BOX':
                print("Can not give that part")
                self.tts_pub.publish("Can not give that part")
            else :
                self.tts_pub.publish("Please specify object to be handed")

        elif msg[0] == 'PLACE':
            if msg[1] == 'ELBOW':
                if self.elbow_target_detected:
                    print("placing elbow")
                    self.tts_pub.publish("Placing elbow")
                    self.pick_and_place_client.place(self.elbow_target_pose)
                    self.tts_pub.publish("setContextMain")
                else:
                    self.tts_pub.publish("Can not place elbow as elbow target not detected")
            elif msg[1] == 'COVER':
                if self.cover_target_detected:
                    print("placing cover")
                    self.tts_pub.publish("Placing cover")
                    self.pick_and_place_client.place(self.cover_target_pose)
                    self.tts_pub.publish("setContextMain")
                else:
                    self.tts_pub.publish("Can not place cover as cover target not detected")
            elif msg[1] == 'PIPE':
                print("placing pipe not implemented")
                self.tts_pub.publish("Placing pipe not implemented. Use pick pipe and move commands instead.")
            elif msg[1] == 'BIG' and msg[2] == 'BOLT':
                print("placing big bolt not implemented")
                self.tts_pub.publish("placing big bolt not implemented")

        elif msg[0] == 'ASSEMBLE':
            if msg[1] == 'COVER':
                if self.cover_target_detected:
                    print("Assembling cover")
                    self.tts_pub.publish("Assembling cover and then moving back home. Picking it from predefined location")
                    self.move_gripper(0.08)
                    self.pick_and_place_client.pick_and_place(self.cover_pose, self.cover_target_pose.pose)
                    time.sleep(1)
                    self.move_joint_space(self.joint_pose_home)
                    self.tts_pub.publish("setContextMain")
                else:
                    self.tts_pub.publish("Unable to assemble cover as its target is neither detected nor predefined")

            elif msg[1] == 'ELBOW':
                if self.elbow_detected and self.elbow_target_detected:
                    print("Assembling elbow")
                    self.tts_pub.publish("Assembling elbow and then moving back home.")
                    self.move_gripper(0.08)
                    self.pick_and_place_client.pick_and_place(self.elbow_pose, self.elbow_target_pose.pose)
                    time.sleep(1)
                    self.move_joint_space(self.joint_pose_home)
                    self.tts_pub.publish("setContextMain")
                else:
                    if self.elbow_detected:
                        self.tts_pub.publish("Unable to assemble elbow as its target is neither detected nor predefined")
                    else:
                        self.tts_pub.publish("Unable to assemble elbow as it is neither detected nor has a predefined location")

        elif msg[0] == 'PAUSE':
            print("Pausing the robot")
            self.tts_pub.publish("Stopping the robot")
        
        elif msg[0] == 'QUALITY' and msg[1] == 'CHECK':
            if self.number_of_big_bolts_detected < 4 or self.bolt_hole_detected==True:
                print("Possible Quality error! Check if all the bolts are placed")
                self.tts_pub.publish("Possible quality error. Check if all the bolts are placed.")
            else:
                self.tts_pub.publish("Quality check done.It looks ok to me")

        elif msg[0] == 'CONTINUE':
            print("Continuing")
            self.tts_pub.publish("Continuing")

        elif msg[0] == 'TOOL':
            if msg[1] == 'OPEN':
                print("Opening the gripper")
                self.tts_pub.publish("Opening the gripper")
                self.move_gripper(0.08)
            elif msg[1] == 'CLOSE':
                print("Closing the gripper")
                self.tts_pub.publish("Closing the gripper")
                self.move_gripper(0.00)
            elif msg[1] == 'ROTATE':
                #This is not tested yet!
                print("Rotating the gripper")
                self.tts_pub.publish("Rotating the gripper")
                robot_pose = self.own_commander.get_current_pose().pose
                orientation_list = [robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w]
                (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
                self.rotate_EE(yaw)

        elif msg[0] == 'POSITION':

            if msg[1] == 'SCREWDRIVER' and msg[2] == 'LOCATION':
                if self.screwdriver_detected:
                    print("Moving to screwdriver location")
                    self.tts_pub.publish("Moving to screwdriver location")
                    self.move_to(self.screwdriver_pose, 0.08)
                    self.tts_pub.publish("setContextMain")
                else:
                    self.tts_pub.publish("Cannot move to screwdriver location as it was not detected")
                
            elif msg[1] == 'ALLEN' and msg[2] == 'KEY' and msg[3] == 'LOCATION':
                if self.allen_key_detected:
                    print("Moving to allen key location")
                    self.tts_pub.publish("Moving to allen key location")
                    self.move_to(self.allen_key_pose)
                    self.tts_pub.publish("setContextMain")
                else:
                    self.tts_pub.publish("Cannot move to allen key location as it was not detected")

            elif msg[1] == 'BIG' and msg[2] == 'BOLT' and msg[3] == 'LOCATION':
                if self.big_bolt_poses == False or (self.big_bolt_ct > (self.number_of_big_bolts_detected-1)):
                    print("Can not move to big bolt location as no big bolts were detected or all detected ones handled")
                    self.tts_pub.publish("Can not move to big bolt location as no big bolts were detected or all detected ones handled")
                else:
                    print("Moving to big bolt location")
                    self.tts_pub.publish("Moving to big bolt location")
                    self.big_bolt_pose.pose = self.big_bolt_poses[self.big_bolt_ct]
                    self.big_bolt_pose.pose.position.z = 0.118
                    self.move_to(self.big_bolt_pose)
                    self.tts_pub.publish("setContextMain")

            elif msg[1] == 'SMALL' and msg[2] == 'BOLT' and msg[3] == 'LOCATION':
                if self.small_bolt_poses == False or (self.small_bolt_ct > (self.number_of_small_bolts_detected-1)):
                    print("Can not move to small bolt location as no small bolts were detected or all detected ones handled")
                    self.tts_pub.publish("Can not move to small bolt location as no small bolts were detected or all detected ones handled")
                else:
                    print("Moving to small bolt location")
                    self.tts_pub.publish("Moving to small bolt location")
                    self.small_bolt_pose.pose = self.small_bolt_poses[self.small_bolt_ct]
                    self.small_bolt_pose.pose.position.z = 0.118
                    self.move_to(self.small_bolt_pose)
                    self.tts_pub.publish("setContextMain")

            elif msg[1] == 'MEDIUM' and msg[2] == 'BOLT' and msg[3] == 'LOCATION':
                if self.medium_bolt_poses == False or (self.medium_bolt_ct > (self.number_of_medium_bolts_detected-1)):
                    print("Can not move to medium bolt location as no medium bolts were detected or all detected ones handled")
                    self.tts_pub.publish("Can not move to medium bolt location as no medium bolts were detected or all detected ones handled")
                else:
                    print("Moving to medium bolt location")
                    self.tts_pub.publish("Moving to medium bolt location")
                    self.medium_bolt_pose.pose = self.medium_bolt_poses[self.medium_bolt_ct]
                    self.medium_bolt_pose.pose.position.z = 0.118
                    self.move_to(self.medium_bolt_pose)
                    self.tts_pub.publish("setContextMain")

            elif msg[1] == 'PIPE' and msg[2] == 'LOCATION':
                if self.pipe_detected:
                    print("Moving to pipe location")
                    self.tts_pub.publish("Moving to pipe location")
                    self.move_to(self.pipe_pose)
                    self.tts_pub.publish("setContextMain")
                else:
                    self.tts_pub.publish("Cannot move to pipe location as it was not detected")

            elif msg[1] == 'COVER' and msg[2] == 'LOCATION':
                #Cover location is predefined
                print("Moving to cover location which is predefined")
                self.tts_pub.publish("Moving to cover location which is predefined")
                self.move_to(self.cover_pose, 0.08)
                self.tts_pub.publish("setContextMain")
            
            elif msg[1] == 'ELBOW' and msg[2] == 'LOCATION':
                if self.elbow_detected:
                    print("Moving to elbow location")
                    self.tts_pub.publish("Moving to elbow location")
                    self.move_to(self.elbow_pose, 0.08)
                    self.tts_pub.publish("setContextMain")
                else:
                    self.tts_pub.publish("Cannot move to elbow location as it was not detected")

            elif msg[1] == 'METALLIC' and msg[2] == 'BOX' and msg[3] == 'LOCATION':
                self.tts_pub.publish("This part cannot be handled")
                print("This part cannot be handled")

            elif msg[1] == 'PIPE' and (msg[2] == 'GOAL' or msg[2] == 'TARGET'):
                self.tts_pub.publish("No pipe target is defined")
                print("No pipe target is defined")

            elif msg[1] == 'COVER' and (msg[2] == 'GOAL' or msg[2] == 'TARGET'):
                if self.cover_target_detected:
                    print("Moving to cover target")
                    self.tts_pub.publish("Moving to cover target location")
                    self.move_to(self.cover_target_pose, False)
                    self.tts_pub.publish("setContextMain")
                else:
                    self.tts_pub.publish("Cannot move to cover target location as it was not detected")

            elif msg[1] == 'ELBOW' and (msg[2] == 'GOAL' or msg[2] == 'TARGET'):
                if self.elbow_target_detected:
                    print("Moving to elbow target")
                    self.tts_pub.publish("Moving to elbow target location")
                    self.move_to(self.elbow_target_pose, False)
                    self.tts_pub.publish("setContextMain")
                else:
                    self.tts_pub.publish("Cannot move to elbow target location as it was not detected")
       
        elif msg[0] == 'HOLD':
            msg_length = len(msg)
            if msg[msg_length-1] == 'MILLIMETERS' or msg[msg_length-1] == 'MILLIMETRES':
                distance = float(msg[msg_length-2])/1000
                dist_message = int(msg[msg_length-2])
                unit_msg = "millimeters"
            if msg[msg_length-1] == 'CENTIMETERS' or msg[msg_length-1] == 'CENTIMETRES':
                distance = float(msg[msg_length-2])/100
                unit_msg = "centimeters"
                dist_message = int(msg[msg_length-2])
            if msg[1] == 'SCREWDRIVER' and msg[2] == 'LOCATION':
                if self.screwdriver_detected:
                    print("Moving above screwdriver location")
                    tts_msg = "Moving {} {} above screwdriver location".format(dist_message, unit_msg)
                    self.tts_pub.publish(tts_msg)
                    self.move_above(self.screwdriver_pose, distance)
                    self.tts_pub.publish("setContextMain")
                else:
                    self.tts_pub.publish("Cannot move above screwdriver location as it was not detected")
                
            elif msg[1] == 'ALLEN' and msg[2] == 'KEY' and msg[3] == 'LOCATION':
                if self.allen_key_detected:
                    print("Moving above allen key location")
                    tts_msg = "Moving {} {} above allen key location".format(dist_message, unit_msg)
                    self.tts_pub.publish(tts_msg)
                    self.move_above(self.allen_key_pose, distance)
                    self.tts_pub.publish("setContextMain")
                else:
                    self.tts_pub.publish("Cannot move above allen key location as it was not detected")

            elif msg[1] == 'BIG' and msg[2] == 'BOLT' and msg[3] == 'LOCATION':
                if self.big_bolt_poses == False or (self.big_bolt_ct > (self.number_of_big_bolts_detected-1)):
                    print("Can not move above big bolt location as no big bolts were detected or all detected ones handled")
                    self.tts_pub.publish("Can not move above big bolt location as no big bolts were detected or all detected ones handled")
                else:
                    print("Moving above big bolt location")
                    tts_msg = "Moving {} {} above big bolt location".format(dist_message, unit_msg)
                    self.tts_pub.publish(tts_msg)
                    self.big_bolt_pose.pose = self.big_bolt_poses[self.big_bolt_ct]
                    self.big_bolt_pose.pose.position.z = 0.118
                    self.move_above(self.big_bolt_pose, distance)
                    self.tts_pub.publish("setContextMain")

            elif msg[1] == 'SMALL' and msg[2] == 'BOLT' and msg[3] == 'LOCATION':
                if self.small_bolt_poses == False or (self.small_bolt_ct > (self.number_of_small_bolts_detected-1)):
                    print("Can not move above small bolt location as no small bolts were detected or all detected ones handled")
                    self.tts_pub.publish("Can not move above small bolt location as no small bolts were detected or all detected ones handled")
                else:
                    print("Moving above small bolt location")
                    tts_msg = "Moving {} {} above small bolt location".format(dist_message, unit_msg)
                    self.tts_pub.publish(tts_msg)
                    self.small_bolt_pose.pose = self.small_bolt_poses[self.small_bolt_ct]
                    self.small_bolt_pose.pose.position.z = 0.118
                    self.move_above(self.small_bolt_pose, distance)
                    self.tts_pub.publish("setContextMain")

            elif msg[1] == 'MEDIUM' and msg[2] == 'BOLT' and msg[3] == 'LOCATION':
                if self.medium_bolt_poses == False or (self.medium_bolt_ct > (self.number_of_medium_bolts_detected-1)):
                    print("Can not move above medium bolt location as no medium bolts were detected or all detected ones handled")
                    self.tts_pub.publish("Can not move above medium bolt location as no medium bolts were detected or all detected ones handled")
                else:
                    print("Moving above medium bolt location")
                    tts_msg = "Moving {} {} above medium bolt location".format(dist_message, unit_msg)
                    self.tts_pub.publish(tts_msg)
                    self.medium_bolt_pose.pose = self.medium_bolt_poses[self.medium_bolt_ct]
                    self.medium_bolt_pose.pose.position.z = 0.118
                    self.move_above(self.medium_bolt_pose, distance)
                    self.tts_pub.publish("setContextMain")

            elif msg[1] == 'PIPE' and msg[2] == 'LOCATION':
                if self.pipe_detected:
                    print("Moving above pipe location")
                    tts_msg = "Moving {} {} above pipe location".format(dist_message, unit_msg)
                    self.tts_pub.publish(tts_msg)
                    self.move_above(self.pipe_pose, distance)
                    self.tts_pub.publish("setContextMain")
                else:
                    self.tts_pub.publish("Cannot move above pipe location as it was not detected")

            elif msg[1] == 'COVER' and msg[2] == 'LOCATION':
                #Cover location is predefined
                print("Moving above cover location")
                tts_msg = "Moving {} {} above cover location".format(dist_message, unit_msg)
                self.tts_pub.publish(tts_msg)
                self.move_above(self.cover_pose, distance)
                self.tts_pub.publish("setContextMain")
            
            elif msg[1] == 'ELBOW' and msg[2] == 'LOCATION':
                if self.elbow_detected:
                    print("Moving above elbow location")
                    tts_msg = "Moving {} {} above elbow location".format(dist_message, unit_msg)
                    self.tts_pub.publish(tts_msg)
                    self.move_above(self.elbow_pose, distance)
                    self.tts_pub.publish("setContextMain")
                else:
                    self.tts_pub.publish("Cannot move above elbow location as it was not detected")

            elif msg[1] == 'METALLIC' and msg[2] == 'BOX' and msg[3] == 'LOCATION':
                self.tts_pub.publish("Cannot handle this part")
                print("Cannot handle this part")

            elif msg[1] == 'PIPE' and (msg[2] == 'GOAL' or msg[2] == 'TARGET'):
                self.tts_pub.publish("No pipe target is defined")
                print("No pipe target is defined")

            elif msg[1] == 'COVER' and (msg[2] == 'GOAL' or msg[2] == 'TARGET'):
                if self.cover_target_detected:
                    print("Moving above cover target")
                    tts_msg = "Moving {} {} above cover target location".format(dist_message, unit_msg)
                    self.tts_pub.publish(tts_msg)
                    self.move_above(self.cover_target_pose, distance)
                    self.tts_pub.publish("setContextMain")
                else:
                    self.tts_pub.publish("Cannot move above cover target location as it was not detected")

            elif msg[1] == 'ELBOW' and (msg[2] == 'GOAL' or msg[2] == 'TARGET'):
                if self.elbow_target_detected:
                    print("Moving above elbow target")
                    tts_msg = "Moving {} {} above elbow target location".format(dist_message, unit_msg)
                    self.tts_pub.publish(tts_msg)
                    self.move_above(self.elbow_target_pose, distance)
                    self.tts_pub.publish("setContextMain")
                else:
                    self.tts_pub.publish("Cannot move above elbow target location as it was not detected")

        elif msg[0] == 'MOVE':
            robot_pose = self.own_commander.get_current_pose().pose
            self.next_pose = robot_pose
            if len(msg) == 4:
                if msg[3] == 'MILLIMETERS' or msg[3] == 'MILLIMETRES':
                    self.move_dist = float(msg[2])/1000
                    self.dist_msg = int(msg[2])
                    self.unit = "millimeters"
                elif msg[3] == 'CENTIMETERS' or msg[3] == 'CENTIMETRES':
                    self.move_dist = float(msg[2])/100
                    self.unit = "centimeters"
                    self.dist_msg = int(msg[2])
                else:
                    self.tts_pub.publish("Please specify unit")
                    self.unit = ""
            if len(msg) == 3:
                self.move_dist = float(msg[2])/1000
                self.unit = "millimeters"
                self.dist_msg = int(msg[2])
            if len(msg) == 2:
                self.move_dist = self.step_size
                self.unit = "millimeters"
                self.dist_msg = int(1000*self.step_size)
            if msg[1] == 'DOWN':
                print("moving down")
                tts_msg = "Moving down {} {}.".format(self.dist_msg, self.unit)
                self.tts_pub.publish(tts_msg)
                self.next_pose.position.z -= self.move_dist
                print("Next_pose: ", self.next_pose)
                self.move_cartesian_space_1D(self.next_pose.position.z, False)
                self.tts_pub.publish("setContextMain")
            elif msg[1] == 'UP':
                print("moving up")
                tts_msg = "Moving up {} {}.".format(self.dist_msg, self.unit)
                self.tts_pub.publish(tts_msg)
                self.next_pose.position.z += self.move_dist
                print("Next_pose: ", self.next_pose)
                self.move_cartesian_space_1D(self.next_pose.position.z, False)
                self.tts_pub.publish("setContextMain")
            elif msg[1] == 'RIGHT':
                print("moving right")
                tts_msg = "Moving right {} {}.".format(self.dist_msg, self.unit)
                self.tts_pub.publish(tts_msg)
                self.next_pose.position.y += self.move_dist
                print("Next_pose: ", self.next_pose)
                self.move_cartesian_space_2D([self.next_pose.position.x, self.next_pose.position.y], False)
                self.tts_pub.publish("setContextMain")
            elif msg[1] == 'LEFT':
                print("moving left")
                tts_msg = "Moving left {} {}.".format(self.dist_msg, self.unit)
                self.tts_pub.publish(tts_msg)
                self.next_pose.position.y -= self.move_dist
                print("Next_pose: ", self.next_pose)
                self.move_cartesian_space_2D([self.next_pose.position.x, self.next_pose.position.y], False)
                self.tts_pub.publish("setContextMain")
            elif msg[1] == 'FRONT' or msg[1] == 'FORWARD':
                print("moving forward")
                tts_msg = "Moving forward {} {}.".format(self.dist_msg, self.unit)
                self.tts_pub.publish(tts_msg)
                self.next_pose.position.x += self.move_dist
                print("Next_pose: ", self.next_pose)
                self.move_cartesian_space_2D([self.next_pose.position.x, self.next_pose.position.y], False)
                self.tts_pub.publish("setContextMain")
            elif msg[1] == 'BACK' or msg[1] == 'BACKWARD':
                print("moving back")
                tts_msg = "Moving back {} {}.".format(self.dist_msg, self.unit)
                self.tts_pub.publish(tts_msg)
                self.next_pose.position.x -= self.move_dist
                print("Next_pose: ", self.next_pose)
                self.move_cartesian_space_2D([self.next_pose.position.x, self.next_pose.position.y], False)
                self.tts_pub.publish("setContextMain")
            elif msg[1] == 'HOME':
                self.tts_pub.publish("Moving home")
                self.move_joint_space(self.joint_pose_home)
                self.tts_pub.publish("setContextMain")
                robot_pose_at_home = self.own_commander.get_current_pose().pose
                print("robot pose at home: ", robot_pose_at_home)
        
        elif msg[0] == 'STEP' and msg[1] == 'SIZE':
            if msg[2] == 'LOW':
                print("Setting step size low")
                self.tts_pub.publish("Setting step size low")
                self.step_size = 0.01
            elif msg[2] == 'MID' or msg[2] == 'MEDIUM':
                print("Setting step size medium")
                self.tts_pub.publish("Setting step size medium")
                self.step_size = 0.05
            elif msg[2] == 'HIGH':
                print("Setting step size high")
                self.tts_pub.publish("Setting step size high")
                self.step_size = 0.1
            else:
                print("Unknown step_size!")
                self.tts_pub.publish("Unknown step size")

    def stop_pick_and_place_client(self):
        self.pick_and_place_client.stop()


if __name__ == '__main__':
    rospy.init_node('sensor_fusion', anonymous=True)
    opendr_fusion_node = FusionNode()
    opendr_fusion_node.listen()

    rospy.spin()