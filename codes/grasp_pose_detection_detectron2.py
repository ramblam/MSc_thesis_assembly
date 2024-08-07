#!/usr/bin/env python3

# -*- coding: utf-8 -*-
# Copyright 2020-2022 OpenDR European Project
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

import math
import torch
import random
import pathlib
import argparse
import numpy as np
import message_filters

from opendr.perception.object_detection_2d.detectron2.utils.object_tracker import Tracker

from opendr.engine.data import Image
from opendr.engine.target import BoundingBox, BoundingBoxList
from opendr.perception.object_detection_2d import Detectron2Learner
from opendr.perception.object_detection_2d import draw_bounding_boxes
from opendr.perception.object_detection_2d import vis_utils


import tf
import rospy
from opendr_bridge import ROSBridge
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image as ROS_Image
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose, VisionInfo


class Detectron2GraspDetectionNode:

    def __init__(self, camera_tf_frame="/camera_color_optical_frame", robot_tf_frame="/panda_link0",
                 ee_tf_frame="/panda_link8", input_image_topic="/camera/color/image_raw",
                 input_depth_image_topic="/camera/aligned_depth_to_color/image_raw",
                 camera_info_topic="/camera/color/camera_info",
                 output_image_topic="/opendr/image_grasp_pose_annotated",
                 output_mask_topic="/opendr/image_mask_annotated",
                 object_detection_topic="/opendr/object_detected",
                 grasp_detection_topic="/opendr/grasp_detected",
                 device="cuda", only_visualize=False, model="detectron"):
        """
        Creates a ROS Node for objects detection from RGBD
        :param input_image_topic: Topic from which we are reading the input image
        :type input_image_topic: str
        :param input_depth_image_topic: Topic from which we are reading the input depth image
        :type input_depth_image_topic: str
        :param gesture_annotations_topic: Topic to which we are publishing the predicted gesture class
        :type gesture_annotations_topic: str
        :param device: device on which we are running inference ('cpu' or 'cuda')
        :type device: str
        """
        self.bridge = ROSBridge()

        # Initialize the object detection
        model_path = pathlib.Path("/media/opendr/7dd9f6a3-cbe2-481b-8fbd-473c4c7f2b5d/home/amir/rami_test/test/test2/test_aug_from_json/outputAfterPreprocessingv3")

        if model == "detectron":
            self.learner = Detectron2Learner(device=device)
            if not model_path.exists():
                self.learner.download(path=model_path) 
            self.learner.load(model_path / "model_final.pth")

        self.mask_publisher = rospy.Publisher(output_mask_topic, ROS_Image, queue_size=10)
        self.test_mask_publisher = rospy.Publisher(output_mask_topic+"_test", ROS_Image, queue_size=10)
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
        self.image_publisher = rospy.Publisher(output_image_topic, ROS_Image, queue_size=10)
        self.object_publisher = rospy.Publisher(object_detection_topic, Detection2DArray, queue_size=10)
        self.grasp_publisher = rospy.Publisher(grasp_detection_topic, ObjectHypothesisWithPose, queue_size=10)

        self.camera_info_sub = rospy.Subscriber(camera_info_topic, CameraInfo, self.camera_info_callback)

        self.rgb_sub = message_filters.Subscriber(input_image_topic, ROS_Image)
        self.depth_sub = message_filters.Subscriber(input_depth_image_topic, ROS_Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=5,
                                                              slop=20.0, allow_headerless=True)
        self.ts.registerCallback(self.callback)
        print(only_visualize)
        self.only_visualize = only_visualize

        self._refX = 0
        self._refY = 0
        self._camera_focal = 0

        self.tracker = Tracker() 

        self.start_time = rospy.Time.now()
        self.detection_duration = rospy.Duration(10.0)

        self._ee_tf_frame = ee_tf_frame
        self._robot_tf_frame = robot_tf_frame
        self._camera_tf_frame = camera_tf_frame
        self._listener_tf = tf.TransformListener()

        classes = rospy.get_param('/opendr/object_categories')
        self.colors = dict()
        for obj_class in classes:
            self.colors.update({obj_class:(random.random(), random.random(), random.random())})
        print(self.colors)

    def camera_info_callback(self, msg):
        self._camera_focal = msg.K[0]
        self._refX = msg.K[2]
        self._refY = msg.K[5]

    def create_rviz_marker(self, obj_class, ros_pose, marker_id):
        marker = Marker()
        marker.header.frame_id = self._robot_tf_frame
        marker.header.stamp = rospy.Time.now()
        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 2
        marker.id = marker_id
        # Set the scale of the marker
        marker.scale.x = 0.025
        marker.scale.y = 0.025
        marker.scale.z = 0.025
        # Set the color
        n_classes = len(vis_utils.VOC_COLORS)
        marker.color.r = vis_utils.VOC_COLORS[int(obj_class) % n_classes][2] / 255
        marker.color.g = vis_utils.VOC_COLORS[int(obj_class) % n_classes][1] / 255
        marker.color.b = vis_utils.VOC_COLORS[int(obj_class) % n_classes][0] / 255
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position.x = ros_pose.pose.position.x
        marker.pose.position.y = ros_pose.pose.position.y
        marker.pose.position.z = ros_pose.pose.position.z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        self.marker_pub.publish(marker)

    def compute_orientation(self, seg_mask):
        # M = cv2.moments(seg_mask)

        # calculate x,y coordinate of center
        # cX = int(M["m10"] / M["m00"])
        # cY = int(M["m01"] / M["m00"])

        y, x = np.nonzero(seg_mask)
        x = x - np.mean(x)
        y = y - np.mean(y)
        coords = np.vstack([x, y])
        cov = np.cov(coords)
        evals, evecs = np.linalg.eig(cov)
        sort_indices = np.argsort(evals)[::-1]
        x_v1, y_v1 = evecs[:, sort_indices[0]]  # Eigenvector with largest eigenvalue
        x_v2, y_v2 = evecs[:, sort_indices[1]]

        # theta = np.arctan((x_v1)/(y_v
        theta = math.atan2(y_v1, x_v1)
        theta = math.degrees(theta) - 135
        theta = math.radians(theta)
        return theta

    def compute_depth(self, depth_image, seg_mask):
        seg_mask = seg_mask.astype('bool')
        object_depth_map = depth_image[seg_mask].tolist()
        depth_values = [x for x in object_depth_map if x != 0]
        unique, frequency = np.unique(depth_values, return_counts=True)
        depth = unique[np.where(frequency == max(frequency))]
        real_z = depth[0]/1000 if depth[0]/1000 > 0.15 else 0.15
        return real_z

    def get_center_bbox(self, bbox):
        ctr_x = bbox.left+bbox.width/2
        ctr_y = bbox.top+bbox.height/2
        return (ctr_x, ctr_y)

    def convert_detection_pose(self, ctr_x, ctr_y, depth_image, seg_mask):
        assert self._camera_focal > 0, "camera info not found"
        assert self._refX > 0, "camera info not found"
        assert self._refY > 0, "camera info not found"
        theta = self.compute_orientation(seg_mask)
        mask = np.asarray(seg_mask)
        z_to_surface = self.compute_depth(depth_image, mask)
        to_world_scale = z_to_surface / self._camera_focal
        x_dist = (ctr_x - self._refX) * to_world_scale
        y_dist = (self._refY - ctr_y) * to_world_scale

        my_point = PoseStamped()
        my_point.header.frame_id = self._camera_tf_frame
        my_point.header.stamp = rospy.Time(0)
        my_point.pose.position.x = -x_dist
        my_point.pose.position.y = y_dist
        my_point.pose.position.z = z_to_surface
        #quat_rotcmd = tf.transformations.quaternion_from_euler(0, 0, theta)

        quat_rotcmd = tf.transformations.quaternion_from_euler(0, 0, theta+math.pi)
        my_point.pose.position.x = x_dist
        my_point.pose.position.y = -y_dist
        
        my_point.pose.orientation.x = quat_rotcmd[0]
        my_point.pose.orientation.y = quat_rotcmd[1]
        my_point.pose.orientation.z = quat_rotcmd[2]
        my_point.pose.orientation.w = quat_rotcmd[3]
        ps = self._listener_tf.transformPose(self._robot_tf_frame, my_point)
        return ps.pose

    def draw_seg_masks(self, image, masks):
        masked_image = np.array([])
        if masks:
            masked_image = np.full_like(masks[0], 0)
            for mask in masks:
                mask = np.asarray(mask)
                masked_image += mask
        return masked_image

    def callback(self, image_data, depth_data):
        """
        Callback that process the input data and publishes to the corresponding topics
        :param image_data: input image message
        :type image_data: sensor_msgs.msg.Image
        :param depth_data: input depth image message
        :type depth_data: sensor_msgs.msg.Image
        """
        # Convert sensor_msgs.msg.Image into OpenDR Image and preprocess
        image = self.bridge.from_ros_image(image_data)
        depth_data.encoding = "mono16"
        depth_image = self.bridge._cv_bridge.imgmsg_to_cv2(depth_data, desired_encoding='mono16')
                
        # Run object detection
        object_detections = self.learner.infer(image)

        boxes = BoundingBoxList([box for box, seg_mask in object_detections])

        seg_masks = [seg_mask for box, seg_mask in object_detections if not box.name==4]

        # Get an OpenCV image back
        image = image.opencv()

        # Publish detections in ROS message
        ros_boxes = self.bridge.to_ros_bounding_box_list(boxes)  # Convert to ROS bounding_box_list
        if self.object_publisher is not None:
            self.object_publisher.publish(ros_boxes)

        if (rospy.Time.now() < self.start_time + self.detection_duration):
            print(rospy.Time.now() - self.start_time)
            self.tracker.store_detections(object_detections)

        #Draw tracked masks
        seg_masks_test = list()

        for obj_class in self.tracker.detections.values():
            for obj_detection in obj_class:
                seg_masks_test.append(obj_detection[2])

        masks_test = self.draw_seg_masks(image, seg_masks_test)

        if self.image_publisher is not None:
            # Annotate image with object detection boxes
            image = draw_bounding_boxes(image, boxes)
            masks = self.draw_seg_masks(image, seg_masks)
            
            # Convert the annotated OpenDR image to ROS image message using bridge and publish it
            self.image_publisher.publish(self.bridge.to_ros_image(Image(image), encoding='bgr8')) 

            if not masks.size == 0:
                masked_image = np.zeros_like(image)
                masked_image[:, :, 0] = masks[:]
                masked_image[:, :, 1] = masks[:]
                masked_image[:, :, 2] = masks[:]

                masked_image_test = np.zeros_like(image)
                masked_image_test[:, :, 0] = masks_test[:]
                masked_image_test[:, :, 1] = masks_test[:]
                masked_image_test[:, :, 2] = masks_test[:]
                self.mask_publisher.publish(self.bridge.to_ros_image(Image(masked_image)))
                self.test_mask_publisher.publish(self.bridge.to_ros_image(Image(masked_image_test)))
            else:
                self.mask_publisher.publish(self.bridge.to_ros_image(Image(image), encoding='bgr8'))

        if not self.only_visualize:
            marker_id = 0
            #for bbox, mask in zip(boxes, seg_masks):
            for key, value in self.tracker.detections.items():
                for obj_detection in value:
                    try:
                        obj_pose = ObjectHypothesisWithPose()
                        obj_pose.id = key
                        obj_pose.pose.pose = self.convert_detection_pose(obj_detection[0], obj_detection[1], depth_image, obj_detection[2])
                        self.create_rviz_marker(key, obj_pose.pose, marker_id)
                        self.grasp_publisher.publish(obj_pose)
                        marker_id+=1
                    except Exception as e:
                        print(e)
                        print("[Warning: skipping a conversion. Pose not valid.]")


if __name__ == '__main__':
    image_topic = "/camera/color/image_raw"
    depth_topic = "/camera/aligned_depth_to_color/image_raw"
    camera_info_topic = "/camera/color/camera_info"
    camera_tf_frame = "/camera_color_optical_frame"
    robot_tf_frame = "/panda_link0"
    ee_tf_frame = "/panda_link8"
    only_visualize = False


    rospy.init_node('opendr_grasp_pose_detection', anonymous=True)
    # Select the device for running
    try:
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
    except Exception:
        device = 'cpu'

    rospy.set_param('/opendr/object_categories', {'0': 'allen key', '1': 'screwdriver', '2': 'big bolt',
                    '3': 'pipe', '4': 'medium bolt', '5': 'small bolt', '6': 'bolt hole', '7': 'cover', '8': 'elbow', '9': 'elbow target', '10': 'cover target', '11': 'metallic box'})

    grasp_detection_node = Detectron2GraspDetectionNode(camera_tf_frame=camera_tf_frame,
                                                        robot_tf_frame=robot_tf_frame,
                                                        ee_tf_frame=ee_tf_frame,
                                                        input_image_topic=image_topic,
                                                        input_depth_image_topic=depth_topic,
                                                        device=device,
                                                        only_visualize=only_visualize,
                                                        camera_info_topic=camera_info_topic)
    rospy.loginfo("Detectron2 object detection node started!")

    try:
        obj_cat_pub = rospy.Publisher("/opendr/object_categories", VisionInfo, queue_size=1)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            obj_cat_pub.publish(VisionInfo(method="object categories", database_location="/opendr/object_categories"))
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
