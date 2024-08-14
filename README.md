# MSc_thesis_assembly
This repository contains codes for a robot system combining speech recognition, TTS, object detection and robot control.

Notebooks directory contains jupyter notebooks used for image augmentation and training the object detection model. Utilized with changes based on previous work by Ekrekli et al. for their article "Co-Speech Gestures for Human-Robot Collaboration" (2023).

Codes directory contains the following:

  * opendr_fusion_rami.py: The main code of the system that combines the robot control, voice commands and object detection.
  * dynaset_assembly_main.py: The action code that handles the speech recognition and TTS. Creoir Edge VUI SDK is utilized for speech recognition and TTS, see: https://creoir.com/edgevui/
  * grasp_pose_detection_detectron2.py: Handles object detection. Converts the detections from camera frame to robot frame and publishes poses of detected objects to corresponding topic. Utilized with changes based on previous work by Ekrekli et al.
  * detectron2_learner.py: Learner utilized in grasp_pose_detection code.
  * Opendr_control directory: Contains the lower-level implementations for controlling the robot and gripper, such as pick&place. Largely based on MoveIt (http://moveit.ros.org/) and OpenDR (https://github.com/opendr-eu/opendr).
