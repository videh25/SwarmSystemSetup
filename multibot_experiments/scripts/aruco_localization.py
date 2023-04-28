#! /usr/bin/python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from tf.transformations import quaternion_from_matrix
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3
from std_msgs.msg import Header
from typing import Dict

class ArucoLocalizationNode:
    def __init__(self, camera_ns: str, frame_name: str, markerSizeInCM: float, ID_Dict: Dict[int, str]) -> None:
        # Aruco Processing Standard Utils
        self.cv2bridge = CvBridge()
        self.camera_ns = camera_ns
        self.frame_name = frame_name
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.dict_aruco = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.marker_size_cm = markerSizeInCM
        self.ID_Dict = ID_Dict
        self.rate = rospy.Rate(10)

        # Camera Info Variables
        self.camera_data_fetched = False
        self.camera_matrix = None
        self.camera_distortion = None
        self.latest_image = None

        self.pubs_and_subs()

    def pubs_and_subs(self):
        self.caminfo_subcriber = rospy.Subscriber(self.camera_ns + "/camera_info", CameraInfo, self.fetch_camera_info)
        self.img_subscriber = rospy.Subscriber(self.camera_ns + "/image_raw", Image, self.aruco_detect)

        self.tf_broadcater = tf2_ros.TransformBroadcaster()

    def fetch_camera_info(self, msg: CameraInfo):
        self.camera_matrix = (np.resize(np.array(msg.K), (3,3)))
        self.camera_distortion = np.array(msg.D)
        self.camera_data_fetched = True
        self.caminfo_subcriber.unregister()

    def aruco_detect(self, msg: Image):
        rospy.loginfo("Inside the callback")
        self.rate.sleep()
        if not self.camera_data_fetched:
            self.caminfo_subcriber = rospy.Subscriber(self.camera_ns + "/camera_info", CameraInfo, self.fetch_camera_info)
            return

        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.cv2bridge.imgmsg_to_cv2(msg, "bgr8")
        except (CvBridgeError, e):
            print(e)
        
        self.latest_image = cv2_img

        if self.latest_image is not None:
            gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dict_aruco, parameters=self.parameters)

            if (ids is not None) and (len(ids) != 0):
                rvec , tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size_cm, self.camera_matrix, self.camera_distortion)
                
                list_tvec = [(vec[0])/100 for vec in tvec]
                list_rvec = [(vec[0]) for vec in rvec]
                list_ids = np.ravel(ids)

                t = TransformStamped()
                for tvec_, rvec_, id_ in zip(list_tvec, list_rvec, list_ids):
                    cv2.aruco.drawAxis(self.latest_image,self.camera_matrix,self.camera_distortion,rvec_, tvec_, 0.2)
                    
                    rotation_matrix = np.zeros((4,4), dtype = 'float')
                    rotation_matrix[3][3] = 1.
                    rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvec_)

                    # convert the matrix to a quaternion
                    quaternion = quaternion_from_matrix(rotation_matrix)

                    t.header = Header(stamp=rospy.Time.now(), frame_id=self.frame_name)
                    t.child_frame_id = self.ID_Dict[int(id_)]

                    t.transform.translation = Vector3(x = tvec_[0], y = tvec_[1], z = tvec_[2])
                    t.transform.rotation.x = quaternion[0]
                    t.transform.rotation.y = quaternion[1]
                    t.transform.rotation.z = quaternion[2]
                    t.transform.rotation.w = quaternion[3]

                    self.tf_broadcater.sendTransform(t)
            else:
                rospy.loginfo("No Aruco Marker Detected")
        




if __name__ == "__main__":
    rospy.init_node('ArucoLocalizationNode')
    bots_data = rospy.get_param('experiment_bots')
    aruco_id_dict = {bots_data[bot_name]["aruco_value"]: bot_name for bot_name in bots_data}

    camera_namespace = rospy.get_param(rospy.get_name() + '/drone_camera')
    frame_name = rospy.get_param(rospy.get_name() + '/camera_frame_name')
    rospy.loginfo(frame_name)
    node_ = ArucoLocalizationNode(camera_namespace, frame_name, 20*0.7, aruco_id_dict)
    rospy.spin()