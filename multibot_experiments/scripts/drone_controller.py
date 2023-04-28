#! /usr/bin/python3

import rospy
import tf
import actionlib
from hector_uav_msgs.msg import PoseAction, PoseGoal
from geometry_msgs.msg import PoseStamped

import numpy as np
from typing import Dict

class DroneController:
    def __init__(self, drone_height: float, ID_Dict: Dict[int, str]) -> None:
        # Aruco Processing Standard Utils
        self.tf_listener = tf.TransformListener()
        self.ID_Dict = ID_Dict
        self.position = np.array([0, 0, 0]) #[x, y, z]
        self.centroid = np.array([0, 0])

        self.pose_cmd_client = actionlib.SimpleActionClient('action/pose', PoseAction)
        self.pose_cmd_client.wait_for_server()

        self.pose_goal = PoseGoal()
        self.pose_goal.target_pose.header.frame_id = 'world'
        self.pose_goal.target_pose.pose.position.x = 0
        self.pose_goal.target_pose.pose.position.y = 0
        self.pose_goal.target_pose.pose.position.z = drone_height

        self.pose_cmd_client.send_goal(self.pose_goal)
        rospy.sleep(3)
        self.centroid_update_timer = rospy.Timer(rospy.Duration(5), self.update_centroid)
        self.pubs_and_subs()
        rospy.logwarn("Completed stratup")

    def pubs_and_subs(self):
        self.caminfo_subcriber = rospy.Subscriber("ground_truth_to_tf/pose", PoseStamped, self.update_self_position)

    def update_self_position(self, msg: PoseStamped):
        self.position[0] = msg.pose.position.x
        self.position[1] = msg.pose.position.y
        self.position[2] = msg.pose.position.z

    def update_centroid(self, event):
        bot_positions = []
        for aruco in range(len(self.ID_Dict)):
            trans, _ = self.tf_listener.lookupTransform(self.ID_Dict[aruco], 'drone', rospy.Time(0))
            bot_positions.append(np.array((trans[1], -trans[0])))
        
        bot_positions = np.array(bot_positions)
        self.centroid = np.average(bot_positions, axis = 0)
        if np.linalg.norm(self.centroid) > 0.8:
            self.pose_goal.target_pose.pose.position.x = self.position[0]+self.centroid[0]
            self.pose_goal.target_pose.pose.position.y = self.position[1]+self.centroid[1]
            self.pose_cmd_client.send_goal(self.pose_goal)
        rospy.logwarn(self.centroid)

if __name__ == "__main__":
    from hector_uav_msgs.srv import EnableMotors
    
    rospy.init_node('DroneControllerNode')
    bots_data = rospy.get_param('experiment_bots')
    aruco_id_dict = {bots_data[bot_name]["aruco_value"]: bot_name for bot_name in bots_data}
    num_bots = len(aruco_id_dict)

    rospy.wait_for_service('enable_motors')
    try:
        my_service = rospy.ServiceProxy('enable_motors', EnableMotors)
        response = my_service(True)  # Call the service with no arguments
    except rospy.ServiceException as e:
        print("Service call failed:", e)

    node_ = DroneController(2, aruco_id_dict)
    rospy.spin()