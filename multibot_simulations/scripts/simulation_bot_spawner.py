#! /usr/bin/python3

import rospy
import rospkg
import os
import subprocess
"""
Spawns the bots as specified in the bots_data.yaml file
"""
def SimulationBotSpawn():
    rospy.init_node('SimulationBotSpawner')
    rospack = rospkg.RosPack()
    bots_data = rospy.get_param('experiment_bots')
    turtlebot3_markers_description_path = rospack.get_path('turtlebot3_markers_description')

    for key in bots_data:
        os.environ['ROS_NAMESPACE'] = key
        
        # Parse the xacro and set the robot_desciption
        xacro_parsing_cmd = ['rosrun', 'xacro', 'xacro', turtlebot3_markers_description_path + f'/urdf/turtlebot3_{bots_data[key]["model"]}_marker.urdf.xacro', 'aruco_value:=' + str(int(bots_data[key]['aruco_value']))]
        xacro_parsing = subprocess.Popen(xacro_parsing_cmd, stdout=subprocess.PIPE)
        urdf_txt, _ = xacro_parsing.communicate()
        rospy.set_param( key+ '/robot_description', str(urdf_txt, encoding='UTF-8'))
        
        # Start the robot_state_publisher after setting apt parameters 
        rospy.set_param( key+ '/robot_state_publisher/publish_frequency', 50.0)
        rospy.set_param( key+ '/robot_state_publisher/tf_prefix', key)
        subprocess.run('rosrun robot_state_publisher robot_state_publisher &', shell=True)

        # Start the gazebo spawner
        subprocess.run(f'rosrun gazebo_ros spawn_model -urdf -model {key} -x {bots_data[key]["spawn_pose"]["x"]} -y {bots_data[key]["spawn_pose"]["y"]} -z 0 -Y {bots_data[key]["spawn_pose"]["yaw"]} -param robot_description &', shell=True)
        

if __name__ == "__main__":
    SimulationBotSpawn(),