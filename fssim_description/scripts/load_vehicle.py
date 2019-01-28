#!/usr/bin/env python
import sys
import rospy
import xacro
import subprocess

def load_robot_description():
    sensors_config_file =   rospy.get_param('/fssim/sensors_config_file')
    car_config_file     =   rospy.get_param('/fssim/car_config_file')
    car_dimensions_file =   rospy.get_param('/fssim/car_structure_file')
    robot_name          =   rospy.get_param('/fssim/robot_name')

    model_filepath = rospy.get_param('/fssim/model_filepath')

    print sensors_config_file
    print car_config_file
    print car_dimensions_file
    print model_filepath

    try:
        command_string = "rosrun xacro xacro --inorder {} robot_name:='{}' sensors_config_file:='{}' car_config_file:='{}' car_dimensions_file:='{}".format(model_filepath, 
            robot_name, sensors_config_file, car_config_file, car_dimensions_file)
        robot_description = subprocess.check_output(command_string, shell=True, stderr=subprocess.STDOUT)   
    except subprocess.CalledProcessError as process_error:
        rospy.logfatal('Failed to run xacro command with error: \n%s', process_error.output)
        sys.exit(1)

    rospy.set_param("/robot_description", robot_description)

if __name__ == '__main__':
    try:
        rospy.init_node('load_robot_description', anonymous=True)
        load_robot_description()
    except rospy.ROSInterruptException:
        pass