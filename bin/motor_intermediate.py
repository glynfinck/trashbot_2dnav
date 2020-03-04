#! /usr/bin/env python
import rospy
import yaml
import os
from std_msgs.msg import String

from geometry_msgs.msg import Twist

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASE_LOCAL_PLANNER_PATH = BASE_DIR+"/param"+"/base_local_planner_params.yaml"
DESIRED_BASE_LOCAL_PLANNER_PATH = BASE_DIR+"/param"+"/desired_base_local_planner_params.yaml"

with open(BASE_LOCAL_PLANNER_PATH, 'r') as stream:
    try:
        base_local_planner_yaml = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

with open(DESIRED_BASE_LOCAL_PLANNER_PATH, 'r') as stream:
    try:
        desired_base_local_planner_yaml = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

print(type(desired_base_local_planner_yaml))

INPUT_PLANNER = base_local_planner_yaml.get("TrajectoryPlannerROS")
DESIRED_PLANNER = desired_base_local_planner_yaml.get("TrajectoryPlannerROS")
LINEAR_VEL_SCALE=1.0
ANGULAR_VEL_SCALE=1.0
cmd_vel_pub = rospy.Publisher('/cmd_vel', String, queue_size=10)
rospy.init_node('motor_intermediate', anonymous=True)

def convert_range(x,InputLow,InputHigh,OutputLow,OutputHigh):
    return ((x - InputLow) / (InputHigh - InputLow)) * (OutputHigh - OutputLow) + OutputLow
    
def callback(data):
    new_vel = Twist()
    new_vel.linear.x = convert_range(
        data.linear.x,
        INPUT_PLANNER.get("min_vel_x"),
        INPUT_PLANNER.get("max_vel_x"),
        DESIRED_PLANNER.get("min_vel_x"),
        DESIRED_PLANNER.get("max_vel_x")
    )
    new_vel.linear.y = convert_range(
        data.linear.y,
        INPUT_PLANNER.get("min_vel_x"),
        INPUT_PLANNER.get("max_vel_x"),
        DESIRED_PLANNER.get("min_vel_x"),
        DESIRED_PLANNER.get("max_vel_x")
    )
    new_vel.linear.z = convert_range(
        data.linear.z,
        INPUT_PLANNER.get("min_vel_x"),
        INPUT_PLANNER.get("max_vel_x"),
        DESIRED_PLANNER.get("min_vel_x"),
        DESIRED_PLANNER.get("max_vel_x")
    )

    if float(new_vel.linear.x) == 0.0 and float(new_vel.linear.y) and float(new_vel.linear.z):
        new_vel.angular.x = convert_range(
            data.angular.x,
            INPUT_PLANNER.get("min_vel_theta"),
            INPUT_PLANNER.get("max_vel_theta"),
            DESIRED_PLANNER.get("min_vel_theta"),
            DESIRED_PLANNER.get("max_vel_theta")
        )
        new_vel.angular.y = convert_range(
            data.angular.y,
            INPUT_PLANNER.get("min_vel_theta"),
            INPUT_PLANNER.get("max_vel_theta"),
            DESIRED_PLANNER.get("min_vel_theta"),
            DESIRED_PLANNER.get("max_vel_theta")
        )
        new_vel.angular.z = convert_range(
            data.angular.z,
            INPUT_PLANNER.get("min_vel_theta"),
            INPUT_PLANNER.get("max_vel_theta"),
            DESIRED_PLANNER.get("min_vel_theta"),
            DESIRED_PLANNER.get("max_vel_theta")
        )
    else:
        new_vel.angular.x = convert_range(
            data.angular.x,
            (-1)*INPUT_PLANNER.get("min_in_place_vel_theta"),
            INPUT_PLANNER.get("min_in_place_vel_theta"),
            (-1)*DESIRED_PLANNER.get("min_in_place_vel_theta"),
            DESIRED_PLANNER.get("min_in_place_vel_theta")
        )
        new_vel.angular.y = convert_range(
            data.angular.y,
            INPUT_PLANNER.get("min_in_place_vel_theta"),
            INPUT_PLANNER.get("min_in_place_vel_theta"),
            DESIRED_PLANNER.get("min_in_place_vel_theta"),
            DESIRED_PLANNER.get("min_in_place_vel_theta")
        )
        new_vel.angular.z = convert_range(
            data.angular.z,
            INPUT_PLANNER.get("min_in_place_vel_theta"),
            INPUT_PLANNER.get("min_in_place_vel_theta"),
            DESIRED_PLANNER.get("min_in_place_vel_theta"),
            DESIRED_PLANNER.get("min_in_place_vel_theta")
        )

    cmd_vel_pub.publish(new_vel)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.Subscriber("/inter_cmd_vel", Twist, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()