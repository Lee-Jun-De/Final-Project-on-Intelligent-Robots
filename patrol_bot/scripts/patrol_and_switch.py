#!/usr/bin/env python
import rospy
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
import actionlib

# 巡邏點（包括門口）
patrol_points = [
    {"name": "hallway_point1", "x": 1.0, "y": 2.0, "theta": 0.0},
    {"name": "classroom1_door", "x": 3.0, "y": 1.5, "theta": 1.57, "map": "classroom1"},
    {"name": "classroom2_door", "x": 5.0, "y": 2.0, "theta": 0.0, "map": "classroom2"}
]

current_pose = None

# 回調函數：更新機器人當前位置
def pose_callback(data):
    global current_pose
    current_pose = data.pose.pose

# 判斷是否到達目標點
def is_at_goal(goal_x, goal_y, threshold=0.5):
    global current_pose
    if not current_pose:
        return False
    distance = math.sqrt((current_pose.position.x - goal_x)**2 + (current_pose.position.y - goal_y)**2)
    return distance < threshold

# 發送導航目標點
def move_to_goal(x, y, theta):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = math.sin(theta / 2.0)
    goal.target_pose.pose.orientation.w = math.cos(theta / 2.0)

    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

# 切換地圖
def switch_map(map_name):
    rospy.set_param('/map_file', f'/path/to/maps/{map_name}.yaml')
    rospy.loginfo(f"Switched to map: {map_name}")

if __name__ == '__main__':
    rospy.init_node('patrol_and_map_switcher')

    # 訂閱AMCL
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)

    # 巡邏
    for point in patrol_points:
        rospy.loginfo(f"Navigating to {point['name']}...")
        if move_to_goal(point['x'], point['y'], point['theta']):
            rospy.loginfo(f"Reached {point['name']}")

            # 如果是門口，進行地圖切換
            if "map" in point:
                rospy.loginfo(f"At door: {point['name']}. Switching to map: {point['map']}")
                switch_map(point['map'])
        else:
            rospy.logwarn(f"Failed to reach {point['name']}")

