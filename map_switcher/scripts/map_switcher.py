#!/usr/bin/env python
import rospy
import os
from subprocess import Popen, call
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient

class MapSwitcher:
    def __init__(self):
        rospy.init_node('map_switcher', anonymous=True)

        # 設置導航客戶端
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("Move Base server is ready.")

        # 儲存啟動文件路徑
        self.launch_files = {
            "world": "turtlebot3_gazebo turtlebot3_world.launch",
            "house": "turtlebot3_gazebo turtlebot3_house.launch"
        }
        self.current_process = None

        # 目標點位 (在 world 地圖中的點位)
        self.goal_world = {'x': 1.0, 'y': 1.0}  # world 地圖的導航點

    def set_goal(self, x, y):
        """設定導航目標點"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo(f"Sending goal: ({x}, {y})")
        self.client.send_goal(goal)
        self.client.wait_for_result()

        return self.client.get_result()

    def switch_to_house(self):
        """切換到 house 地圖"""
        rospy.loginfo("Switching to house map...")

        # 關閉目前的 Gazebo 啟動節點
        if self.current_process:
            rospy.loginfo("Shutting down current Gazebo environment...")
            self.current_process.terminate()
            self.current_process.wait()
            rospy.sleep(5)

        # 啟動新的地圖
        rospy.loginfo("Launching house map...")
        self.current_process = Popen(["roslaunch"] + self.launch_files["house"].split())

        # 等待新地圖加載完成
        rospy.sleep(10)
        rospy.loginfo("House map is ready.")

    def run(self):
        """主程序執行邏輯"""
        # 啟動 world 地圖
        rospy.loginfo("Launching world map...")
        self.current_process = Popen(["roslaunch"] + self.launch_files["world"].split())
        rospy.sleep(10)  # 等待 world 地圖啟動完成

        # 導航到指定點位
        rospy.loginfo("Navigating to the goal in world map...")
        self.set_goal(self.goal_world['x'], self.goal_world['y'])

        rospy.loginfo("Reached goal in world map.")
        rospy.sleep(2)

        # 切換到 house 地圖
        self.switch_to_house()

if __name__ == '__main__':
    try:
        map_switcher = MapSwitcher()
        map_switcher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Map switcher interrupted.")

