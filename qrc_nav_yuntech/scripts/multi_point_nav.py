#!/usr/bin/env python3
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

# 儲存目標位置
goalLocate = {
    1: (-6.43, -3.22),
    2: (-6.37, 3.16),
    3: (-3.36, 4.03),
    4: (1.12, 3.14),
    5: (5.57, 1.24),
    6: (6.00, -4.30),
}

def get_params():
    global goalLocate
    for i in range(1, 7):
        x = rospy.get_param(f"/Point{i}/x", 0.0)
        y = rospy.get_param(f"/Point{i}/y", 0.0)
        goalLocate[i] = (x, y)
    rospy.loginfo("導航點位參數: %s", goalLocate)

def send_nav_goal(x, y):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0
    rospy.loginfo(f"導航至目標: x={x}, y={y}")
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_state() == actionlib.GoalStatus.SUCCEEDED
        
def main():
    rospy.init_node('multi_point_nav')
    get_params()  # 獲取導航目標參數

    while not rospy.is_shutdown():
        # 接受用戶輸入的導航點序列
        nav_input = input("\n輸入要導航的點位序列 (1-6，例如 361)： ")

        # 驗證輸入是否有效
        if not nav_input.isdigit() or any(c not in "123456" for c in nav_input):
            rospy.loginfo("輸入無效，請輸入 1 到 6 的數字序列")
            continue

        for char in nav_input:
            point_index = int(char)
            rospy.loginfo(f"導航至點位 {point_index}")
            x, y = goalLocate.get(point_index, (0, 0))
            success = send_nav_goal(x, y)
            if not success:
                rospy.loginfo(f"導航到點位 {point_index} 失敗")
                break


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

