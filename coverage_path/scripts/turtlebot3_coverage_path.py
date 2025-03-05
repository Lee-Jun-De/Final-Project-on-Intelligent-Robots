#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
import numpy as np

# 全局變數
map_data = None
resolution = 0.0
origin = None
pub_cmd_vel = None
stop_flag = False

def map_callback(data):
    """
    地圖數據回調函數
    """
    global map_data, resolution, origin
    rospy.loginfo("Map received!")
    map_data = np.array(data.data).reshape((data.info.height, data.info.width))
    resolution = data.info.resolution
    origin = (data.info.origin.position.x, data.info.origin.position.y)
    rospy.loginfo("Map dimensions: %dx%d, Resolution: %f", data.info.width, data.info.height, resolution)

def laser_callback(data):
    """
    雷射數據回調函數，檢測障礙物
    """
    global stop_flag
    min_distance = min(data.ranges)
    if min_distance < 0.01:  # 距離小於0.01米時停止
        rospy.logwarn("Obstacle detected! Stopping...")
        stop_robot()
        stop_flag = True
    else:
        stop_flag = False

def stop_robot():
    """
    停止機器人
    """
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub_cmd_vel.publish(twist)

def generate_coverage_path(grid):
    """
    生成覆蓋路徑 (之字形)，避開障礙物
    """
    path = []
    rows, cols = grid.shape
    for r in range(rows):
        if r % 2 == 0:  # 偶數行，從左到右
            for c in range(cols):
                if grid[r, c] == 0:  # 僅處理可通行區域
                    path.append((r, c))
        else:  # 奇數行，從右到左
            for c in range(cols - 1, -1, -1):
                if grid[r, c] == 0:
                    path.append((r, c))
    return path

def grid_to_world(grid_coord, resolution, origin):
    """
    將網格座標轉換為世界座標
    """
    world_x = grid_coord[1] * resolution + origin[0]
    world_y = grid_coord[0] * resolution + origin[1]
    return (world_x, world_y)

def move_to_goal(x, y, pub):
    """
    發送目標點到 move_base
    """
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.w = 1.0  # 朝向正方向
    rospy.loginfo("Sending goal: (%f, %f)", x, y)
    pub.publish(goal)
    rospy.sleep(1)  # 給導航足夠時間處理

def main():
    global map_data, pub_cmd_vel, stop_flag
    rospy.init_node('turtlebot3_coverage_path', anonymous=True)

    # 訂閱地圖話題
    rospy.Subscriber('/map', OccupancyGrid, map_callback)

    # 訂閱雷射掃描話題
    rospy.Subscriber('/scan', LaserScan, laser_callback)

    # 發佈控制指令到 /cmd_vel
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # 發佈目標點到 move_base
    pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    rospy.loginfo("Waiting for map...")
    rospy.sleep(5)  # 等待地圖加載

    if map_data is None:
        rospy.logerr("Map not received. Exiting...")
        return

    # 生成覆蓋路徑
    rospy.loginfo("Generating coverage path...")
    path_in_grid = generate_coverage_path(map_data)
    path_in_world = [grid_to_world(coord, resolution, origin) for coord in path_in_grid]

    # 移動至每個目標點
    rospy.loginfo("Executing path...")
    for waypoint in path_in_world:
        if stop_flag:
            rospy.logwarn("Paused due to obstacle. Waiting...")
            while stop_flag:
                rospy.sleep(0.1)  # 等待障礙物移除
        move_to_goal(waypoint[0], waypoint[1], pub_goal)

    rospy.loginfo("Coverage path completed!")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

