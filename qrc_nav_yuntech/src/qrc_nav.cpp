#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <string>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

double goalLocate[7][2];  // 存儲目標位置，包括點 1~6

// 從參數伺服器讀取目標位置
void getparam(ros::NodeHandle &nh)
{
    nh.getParam("/Point1/x", goalLocate[1][0]);
    nh.getParam("/Point1/y", goalLocate[1][1]);
    nh.getParam("/Point2/x", goalLocate[2][0]);
    nh.getParam("/Point2/y", goalLocate[2][1]);
    nh.getParam("/Point3/x", goalLocate[3][0]);
    nh.getParam("/Point3/y", goalLocate[3][1]);
    nh.getParam("/Point4/x", goalLocate[4][0]);
    nh.getParam("/Point4/y", goalLocate[4][1]);
    nh.getParam("/Point5/x", goalLocate[5][0]);
    nh.getParam("/Point5/y", goalLocate[5][1]);
    nh.getParam("/Point6/x", goalLocate[6][0]);
    nh.getParam("/Point6/y", goalLocate[6][1]);

    // 調試輸出
    printf("\n -------------------------\n");
    for (int i = 1; i <= 6; ++i)
    {
        printf("Point %d: x=%f, y=%f\n", i, goalLocate[i][0], goalLocate[i][1]);
    }
    printf("-------------------------\n");
}

// 發送導航目標
void sendnavgoal(double navgoal[2])
{
    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("等待 move_base 操作伺服器啟動");
    }

    move_base_msgs::MoveBaseGoal goal;

    // 設定目標位置
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = navgoal[0];
    goal.target_pose.pose.position.y = navgoal[1];
    goal.target_pose.pose.position.z = 0;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal to x: %f, y: %f", navgoal[0], navgoal[1]);
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("到達目標");
    }
    else
    {
        ROS_INFO("未能到達目標");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_point_nav");
    ros::NodeHandle nh;

    getparam(nh);  // 讀取導航目標

    std::string navInput;
    while (ros::ok())
    {
        printf("\n輸入要導航的點位序列 (1-6，例如 361)： ");
        std::cin >> navInput;

        bool validInput = true;
        for (char c : navInput)
        {
            if (c < '1' || c > '6')  // 檢查是否輸入有效點位
            {
                validInput = false;
                break;
            }
        }

        if (validInput)
        {
            for (char c : navInput)
            {
                int navtable = c - '0';  // 將字符轉為整數
                ROS_INFO("導航至點位 %d", navtable);
                sendnavgoal(goalLocate[navtable]);  // 發送導航目標
            }
        }
        else
        {
            printf("輸入無效。請輸入 1 到 6 的數字序列\n");
        }
    }

    return 0;
}

