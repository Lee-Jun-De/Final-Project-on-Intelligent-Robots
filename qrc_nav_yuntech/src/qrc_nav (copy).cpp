#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

double goalLocate[5][2];  // 存儲目標位置

// 從參數伺服器讀取目標位置
void getparam(ros::NodeHandle &nh)
{
    nh.getParam("/Table1/x", goalLocate[1][0]);
    nh.getParam("/Table1/y", goalLocate[1][1]);
    nh.getParam("/Table2/x", goalLocate[2][0]);
    nh.getParam("/Table2/y", goalLocate[2][1]);
    nh.getParam("/Table3/x", goalLocate[3][0]);
    nh.getParam("/Table3/y", goalLocate[3][1]);
    nh.getParam("/Table4/x", goalLocate[4][0]);
    nh.getParam("/Table4/y", goalLocate[4][1]);

    // 調試輸出
    printf("\n -------------------------\n home %f %f \n table1 %f %f\n table2 %f %f\n table3 %f %f\n table4 %f %f\n -------------------------\n ", goalLocate[0][0], goalLocate[0][1], goalLocate[1][0], goalLocate[1][1], goalLocate[2][0], goalLocate[2][1], goalLocate[3][0], goalLocate[3][1], goalLocate[4][0], goalLocate[4][1]);
}

// 設定導航目標
int setNavTable()
{
    printf("\n---------TIRT2022--------\n|PRESSE A KEY:           |\n|1  Table1                |\n|2  Table2                |\n|3  Table3                |\n|4  Table4                |\n------------------------\n Where to go\n");

    int navTable;
    std::cin >> navTable;

    while (navTable < 1 || navTable > 4)
    {
        printf("\nERR Input: %d \n please Enter 1-4 ", navTable);
        std::cin >> navTable;
    }

    return navTable;
}

// 發送導航目標
void sendnavgoal(double navgoal[2])
{
    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    // 設定目標位置
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = navgoal[0];
    goal.target_pose.pose.position.y = navgoal[1];
    goal.target_pose.pose.position.z = 0;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal x %f y %f", navgoal[0], navgoal[1]);
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The base reached the target");
    }
    else
    {
        ROS_INFO("The base failed to reach the target");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "qrc_nav");
    ros::NodeHandle nh;

    getparam(nh);  // 讀取導航目標

    int navtable;
    while (ros::ok())
    {
        navtable = setNavTable();  // 用戶選擇導航目標

        // 根據用戶選擇的目標，從該目標開始依次導航到後面的目標
        for (int i = navtable; i <= 4; i++)  // 從選擇的目標開始，依次導航
        {
            sendnavgoal(goalLocate[i]);  // 發送導航目標
        }

        ROS_INFO("Completed navigating through selected tables.");
    }

    return 0;
}

