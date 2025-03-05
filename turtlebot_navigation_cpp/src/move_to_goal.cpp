#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <unistd.h>

// 定義一個動作客戶端類型，用於導航
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

double goalLocate[7][2]; // 存儲各個導航點的位置
std::string zbarCBMsg = ""; // 存儲從zbar獲取的QRcode訊息
bool keepRunning = true; // 控制程式是否繼續運行的變量

// 從參數獲取導航點位置的函數
void getparam(ros::NodeHandle &nh) {
    nh.getParam("/home/x", goalLocate[0][0]);
    nh.getParam("/home/y", goalLocate[0][1]);
    nh.getParam("/Table1/x", goalLocate[1][0]);
    nh.getParam("/Table1/y", goalLocate[1][1]);
    nh.getParam("/Table2/x", goalLocate[2][0]);
    nh.getParam("/Table2/y", goalLocate[2][1]);
    nh.getParam("/Table3/x", goalLocate[3][0]);
    nh.getParam("/Table3/y", goalLocate[3][1]);
    nh.getParam("/Table4/x", goalLocate[4][0]);
    nh.getParam("/Table4/y", goalLocate[4][1]);
    nh.getParam("/Table5/x", goalLocate[5][0]);
    nh.getParam("/Table5/y", goalLocate[5][1]);
    nh.getParam("/home2/x", goalLocate[6][0]);
    nh.getParam("/home2/y", goalLocate[6][1]);
    
    printf("\n -------------------------\n 出發區 %f %f \n table1 %f %f\n table2 %f %f\n table3 %f %f\n table4 %f %f\n table5 %f %f\n 備餐區 %f %f \n -------------------------\n ", 
           goalLocate[0][0], goalLocate[0][1], goalLocate[1][0], goalLocate[1][1], goalLocate[2][0], goalLocate[2][1], 
           goalLocate[3][0], goalLocate[3][1], goalLocate[4][0], goalLocate[4][1], goalLocate[5][0], goalLocate[5][1], 
           goalLocate[6][0], goalLocate[6][1]);
}

// 設置導航目標
void sendnavgoal(double navgoal[2], bool waitForQR) {
    MoveBaseClient ac("move_base", true);
    
    // 等待行動伺服器啟動
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    // 設定導航目標
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = navgoal[0];
    goal.target_pose.pose.position.y = navgoal[1];
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal x %f y %f", navgoal[0], navgoal[1]);
    ac.sendGoal(goal);
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, reached the target");
    } else {
        ROS_INFO("Failed to reach the target");
    }

    if (waitForQR) {
        sleep(3); // 等待3秒模擬掃描QRcode
        zbarCBMsg = "";  // 重置QRcode訊息以等待新的掃描
        while (zbarCBMsg == "") {
            ros::spinOnce(); // 保持旋轉直到接收到QRcode
        }
        ROS_INFO("QR Code scanned: %s", zbarCBMsg.c_str());
    }
}

// zbar回調函數，用來處理QR碼掃描結果
void zbarCallback(const std_msgs::String::ConstPtr &msg) {
    zbarCBMsg = msg->data;
}

// 讓使用者輸入導航目的地桌號
int setNavTable() {
    printf("\n---------TIRT2024--------\n|PRESSE A KEY:           |\n|1  Table1               |\n|2  Table2               |\n|3  Table3               |\n|4  Table4               |\n|5  Table5               |\n------------------------\nWhere to go: ");

    float navTable;
    scanf("%f", &navTable);
    navTable = (int)navTable;

    while (navTable < 1 || navTable > 5) {
        printf("\nERR Input: %2.0f \n 請輸入1-5: ", navTable);
        scanf("%f", &navTable);
        navTable = (int)navTable;
    }

    return navTable;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_navigation_goals");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("zbar_opencv_code", 1000, zbarCallback);

    getparam(nh); // 獲取導航點位置
    int navtable;

    // 主循環
    while (ros::ok() && keepRunning) {
        // 第一次導航到輸入的桌子
        navtable = setNavTable(); // 獲取目的地桌號
        sendnavgoal(goalLocate[navtable], true); // 導航到桌子並等待QR碼掃描

        // 不論是否掃描到none，繼續導航
        sendnavgoal(goalLocate[6], false); // 導航到home2，不等待QR碼
        char temp[10];
        std::cout << "在確認物品放置後按任意鍵+ENTER繼續...";
        std::cin >> temp; // 等待使用者確認

        // 第二次導航回到同一桌子
        sendnavgoal(goalLocate[navtable], true); // 導航到桌子並等待QR碼
        sendnavgoal(goalLocate[6], false); // 導航到home2，不等待QR碼
        std::cout << "在確認物品放置後按任意鍵+ENTER繼續...";
        std::cin >> temp;

        // 第三次導航回到同一桌子，等待none作為QRcode
        sendnavgoal(goalLocate[navtable], true); // 導航到桌子並等待QR碼
        if (zbarCBMsg == "none") {
            ROS_INFO("QR Code: none received, returning to home");
            sendnavgoal(goalLocate[0], false); // 回到home點
        }

        keepRunning = false; // 結束循環
        ros::spinOnce(); // 處理ROS事件
    }

    ROS_INFO("Program terminating...");
    return 0;
}

