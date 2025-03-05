from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO
import rospy
from geometry_msgs.msg import Twist
import subprocess
import os
import signal
import sys
sys.path.append('/home/junde/catkin_ws/src/qrc_nav_yuntech/scripts')  # 添加多點導航模組的路徑
from multi_point_nav import goalLocate, get_params, send_nav_goal  # 匯入模組


# 初始化 Flask 和 SocketIO
app = Flask(__name__)
socketio = SocketIO(app)

# ROS 初始化
rospy.init_node('web_interface', anonymous=True)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# 儲存 Gazebo、RViz 和導航的進程 PID
gazebo_process = None
rviz_process = None
navigation_process = None
qrc_nav_process = None

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/create_map')
def create_map():
    global gazebo_process, rviz_process
    map_type = request.args.get('map_type', '')
    if map_type == 'world':
        gazebo_command = ['roslaunch', 'turtlebot3_gazebo', 'turtlebot3_world.launch']
    elif map_type == 'house':
        gazebo_command = ['roslaunch', 'turtlebot3_gazebo', 'turtlebot3_house.launch']
    else:
        return "未知的地圖類型", 400

    try:
        # 啟動對應的 Gazebo 地圖
        gazebo_process = subprocess.Popen(gazebo_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # 啟動 gmapping
        subprocess.Popen(
            ['roslaunch', 'turtlebot3_slam', 'turtlebot3_slam.launch', 'slam_methods:=gmapping'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        # 啟動 RViz
        rviz_process = subprocess.Popen(['rosrun', 'rviz', 'rviz'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return render_template('create_map.html', back_url='/')
    except Exception as e:
        return f"地圖啟動失敗：{e}", 500

@app.route('/shutdown', methods=['POST'])
def shutdown():
    global gazebo_process, rviz_process, navigation_process, qrc_nav_process
    try:
        # 關閉 Gazebo 進程
        if gazebo_process:
            os.killpg(os.getpgid(gazebo_process.pid), signal.SIGTERM)
            gazebo_process = None

        # 關閉 RViz 進程
        if rviz_process:
            os.killpg(os.getpgid(rviz_process.pid), signal.SIGTERM)
            rviz_process = None

        # 關閉導航進程
        if navigation_process:
            os.killpg(os.getpgid(navigation_process.pid), signal.SIGTERM)
            navigation_process = None

        # 關閉 QRC Nav 進程
        if qrc_nav_process:
            os.killpg(os.getpgid(qrc_nav_process.pid), signal.SIGTERM)
            qrc_nav_process = None

        return jsonify({'message': 'Gazebo、RViz、導航和 QRC Nav 進程已成功關閉！'})
    except Exception as e:
        return jsonify({'message': f'關閉過程失敗：{e}'}), 500

@app.route('/save_map', methods=['POST'])
def save_map():
    map_name = request.json.get('map_name', '')
    if not map_name:
        return jsonify({'status': 'error', 'message': '地圖名稱不能為空'}), 400

    try:
        # 儲存地圖的指令，將地圖保存到指定路徑
        map_saver_command = ['rosrun', 'map_server', 'map_saver', '-f', f'/home/junde/catkin_ws/map/{map_name}']
        subprocess.run(map_saver_command, check=True)
        return jsonify({'status': 'success', 'message': f'地圖 "{map_name}" 已成功儲存！'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'儲存地圖失敗：{e}'}), 500

@app.route('/get_maps', methods=['GET'])
def get_maps():
    map_dir = '/home/junde/catkin_ws/map'
    try:
        # 列出指定目錄中的所有 .yaml 文件
        files = [f for f in os.listdir(map_dir) if f.endswith('.yaml')]
        return jsonify({'maps': files})
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/start_navigation', methods=['POST'])
def start_navigation():
    global navigation_process, gazebo_process, qrc_nav_process
    map_name = request.json.get('map_name', '')
    map_dir = '/home/junde/catkin_ws/map'
    map_file = os.path.join(map_dir, map_name)

    if not map_name or not os.path.exists(map_file):
        return jsonify({'error': '地圖不存在！'}), 400

    try:
        # 根據地圖啟動對應的 Gazebo 環境
        if 'house' in map_name:
            gazebo_command = ['roslaunch', 'turtlebot3_gazebo', 'turtlebot3_house.launch']
        else:
            gazebo_command = ['roslaunch', 'turtlebot3_gazebo', 'turtlebot3_world.launch']

        # 啟動 Gazebo 模擬環境
        gazebo_process = subprocess.Popen(gazebo_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        # 啟動導航功能
        navigation_process = subprocess.Popen(
            ['roslaunch', 'turtlebot3_navigation', 'turtlebot3_navigation.launch', f'map_file:={map_file}'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        # 啟動 QRC Nav
        qrc_nav_process = subprocess.Popen(
            ['roslaunch', 'qrc_nav', 'qrc_nav.launch'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        return jsonify({'message': f'導航啟動成功，地圖: {map_name}，並已啟動 QRC Nav！'})
    except Exception as e:
        return jsonify({'error': f'導航啟動失敗：{e}'}), 500

@app.route('/navigate_to_point', methods=['GET'])
def navigate_to_point_page():
    return render_template('navigate_to_point.html')

@app.route('/navigate_to_point', methods=['POST'])
def navigate_to_point():
    data = request.json
    point_sequence = data.get('point_sequence', '')

    if not point_sequence or not point_sequence.isdigit():
        return jsonify({'error': '無效的點位序列，請輸入數字序列！'}), 400

    try:
        get_params()  # 獲取導航參數
        for point in point_sequence:
            point_index = int(point)
            if point_index not in goalLocate:
                return jsonify({'error': f'點位 {point_index} 不存在！'}), 400

            x, y = goalLocate[point_index]
            success = send_nav_goal(x, y)

            if not success:
                return jsonify({'error': f'導航至點位 {point_index} 失敗！'}), 500

        return jsonify({'message': '導航完成！'})
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@socketio.on('move_robot')
def move_robot(data):
    twist = Twist()
    if data['direction'] == 'forward':
        twist.linear.x = 0.2
    elif data['direction'] == 'backward':
        twist.linear.x = -0.2
    elif data['direction'] == 'left':
        twist.angular.z = 0.5
    elif data['direction'] == 'right':
        twist.angular.z = -0.5
    else:
        twist.linear.x = 0
        twist.angular.z = 0
    cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000)

