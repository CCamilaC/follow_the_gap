import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/camila/ros2_ws/src/my_robot_test/install/my_robot_test'
