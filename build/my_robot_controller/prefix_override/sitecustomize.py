import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/joshros/Desktop/ros2cc_ws/install/my_robot_controller'