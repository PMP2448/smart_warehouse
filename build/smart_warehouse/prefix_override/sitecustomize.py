import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pmp/ros2_ws/src/smart_warehouse/install/smart_warehouse'
