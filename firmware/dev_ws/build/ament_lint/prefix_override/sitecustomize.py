import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/prhayogo/Downloads/Nexus_Gazebo/firmware/dev_ws/install/ament_lint'
