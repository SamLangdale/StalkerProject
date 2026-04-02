import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sam/stalkerProject/StalkerProject/install/3d_lidar'
