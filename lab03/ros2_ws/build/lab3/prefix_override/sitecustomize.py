import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/natalia/Documentos/GitHub/IR2136/lab03/ros2_ws/install/lab3'
