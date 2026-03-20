import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/laptop10/Desktop/ws_1/install/cylinder_detector_pkg'
