import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/matija-pongracic/PAS/PAS_DZ_02/install/path_planning'
