import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ericcardinal/trial_project/src/talker_listener/install/talker_listener'
