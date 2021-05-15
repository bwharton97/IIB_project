"""Constants that must be shared between sever and pi scripts"""
PI_IP_ADDRESSES = ['10.42.0.171', '10.42.0.239']
SERVER_IP = '10.42.0.1'
#RESOLUTION = (1920, 1080)
#RESOLUTION = (640, 480)
# Be careful of changing resolution or framerate and loading old video
RESOLUTION = (1640, 922)
FRAMERATE = 30
STAT_FREQ = 3  # In seconds. For streaming analysis