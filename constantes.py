import numpy as np


#===============================================#
#                                               #
#             Structural parameters             #
#                                               #
#===============================================#

LF = 0.792
LR = 0.738
WHEELBASE = LF + LR

FRONT_AXLE = np.array([LF, 0.0])


#===============================================#
#                                               #
#                Steer related                  #
#                                               #
#===============================================#

STANLEY_DEN = 0.01
STANLEY_GAIN = 0.25

STEERING_ANGLE_TO_STEER_WHEEL = 3.5

MIN_STEERING_ANGLE = -np.deg2rad(28.0)
MAX_STEERING_ANGLE =  np.deg2rad(28.0)


#===============================================#
#                                               #
#                 Speed related                 #
#                                               #
#===============================================#

SPEED_KP = 1.0
SPEED_KI = 1.0
SPEED_KD = 0.2

MIN_SPEED = 0.0
MAX_SPEED = 12.0

MAX_LATERAL_ACC =  4.0
