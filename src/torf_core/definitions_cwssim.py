#!/usr/bin/env python3
# ***************************************************************************

# ***************************************************************************/

#
# @author Jan Stankiewicz
#


from enum import Enum
import os

CWSSIM_ROOT = os.path.dirname(os.path.abspath(__file__))

DATA = os.path.join(CWSSIM_ROOT, '..', '..', 'data')
IM_SEQUENCES = os.path.join(DATA, 'im_sequences')
RESULTS = os.path.join(CWSSIM_ROOT, 'results')
ROSLAUNCH = os.path.join(CWSSIM_ROOT, '..', '..', 'launch')

HOME_DIR = os.path.expanduser("~")
ROSLOG = os.path.join(HOME_DIR, '.ros')

# enums
class Homing_outcome(Enum):
    SUCCESFUL_MISSION = 0
    UNFINISHED_UNSPECIFIED = 1
    DID_NOT_CONVERGE = 2
    DID_NOT_CONVERGE_TIMEOUT = 3
    DID_NOT_CONVERGE_SEARCH_PATTERN = 4
    INIT_PROBLEM_OPTIC_FLOW = 5
    BAD_HEIGHT_DATA = 6
    DID_NOT_CONVERGE_SEARCH_PATTERN_TRUNCATED = 7


class CWSSIM_STATE(Enum):
    NOT_INIT = 0
    PRE_OUT = 1
    OUTBOUND = 2
    INTERUPTION = 3
    INBOUND = 4
    FINISHED = 5
    UKNOWN = 6
    TURNING_AROUND = 7


class MULTIPROCESSING_STATUS(Enum):
    UNKNOWN = 0
    NOT_INIT = 1
    INITIALISING = 2
    INITIALISED = 3