#!/usr/bin/env python2
# ***************************************************************************

# ***************************************************************************/

#
# @author Jan Stankiewicz
#


from enum import Enum
import os

CWSSIM_ROOT = os.path.dirname(os.path.abspath(__file__))

DATA = os.path.join(CWSSIM_ROOT, 'data')
IM_SEQUENCES = os.path.join(DATA, 'im_sequences')
RESULTS = os.path.join(CWSSIM_ROOT, 'results')
ROSLAUNCH = os.path.join(CWSSIM_ROOT, 'launch')

HOME_DIR = os.path.expanduser("~")
ROSLOG = os.path.join(HOME_DIR, '.ros')