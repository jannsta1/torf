#!/usr/bin/env python3

# global imports
import os

# local imports
from autorun_base import *

# parse test arguments
args_dict = parse_repeat_cwssim_model(sys.argv[1:], required_parsers=('tester', 'sm'))

# Generate the test arrangement
lower = args_dict['tester'].lower
args_dict['tester'].worlds = ['forest_plane_fine.world']
args_dict['tester'].repeats = 10

args_dict['sm'].duration_out = 20
args_dict['sm'].t_familiar = 2.0

# iterate through tests
for this_height in np.arange(2.5, 15.5, 2.5):
    args_dict['sm'].mission_hgt = this_height
    save_name = os.path.join('~', '.ros', '')
    test_var_name = 'altitude_' + str(this_height) + 'm'
    these_results = do_autorun(args_dict['tester'], args_dict['sm'], autosort=True, test_variable=test_var_name)
    print(these_results)
