#!/usr/bin/env python3
# helper file for running autorun in a particular world

from autorun_base import *

args_dict = parse_repeat_cwssim_model(sys.argv[1:], required_parsers=('tester', 'sm'))

# Generate the test arangement
lower = args_dict['tester'].lower

args_dict['tester'].worlds = 'Seville.world'  # +forest_plane_fine.world+forest.world+Furkastrasse_retextured.world'
args_dict['tester'].repeats = 1

args_dict['sm'].duration_out = 15

# iterate through tests
these_results = do_autorun(args_dict['tester'], args_dict['sm'], autosort=True)
