#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Allows passing of test repeater arguments (using the 'tester' keyword) while still enabling all sweep mission parameters
to be passed without having to maintain 2 arguments profiles

"""
from warnings import warn
import argparse
import rospy
import sys

from torf_core.argparse_sweep_mission import sm_argparse, str2bool

VALID_WORLD_LIST = ['flat', 'bumpy', 'Furkastrasse', 'Furkastrasse_textured', 'forest_plane', 'forest_plane_fine', 'Seville']


########### specialist parsing functions to enable user convenient argument passing:

def parse_worlds(worlds_arg_in, plus_symbol='+'):
    """
    returns a list containing gazebo worlds according to the input string. There are three options for command line
    input:
    1) "all" - returns all worlds from VALID_WORLD_LIST
    2) A string containing desired worlds separated by a "+" symbol, e.g. "flat+bumpy"
    3) A string containing a single world name (must be in the VALID_WORLD_LIST)
    """
    # first case - we want to test all worlds
    if worlds_arg_in == "all":
        world_list = VALID_WORLD_LIST
    # second case, we have a list of worlds seperated by & symbols
    elif plus_symbol in worlds_arg_in:
        world_list = worlds_arg_in.split(plus_symbol)
        print(('received these world arguments {}'.format(world_list)))
    # finally, a string has been input - convert this to a list
    elif type(worlds_arg_in) == str:
        world_list = [worlds_arg_in, ]
    else:
        raise TypeError("Unable to preocess world argument {} of type {}".format(worlds_arg_in, type(worlds_arg_in)))

    worlds_out = []
    for world_idx, world in enumerate(world_list):
        # strip world for consistent comparison
        if world.endswith('.world'):
            world = world[:-6]

        if world in VALID_WORLD_LIST:
            worlds_out.append(world + '.world')
        else:
            warn(argparse.ArgumentTypeError('Unrecognised world argument {}'.format(world)))

    if len(worlds_out) == 0:
        raise ('no worlds found from input {}'.format(worlds_out))

    return worlds_out


def parse_hgt_mode(hgt_mode_req_in):
    """
    parses command line height mode settings
    :param hgt_mode_req_in:
    :return:
    """
    if type(hgt_mode_req_in) == str:
        if hgt_mode_req_in == 'baro':
            return [0, ]
        elif hgt_mode_req_in in ['lidar', 'l']:
            return [2, ]
        elif hgt_mode_req_in == 'both':
            return [0, 2]
    else:
        raise argparse.ArgumentTypeError('Unrecognised hgt_mode_req argument {}'.format(hgt_mode_req_in))

########### Test repeat arguments
def repeat_test_parser(sys_argv_in, subparsers=None, return_args=True, started_by_ros=True,
                       lower_default=0.0, range_default=60.0, delta_default=10.0,
                       ):
    """
    parses test repeater arguments in a subparse
    :param sys_argv_in:
    :param subparsers:
    :param return_args:
    :param started_by_ros:
    :return:
    """
    if subparsers:
        this_parser = subparsers.add_parser('tester', description="This parses arguments for a central complex drone model.")
    else:
        this_parser = argparse.ArgumentParser('tester', description="This parses arguments for a central complex drone model.")

    this_parser.add_argument('-r', '--repeats', type=int, default=1)
    this_parser.add_argument('-g', '--gui', help=" todo ", type=str2bool, default=True)
    this_parser.add_argument('-v', '--vehicle', help="", type=str, default='typhoon_copy')
    this_parser.add_argument('-w', '--worlds', help="name of gazebo worlds to simulate, if multiple desired, join with + "
                             "symbol (don't leave spaces) e.g. bumpy.flat", type=parse_worlds, default=['forest_plane_fine.world',])
    this_parser.add_argument('-a', '--hgt_mode_req', type=parse_hgt_mode, default=[0, ], help=" todo ")
    this_parser.add_argument('-i', '--save_images', type=str2bool, default=False, help="Do we save image topics in our bag")

    # the following arguments are useful for iterating through a range of parameters
    this_parser.add_argument('-l', '--lower', type=float, default=lower_default, help="initial experimental values")
    this_parser.add_argument('-m', '--range', type=float, default=range_default, help="range of experimental values")
    this_parser.add_argument('-d', '--delta', type=float, default=delta_default, help="interval between experimental values")

    if not return_args:
        return
    if started_by_ros:
        args = this_parser.parse_args(rospy.myargv(argv=sys_argv_in)[1:])
    else:
        args = this_parser.parse_args(rospy.myargv(argv=sys_argv_in))

    return args, this_parser, subparsers


def sweep_mission_test_parser(sys_argv_in, subparsers=None, return_args=True, started_by_ros=True):
    """
    Wraps the main argparse_sweep_mission.py module - only required if this is the first subparser
    :param sys_argv_in:
    :param subparsers:
    :param return_args:
    :param started_by_ros:
    :return:
    """

    parser_description_str = "This parses arguments for the sweep mission."
    if subparsers:
        that_parser = subparsers.add_parser('sm', description=parser_description_str)
    else:
        that_parser = argparse.ArgumentParser('sm', description=parser_description_str)

    if not return_args:
        return

    if started_by_ros:
        args = that_parser.parse_args(rospy.myargv(argv=sys_argv_in)[1:])
    else:
        args = that_parser.parse_args(rospy.myargv(argv=sys_argv_in))

    return args, that_parser, subparsers


def get_repeater_parser(args, **kwargs):
    """
    Wraps the repeat_test_parser method - only required if this is the first subparser
    :param args:
    :param kwargs:
    :return:
    """
    parser = argparse.ArgumentParser(prog='TEST_REPEATER')
    subparsers = parser.add_subparsers(help='sub-command help', dest='subparser_name')

    repeat_test_parser(args, subparsers=subparsers, return_args=False, started_by_ros=False, **kwargs)

    return parser, subparsers

def parse_all_subparsers(parser, args, verbose=True):
    """
    Parses all arguments to each sub parser
    :param parser:
    :param args:
    :param verbose:
    :return:
    """
    arg_dict = {}
    while args:
        name = args[0]
        print(('parsing: ' + args[0]))
        options, args = parser.parse_known_args(args)
        arg_dict[name] = options
        if not options.subparser_name:
            break

    if verbose:
        for idx, val in enumerate(arg_dict):
            print((idx, val))
            # print (arg_dict['name_space'][idx])
            # print ('')

    return arg_dict


def parse_repeat_cwssim_model(args, required_parsers=('tester', 'sm'), **kwargs):
    """
    parses test repeat and torf arguments
    :param args:
    :param required_parsers:
    :param kwargs:
    :return:
    """
    for parser_str in required_parsers:
        if parser_str in args:
            continue
        else:
            print(('appending required parser {} to args'.format(parser_str)))
            args.append(parser_str)

    parser, subparsers = get_repeater_parser(args, **kwargs)
    sm_argparse(args, started_by_ros=False, return_args=False, subparsers=subparsers)
    arg_dict = parse_all_subparsers(parser, args)
    return arg_dict


if __name__ == '__main__':

    args = rospy.myargv(argv=sys.argv)[1:]
    arg_dict = parse_repeat_cwssim_model(args)

    print((arg_dict['sm']))
    print((arg_dict['tester']))