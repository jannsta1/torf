#!/usr/bin/env python3

# global imports
from torf.msg import Cwssim_status
import numpy as np
import roslaunch
import rosgraph
import time
import os

# local imports
from arg_parser import *

# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from torf_core.definitions_cwssim import ROSLAUNCH, ROSLOG
from torf_core.utils import date_string

"""
A script for automating several runs of the torf_ros node in a simulation environment using the roslaunch api

"""


def get_uuid():
    return roslaunch.rlutil.get_or_generate_uuid(None, False)


class Cwssim_launch_node(object):
    ''' A helper class to handle the roslaunch api for the cwssim_simulate.launch file '''
    def __init__(self,
                 run_id=None,
                 cwssim_args=None,
                 world="forest_plane_fine.world",
                 vehicle="typhoon_copy",
                 gui=True,
                 bag_prefix='',
                 save_images=False,
                 ):

        if run_id is None:
            self.run_id = get_uuid()
        else:
            self.run_id = run_id

        self.__roslaunch_arguments_dict = vars(cwssim_args)
        self.__roslaunch_arguments_dict['world'] = world
        self.__roslaunch_arguments_dict['vehicle'] = vehicle
        self.__roslaunch_arguments_dict['gui'] = gui
        self.__roslaunch_arguments_dict['bag_prefix'] = bag_prefix
        self.__roslaunch_arguments_dict['save_images'] = save_images
        self.__roslaunch_arguments_dict['main_image_width'] = cwssim_args.im_w
        self.__roslaunch_arguments_dict['main_image_height'] = cwssim_args.im_h

        rospy.init_node('mav_launch', anonymous=True)
        self.launch = roslaunch.parent.ROSLaunchParent(self.run_id, [os.path.join(ROSLAUNCH, 'cwssim_simulate.launch')])  # ["/home/jan/px4/Firmware/launch/mavros_posix_sitl.launch"])

    def load_roslaunch_arguments_list(self):
        """
        place required arguments in the sys.argv environment
        :return:
        """
        for key, val in list(self.__roslaunch_arguments_dict.items()):
            # print key, type(key), val, type(val)
            item = key + ":= " + str(val)
            sys.argv.append(item)

    def clean_roslaunch_arguments_list(self):
        """
        clean up arguments from the sys.argv environment
        :return:
        """
        for key, val in list(self.__roslaunch_arguments_dict.items()):
            item = key + ":= " + str(val)
            sys.argv.remove(item)

    def spin_up(self):
        """
        launch the roslaunch file
        :return:
        """
        self.load_roslaunch_arguments_list()
        # print 'ROS sys.argv is now : '
        # print sys.argv
        # sys.argv.append("vehicle:=" + self.vehicle_name)
        self.launch.start()
        rospy.loginfo('JS - launching ')

    def clean_up(self):
        """
        clean up - should be used each time this class is used
        :return:
        """
        self.launch.shutdown()
        self.clean_roslaunch_arguments_list()


class WatchdogTimer(object):
    """
    A backup mechanism for shutting down each simulation run in case it hangs.
    """
    def __init__(self,
                 duration=250,
                 ):

        self.timed_out = False
        self.timer = rospy.Timer(rospy.Duration(duration), self.callback)

    def callback(self, timer):
        self.timed_out = True


def do_autorun(test_args, cwssim_args, save_name=None, autosort=False, test_id=None, test_variable=None):
    """
    runs the simulate torf launch file 'repeats' times with the logfiles saved in the default location (~/.ros).

    The procedure is repeated in all of the required worlds as defined in the test_args Namespace

    :param repeats: how many times each test is repeated
    :return:
    """
    normal_exit_summary = np.zeros(test_args.repeats)

    for world in test_args.worlds:

        for jdx in range(test_args.repeats):

            rospy.loginfo('running main test with index {}, inside a while loop until shutdown detected'.format(jdx))

            run_id = get_uuid()
            print ('starting roscore')
            parent = roslaunch.parent.ROSLaunchParent(run_id, [], is_core=True)
            if rosgraph.is_master_online():  # Checks the master uri and results boolean (True or False)
                print ('ROS MASTER is already online no need to restart')
            else:
                print('ROS MASTER is offline, starting new roscore')
                parent.start()
            print('roscore started')

            print('Launching torf simulate ')
            if save_name:
                bag_prefix = save_name + '_' + world[0:-6] + '_r_' + str(jdx) + '.bag'  # todo - make world optional - perhaps create subfolder for each test run
            elif autosort:
                # if test_variable:

                if not test_id:
                    test_id = date_string()
                bag_path = os.path.join(ROSLOG, test_id, world[0:-6], str(test_variable))
                if not os.path.exists(bag_path):
                    try:
                        os.makedirs(bag_path)
                    except OSError:
                        # if e.errno != errno.EEXIST:
                        raise OSError()
                bag_prefix = os.path.join(bag_path, 'r_' + str(jdx))
                # else:
                #     if not test_id:
                #         test_id = ""
                #     bag_prefix = os.path.join(ROSLOG, test_id, world[0:-6], 'r_' + str(jdx))
                    # raise IOError("the 'test_variable' parameter must be passed in autosorting mode")
            else:
                bag_prefix = 'sweep_mission_' + world[0:-6] + '_r_' + str(jdx)

            print(('saving too {}'.format(bag_prefix)))
            # print (bag_path)
            print (ROSLOG)
            print (ROSLAUNCH)

            lf = Cwssim_launch_node(run_id=run_id, cwssim_args=cwssim_args,
                                    world=world,
                                    vehicle=test_args.vehicle,
                                    gui=test_args.gui,
                                    bag_prefix=bag_prefix,
                                    save_images=test_args.save_images,
                                    )
            lf.spin_up()

            # set the test complete parameter to False
            cx_test_complete_param_str = 'cwssim_test_complete'
            rospy.set_param(cx_test_complete_param_str, False)

            try:
                rospy.wait_for_message('cwssim_node/cwssim_status', Cwssim_status, 60)
            except Exception as e:
                rospy.logwarn(e)
                rospy.loginfo('timed out waiting for torf status - will try next bit anyway')

            try:
                rospy.loginfo('All nodes initiliased succesfully - starting our test loop')
                wd = WatchdogTimer()
                while True:
                    if rospy.is_shutdown():
                        rospy.logerr(('ROS shutdown request - qutting auto mission'))
                        normal_exit_summary[jdx] = 2.0
                        break

                    mish_comp = rospy.get_param(cx_test_complete_param_str, False)
                    if mish_comp:
                        rospy.logwarn('terminating mission as we think its finished')
                        normal_exit_summary[jdx] = 1.0
                        break

                    if wd.timed_out:
                        rospy.logerr('Watchdog timer elapsed - quitting')
                        break

                    rospy.logwarn_throttle(20, 'running main test with index {}, inside a while loop until shutdown detected'.format(jdx))
                    rospy.sleep(1.0)

            except Exception:
                rospy.logerr('Problem with loop {} in auto test'.format(jdx))

            rospy.loginfo('waiting 10s before cleanup')
            time.sleep(10)

            rospy.loginfo('cleaning up learning flight')
            try:
                lf.launch.shutdown()
            except Exception:
                rospy.logerr('not cleaning up lf node - suspect this has already terminated')

            rospy.loginfo('shutting parent down')
            parent.shutdown()
            rospy.loginfo('waiting 10s before next loop')
            time.sleep(10)

    return normal_exit_summary


if __name__ == '__main__':

    args_dict = parse_repeat_cwssim_model(sys.argv[1:], required_parsers=('tester', 'sm'))
    do_autorun(args_dict['tester'], args_dict['sm'])