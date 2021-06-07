#!/usr/bin/env python3
# ***************************************************************************

# ***************************************************************************/

#
# @author Jan Stankiewicz
#


# global imports
from argparse_sweep_mission import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cwssim_container import *
from threading import Thread
import rosparam
import sys

# local imports
from definitions_cwssim import Homing_outcome, CWSSIM_STATE, MULTIPROCESSING_STATUS
from sweep_mission import generate_sweep_mission
from torf.msg import Cwssim_status
from pyx4_base.pyx4_base_classes import Pyx4_base
from pyx4_base.definitions_pyx4 import State_estimation_method

bridge = CvBridge()

# handle different python version implementations of Queue:
try:
   from queue import Queue
except ImportError:
   import queue as Queue


class TorfCwssimHomingMission(Pyx4_base):
    """
    A class that provides logic for the transverse oscillating route following approach outlined in (paper pending)

    See 'argparse_sweep_mission.py' for details of input arguments

    ...

    Attributes
    ----------

    Methods
    -------


    """
    def __init__(self,
                 flight_instructions,
                 ros_rate=100,
                 node_namespace='cwssim_node',
                 camera_topic_name="/resize_img/image",
                 max_outbound_images=120,
                 state_estimation_mode=State_estimation_method.GPS,
                 use_multi_processing=True,
                 close_to_end_thresh_high=25,
                 close_to_end_thresh_low=5,
                 flip_images=True,  # flip images so that we can do mission in there and back fashion :. true by default
                 im_w=235,
                 im_h=150,
                 resize_w=None,
                 resize_h=None,
                 winsize=7,
                 levels=5,
                 nbands=2,
                 ):  # sub class args

        self.use_multiprocessing = use_multi_processing
        self.multiprocessing_status = MULTIPROCESSING_STATUS.NOT_INIT
        self.max_outbound_images = max_outbound_images

        self.nest_dist = 0.0  # note - this has been added purely to allow the sin_wave_2 state to be used

        # general attributes
        self.node_namespace = node_namespace
        self.ros_rate = rospy.Rate(ros_rate)
        self.enforce_sem_mode_flag = False
        self.hgt_initialised = False
        self.consec_invalid_hgts_cnt = 0
        self.global_compass_hdg_deg = np.nan

        # camera attributes
        self.camera_topic_name = camera_topic_name
        self.resize_w = resize_w
        self.resize_h = resize_h
        # to prevent unecessry resize operation:
        if self.resize_w == im_w and self.resize_h == im_h:
            self.resize_w = None
            self.resize_h = None
        self.image_q = Queue()
        self.new_image_evt = False
        self.flip_images = flip_images
        if self.flip_images:
            self.need2reverse_image_idxs = False
        else:
            self.need2reverse_image_idxs = True
        self.this_image = np.zeros((im_h, im_w), dtype=np.uint8)

        # torf attributes
        self.close_to_end_thresh_high = close_to_end_thresh_high
        self.close_to_end_thresh_low = close_to_end_thresh_low

        # cwssim attributes
        self.cwssim_mission_outcome = Homing_outcome.UNFINISHED_UNSPECIFIED.value  # used by autorun package
        self.cwssim_state = CWSSIM_STATE.NOT_INIT
        self.cwssim_score = 0.0
        self.cwssim_score_idx = self.close_to_end_thresh_high + 1  # wanted to send nan here but doesn't seem to be supported by ros
        self.sweep_index = 0
        self.best_cwssim_score_previous_run = 0.0
        self.cwssim_score = 0.0
        self.cwssim_score_idx = 0.0
        self.cwssim_status = Cwssim_status()
        self.cwssim_status_pub = rospy.Publisher(self.node_namespace + '/cwssim_status', Cwssim_status, queue_size=2)

        # Initialise cwssim class
        self.cwssim = Cwsim_container(im_h=im_h,
                                      im_w=im_w,
                                      max_depth=self.max_outbound_images,
                                      winsize=winsize,
                                      levels=levels,
                                      nbands=nbands,
                                      )
        rospy.loginfo("cwssim container loaded")

        # intialise the pyx4 state machine
        super(TorfCwssimHomingMission, self).__init__(
            flight_instructions,
            rospy_rate=2,
            mavros_ns='',
            start_authorised=False,
            state_estimation_mode=state_estimation_mode,
        )
        rospy.loginfo('CWSSIM initialised')

        # make sure required ROS resources are available
        count = 0
        while not self.mavros_interface.wd_initialised and count < 200:
            rospy.logwarn('[CX odometry] : waiting for mavros node to initialise ')
            if rospy.is_shutdown() or not self.node_alive:
                self.shut_node_down(shutdown_message="MAVROS didn't initialise")
            rospy.sleep(2.0)
            count += 1
        rospy.loginfo('MAVROS detected by torf')

        wait_for_imtopic_s = 60
        try:
            rospy.loginfo('waiting {} for camera topic {} to be published'.format(wait_for_imtopic_s, self.camera_topic_name))
            image_msg = rospy.wait_for_message(self.camera_topic_name, Image, timeout=wait_for_imtopic_s)
            rospy.loginfo('receing our image messgae {} with encoding {}'.format(self.camera_topic_name, image_msg.encoding))
        except Exception as e:
            self.shut_node_down(shutdown_message='camera topics not detected shutting down node')

        ################################################################################
        # end of initialisation (all class attributes should be declared by now)
        ################################################################################

        ################################################################################
        # start class threads
        ################################################################################

        # ROS subscribers - this should be initialise last in init function to avoid race conditions
        self.downcam_sub = rospy.Subscriber(self.camera_topic_name, Image, self.downcam_callback, queue_size=5)

        rospy.loginfo('starting cwssim thread')
        self.cwssim_wrap_thread = Thread(target=self.torf_logic, args=())
        self.cwssim_wrap_thread.daemon = True
        self.cwssim_wrap_thread.start()

        # wait a couple of seconds for model to initialise
        rospy.loginfo('waiting a few seconds then starting')
        rospy.sleep(3.0)
        rospy.loginfo('initialise delayed start')
        self.do_delayed_start()

    def initialise_memory_bank_for_multiprocessing(self):
        """
        This method must be run before cwssim can be used with multiple processes/threads
        :return:
        """
        self.multiprocessing_status = MULTIPROCESSING_STATUS.INITIALISING
        rospy.loginfo('start preparing memory bank for multi-processing')
        self.cwssim.prepare_memory_bank_outside()
        self.multiprocessing_status = MULTIPROCESSING_STATUS.INITIALISED
        rospy.loginfo('memory bank ready for multi-processing')

    def publish_cwssim_status(self):
        """
        A method to publish information relating to the most recent CWSSIM operation onto the ROS server
        """
        # publish torf status message
        self.cwssim_status.header.stamp = rospy.Time.now()
        self.cwssim_status.state = self.cwssim_state._name_
        self.cwssim_status.score = self.cwssim_score
        self.cwssim_status.index = int(self.cwssim_score_idx)
        self.cwssim_status.stored_qty = int(self.cwssim.qty_images_stored)
        self.cwssim_status.sweep_index = int(self.sweep_index)
        self.cwssim_status.best_score_previous_run = self.best_cwssim_score_previous_run
        # self.cwssim_status.longitudinal_slowdown_factor =
        # self.cwssim_status.lateral_slowdown_factor =
        self.cwssim_status_pub.publish(self.cwssim_status)

    def torf_logic(self):
        """
        Main logic for torf model used in conjunction with the CWSSIM view matching pipeline.

        Handles CWSSIM image processing depending on the flight state

        """
        while not rospy.is_shutdown() and self.node_alive:

            # Get flight state to see what phase of the mission we are in
            this_state_label = self.commander._flight_instruction.state_label

            # if we are in the outbound/learning flight phase then add new images to our memory database
            if this_state_label == 'Head_out':
                self.cwssim_state = CWSSIM_STATE.OUTBOUND
                if not self.image_q.empty():
                    rospy.logdebug('adding image')
                    self.cwssim.add_image(self.image_q.get())
                    self.publish_cwssim_status()
                # check that we are not lagging behind the image processing task
                if self.image_q.qsize() > 5:
                    rospy.logwarn(
                        'getting behind on image processing - queue size is: {}'.format(self.image_q.qsize()))

            # if we are in the inbound/homing flight phase then interrogate new images for familiarity against the
            # images in our memory database
            elif this_state_label == 'Sweep_state':
                self.cwssim_state = CWSSIM_STATE.INBOUND
                if self.new_image_evt:
                    self.new_image_evt = False
                    if self.use_multiprocessing:
                        if self.multiprocessing_status == MULTIPROCESSING_STATUS.INITIALISED:
                            self.cwssim_score, self.cwssim_score_idx = \
                                self.cwssim.max_query_image_with_index_mp(self.this_image)
                        else:
                            rospy.logwarn_throttle(1, "memory bank not initialised yet - can't do multiprocessing")
                            if self.multiprocessing_status == MULTIPROCESSING_STATUS.NOT_INIT:
                                rospy.logwarn_throttle(1, "trying to initialised memory bank but this should have "
                                                          "already been done - there is an error in the code")
                                self.initialise_memory_bank_for_multiprocessing()
                    else:
                        rospy.logwarn_throttle(5, 'not using multiprocessing, fewer images possible')
                        self.cwssim_score, self.cwssim_score_idx = \
                            self.cwssim.max_query_image_with_index_mp(self.this_image)
                    self.publish_cwssim_status()

            elif (this_state_label == 'Turn around time') or (this_state_label == 'Return to start'):
                self.cwssim_state = CWSSIM_STATE.TURNING_AROUND
                self.new_image_evt = False                         # clear the last image
                if self.multiprocessing_status == MULTIPROCESSING_STATUS.NOT_INIT:
                    if self.need2reverse_image_idxs:
                        rospy.logwarn("flip images before preparing memory bank for multiprocessing")
                    else:
                        rospy.loginfo("preparing memory bank for multiprocessing")
                        self.initialise_memory_bank_for_multiprocessing()
            else:
                self.cwssim_state = CWSSIM_STATE.INTERUPTION

            # if we n
            if self.need2reverse_image_idxs and \
                (this_state_label=='Turn around time' or this_state_label=='Return to start'):
                rospy.logwarn('started reversing CWSSIM image ids')
                self.cwssim.reverse_image_ids()
                self.need2reverse_image_idxs = False
                rospy.logwarn('Completed reversing CWSSIM image ids')

            try:
                self.ros_rate.sleep()
            except:
                # prevent gabble being printed to the terminal
                break

        self.cwssim_state = CWSSIM_STATE.FINISHED
        # do not do tidy up - if pyx4 also calls this it seems to cause problems
        # self.tidy_up()
        rospy.logwarn('setting mission complete parameter to true')
        rosparam.set_param('cwssim_test_complete', "true")
        self.shut_node_down()

    def tidy_up(self):
        """
        Run when this flight state is finished
        Overides pyx4 'tidy_up' method
        """
        rospy.loginfo('tidying up torf_ros.py')
        self.cwssim.tidy_up()
        rospy.logwarn('tidy up torf_ros.py completed')
        rosparam.set_param('cwssim_test_complete', "true")

    # ###########################################
    # # ROS callback functions
    # ###########################################
    def downcam_callback(self, data):
        """
        Deals with incoming images - if we are on the outbound route images are put in a learned image database. If we
        are on an outbound route the image is put into a processing queue. Images received at all other times are
        ignored
        :param data: ROS Image message data
        """
        if self.cwssim_state == CWSSIM_STATE.INBOUND or self.cwssim_state == CWSSIM_STATE.OUTBOUND:

            # optionally resize image subscribed from ROS
            if self.resize_h is not None and self.resize_w is not None:
                self.this_image[:, :] = (cv2.resize(bridge.imgmsg_to_cv2(data), (self.resize_h, self.resize_w)))
            else:
                self.this_image[:, :] = bridge.imgmsg_to_cv2(data)

            # if we are in inbound state - check if the previous image has been processed yet
            if self.new_image_evt and self.cwssim_state == CWSSIM_STATE.INBOUND:
                rospy.logwarn('previous image not yet processed')

            self.new_image_evt = True

            if self.cwssim_state == CWSSIM_STATE.OUTBOUND:
                if not self.cwssim.memory_full:
                    # a queue is used so that photos can be processed without a risk of the buffer being overwritten
                    if self.flip_images:
                        # we optionally flip the images so that we can use the outbound route as a learning route
                        self.image_q.put(np.rot90(self.this_image, 2))
                    else:
                        self.image_q.put(self.this_image)
                else:
                    rospy.logwarn_throttle(3, 'Cwssim memory capacity now full - not logging any further messages')
                    if self.multiprocessing_status == MULTIPROCESSING_STATUS.NOT_INIT and not self.need2reverse_image_idxs:
                        rospy.logwarn('Initialise memory bank for proc')
                        self.initialise_memory_bank_for_multiprocessing()

    def compass_hdg_callback(self, data):
        """
        subscribe quadcopter compass data from PX4 firmware
        :param data:
        :return:
        """
        self.global_compass_hdg_deg = data.data


if __name__ == '__main__':

    node_namespace = 'cwssim_node'
    rospy.init_node(node_namespace, anonymous=True, log_level=rospy.DEBUG)
    args = sm_argparse(sys_argv_in=sys.argv)

    sm_flight_instructions = generate_sweep_mission(args_in=args)

    print(("Parsed flight instructions:\n\n", sm_flight_instructions, "\n"))
    print(('sweep mission args:\n\n', args, "\n"))
    cwssim_class = TorfCwssimHomingMission(
                            flight_instructions=sm_flight_instructions,
                            node_namespace=node_namespace,
                            use_multi_processing=args.multi_process,
                            max_outbound_images=args.outbound_images,
                            camera_topic_name=args.camera_topic,
                            close_to_end_thresh_high=args.close_to_end_thresh_high,
                            close_to_end_thresh_low=args.close_to_end_thresh_low,
                            flip_images=args.flip_images,
                            winsize=args.winsize,
                            im_w=args.im_w,
                            im_h=args.im_h,
                            resize_w=args.resize_w,
                            resize_h=args.resize_h,
                            levels=args.levels,
                            nbands=args.nbands,
                        )

    cwssim_class.run()
    cwssim_class.tidy_up()
