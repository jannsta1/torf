#!/usr/bin/env python2
# ***************************************************************************

# ***************************************************************************/

#
# @author Jan Stankiewicz
#
from __future__ import division

from sensor_msgs.msg import Image
from sweep_mission import generate_sweep_mission
from cx_model_px4.cx_odometry import Cx_odometry
from argparse_sweep_mission import *
from cv_bridge import CvBridge
from cx_model_px4.definitions_cx import *
from torf.cwssim_container import *
from lfdrone.msg import Cwssim_status
from pyx4_base.pyx4_base import *
from enum import Enum
import rosparam
bridge = CvBridge()

try:
   from queue import Queue
except ImportError:
   import Queue as Queue

class CWSSIM_STATE(Enum):
    NOT_INIT = 0               # cwwsim is not initilaised yet
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


class Cwssim_homing_mission(Pyx4_base):

    def __init__(self,
                 mission_args,
                 flight_instructions,
                 results_filename=None,
                 height_mode_req=0,
                 ros_rate=100, # only effects non optic flow operation
                 node_namespace='cwssim_node',
                 camera_topic_name="/resize_img/image",
                 max_outbound_images=120,
                 takeoff_delay=0,
                 cwssim_score_max_buffer_size=100,
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

        self.camera_topic_name = camera_topic_name

        self.use_multiprocessing = use_multi_processing
        self.multiprocessing_status = MULTIPROCESSING_STATUS.NOT_INIT
        self.max_outbound_images = max_outbound_images

        self.nest_dist = 0.0  # todo - this has been added in to allow the sin_wave_2 state to be employed

        self.resize_w = resize_w
        self.resize_h = resize_h

        if self.resize_w == im_w and self.resize_h == im_h:
            # to prevent unecessry resize operation
            self.resize_w = None
            self.resize_h = None

        # first initialise our torf model
        rospy.loginfo("loading torf model")
        self.cwssim = Cwsim_container(im_h=im_h,
                                      im_w=im_w,
                                      max_depth=self.max_outbound_images,
                                      winsize=winsize,
                                      levels=levels,
                                      nbands=nbands,
                                      )
        rospy.loginfo("torf model loaded")

        self.node_namespace = node_namespace

        self.close_to_end_thresh_high = close_to_end_thresh_high
        self.close_to_end_thresh_low = close_to_end_thresh_low

        # initialise data fields
        self.ros_rate = rospy.Rate(ros_rate)

        self.enforce_sem_mode_flag = False
        self.hgt_initialised = False
        self.consec_invalid_hgts_cnt = 0

        self.cwssim_score = 0.0
        self.cwssim_score_idx = self.close_to_end_thresh_high + 1  # wanted to send nan here but doesn't seem to be supported by ros
        self.sweep_index = 0
        self.best_cwssim_score_previous_run = 0.0


        # self.images_stored = maximum_outbound_images
        self.image_q = Queue()
        self.new_image_evt = False

        self.cwssim_state = CWSSIM_STATE.NOT_INIT

        self.flip_images = flip_images
        if self.flip_images:
            self.need2reverse_image_idxs = False
        else:
            self.need2reverse_image_idxs = True

        # todo - make proper state logic for this:
        self.homing_state = False
        self.cwssim_mission_outcome = Homing_outcome.UNFINISHED_UNSPECIFIED.value

        # make sure required filepaths exist
        if results_filename is None:
            self.results_filename = os.path.join(LATEST_SIMULTAION_DIR, 'cwssim_logger.npz')
        else:
            self.results_filename = results_filename

        # todo - don't think this is required for logbags? Check this out i.e. does the file save if dir doesnt already exist
        if not os.path.exists(LATEST_SIMULTAION_DIR):
            os.makedirs(LATEST_SIMULTAION_DIR)

        # intialise the cx model
        super(Cwssim_homing_mission, self).__init__(
            flight_instructions,
            rospy_rate=2,
            mavros_ns='',
            # enforce_height_mode_flag=self.enforce_height_mode_flag,
            # height_mode_req=height_mode_req,
            # enforce_sem_mode_flag=self.enforce_sem_mode_flag,
            start_authorised=False,
            state_estimation_mode=state_estimation_mode,
        )  # sub and super class args
        rospy.loginfo('CWSSIM initialised')

        # make sure required ROS resources are available
        count = 0
        while not self.mavros_interface.wd_initialised and count < 200:
            rospy.logwarn('[CX odometry] : waiting for mavros node to initialise ')
            if rospy.is_shutdown() or not self.node_alive:
                # todo - define unintended shutdown exit function
                self.save_on_close()
                self.shut_node_down()
            rospy.sleep(0.5)
            count += 1

        wait_for_imtopic_s = 60
        try:
            rospy.loginfo('waiting {} for camera topic {} to be published'.format(wait_for_imtopic_s, self.camera_topic_name))
            image_msg = rospy.wait_for_message(self.camera_topic_name, Image, timeout=wait_for_imtopic_s)
            rospy.loginfo('receing our image messgae {} with encoding {}'.format(self.camera_topic_name, image_msg.encoding))
        except Exception as e:
            rospy.signal_shutdown('camera topics not detected shutting down node')
            # sys.exit(1)

        self.cwssim_score = 0.0 #Float32()
        self.cwssim_score_idx = 0.0 #Float32()
        self.cwssim_status = Cwssim_status()
        # self.cwssim_score_pub = rospy.Publisher(self.node_namespace + '/cwssim_score', Float32, queue_size=2)
        self.cwssim_status_pub = rospy.Publisher(self.node_namespace + '/cwssim_status', Cwssim_status, queue_size=2)

        ################################################################################
        # end of initialisation (all class attributes should be declared by now)
        ################################################################################

        ################################################################################
        # start class threads
        ################################################################################

        # ROS subscribers - this should be initialise last in init function to avoid race conditions

        self.downcam_sub = rospy.Subscriber(self.camera_topic_name, Image, self.downcam_callback, queue_size=5)

        # start publish mission details wrapper thread
        self.publish_mission_spec_thread = Thread(target=Cx_odometry.publish_mission_spec,
                                                  args=(mission_args, self.node_namespace, self.mavros_interface))
        self.publish_mission_spec_thread.daemon = True
        self.publish_mission_spec_thread.start()

        rospy.loginfo('starting torf thread')
        self.cwssim_wrap_thread = Thread(target=self.cwssim_wrapper, args=())
        self.cwssim_wrap_thread.daemon = True
        self.cwssim_wrap_thread.start()

        # wait a couple of seconds for model to initialise
        rospy.loginfo('waiting a few seconds then starting')
        rospy.sleep(3.0)
        rospy.loginfo('initialise delayed start')
        self.do_delayed_start()


    @property
    def qty_images_stored(self):
        return self.cwssim.qty_images_stored


    def initialise_memory_bank_for_multiprocessing(self):
        # todo - add other pathways to this functionality - e.g. learning flight complete or just
        # outbound state exited
        self.multiprocessing_status = MULTIPROCESSING_STATUS.INITIALISING
        rospy.loginfo('start preparing memory bank for multi-processing')
        self.cwssim.prepare_memory_bank_outside()
        self.multiprocessing_status = MULTIPROCESSING_STATUS.INITIALISED
        rospy.loginfo('memory bank ready for multi-processing')


    def downcam_callback(self, data):

        # if we are initialised, removed this as we now initialise torf before this state can be read
        #if self.cwssim_state != CWSSIM_STATE.NOT_INIT:

        if self.cwssim_state == CWSSIM_STATE.INBOUND or self.cwssim_state == CWSSIM_STATE.OUTBOUND:

            # optionally resize image subscribed from ROS
            if self.resize_h is not None and self.resize_w is not None:
                self.this_image = (cv2.resize(bridge.imgmsg_to_cv2(data), (self.resize_h, self.resize_w)))
            else:
                self.this_image = bridge.imgmsg_to_cv2(data)

            # if we are in inbound state - check if the previous image has been processed yet
            if self.new_image_evt and self.cwssim_state == CWSSIM_STATE.INBOUND:
                rospy.logwarn('previous image not yet processed')

            self.new_image_evt = True

            if self.cwssim_state == CWSSIM_STATE.OUTBOUND:
                # todo - should there be a difference between multi and non-multiprocessing here?
                # print (self.torf.memory_full)
                if not self.cwssim.memory_full:
                    # a queue is used so that photos can be processed without a risk of the buffer being overwritten
                    if self.flip_images:
                        self.image_q.put(np.rot90(self.this_image, 2))  # we flip the images so that we can do a there an back route
                    else:
                        self.image_q.put(self.this_image)

                else:
                    rospy.logwarn_throttle(3, 'Cwssim memory capacity now full - not logging any further messages')

                    if self.multiprocessing_status == MULTIPROCESSING_STATUS.NOT_INIT and not self.need2reverse_image_idxs:
                        rospy.logwarn('Initialise memory bank for proc')
                        self.initialise_memory_bank_for_multiprocessing()

    def publish_cwssim_status(self):

        # publish torf status message
        self.cwssim_status.header.stamp = rospy.Time.now()
        self.cwssim_status.state = self.cwssim_state._name_
        self.cwssim_status.score = self.cwssim_score
        self.cwssim_status.index = self.cwssim_score_idx
        self.cwssim_status.stored_qty = self.qty_images_stored
        self.cwssim_status.sweep_index = self.sweep_index
        self.cwssim_status.best_score_previous_run = self.best_cwssim_score_previous_run

        # self.cwssim_status.longitudinal_slowdown_factor =
        # self.cwssim_status.lateral_slowdown_factor =
        self.cwssim_status_pub.publish(self.cwssim_status)

    def cwssim_wrapper(self, filtered_steps=0.0, cx_debug_update_rate=5):
        """
        ## brief
        # during a typical mission this central complex wrapper will publish the state of the network including its
        # inputs and outputs at each step

        # a typical mission will consist of arm/takeoff/outbound/inbound/land/shutdown states but it would ultimately be
        # useful to include substates e.g. obstacle avoidance

        # todo - add functionality to suppress model until a particular event e.g. state label == outbound this could be
        just setting velocity to zero when in particular states

        """
        while not rospy.is_shutdown() and self.node_alive:

            # Get flight state
            this_flight_state = self.commander._flight_instruction.flight_instruction_type
            this_state_label = self.commander._flight_instruction.state_label

            if this_state_label == 'Head_out':
                self.cwssim_state = CWSSIM_STATE.OUTBOUND
                if not self.image_q.empty():
                    rospy.logdebug('adding image')
                    self.cwssim.add_image(self.image_q.get())
                    self.publish_cwssim_status()

                # check we are not lagging behind the image processing task
                if self.image_q.qsize() > 5:
                    rospy.logwarn(
                        'getting behind on image processing - queue size is: {}'.format(self.image_q.qsize()))

            elif this_state_label == 'Sweep_state':
                self.cwssim_state = CWSSIM_STATE.INBOUND
                # if not self.image_q.empty():
                if self.new_image_evt == True:
                    # rospy.logdebug('testing image bank size is {}'.format(self.torf.qty_images_stored))
                    self.new_image_evt = False
                    if self.use_multiprocessing:
                        if self.multiprocessing_status == MULTIPROCESSING_STATUS.INITIALISED:
                            # self.cwssim_score.data = self.torf.max_query_image_mp(self.this_image)
                            self.cwssim_score, self.cwssim_score_idx = self.cwssim.max_query_image_with_index_mp(self.this_image)
                            # rospy.loginfo(self.cwssim_score_idx)
                        else:
                            rospy.logwarn_throttle(1, "memory bank not initialised yet - can't do multiprocessing")
                            if self.multiprocessing_status == MULTIPROCESSING_STATUS.NOT_INIT:
                                rospy.logwarn_throttle(1, "trying to initialised memory bank but this should have "
                                                          "already been done - there is an error in the code")
                                self.initialise_memory_bank_for_multiprocessing()
                    else:
                        rospy.logwarn_throttle(5, 'not using multiprocessing, fewer images possible')
                        self.cwssim_score, self.cwssim_score_idx = self.cwssim.max_query_image_with_index_mp(self.this_image)

                    self.publish_cwssim_status()
                    # todo - add "sweep number"
                    # todo - add dection here for state is 'finished'

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
                self.cwssim_state=CWSSIM_STATE.INTERUPTION


            if self.need2reverse_image_idxs and \
                (this_state_label=='Turn around time' or this_state_label=='Return to start'):   # or self.torf.memory_full
                rospy.logwarn('started reversing CWSSIM image ids')
                self.cwssim.reverse_image_ids()
                self.need2reverse_image_idxs = False
                rospy.logwarn('Completed reversing CWSSIM image ids')

            # self.torf.print_info()

            try:
                self.ros_rate.sleep()
            except:
                break

        # do not do tidy up - if pyx4 also calls this it seems to cause problems
        # self.tidy_up()
        rospy.logwarn('setting mission complete parameter to true')
        rosparam.set_param('cwssim_test_complete', "true")


    def tidy_up(self):

        print ('tidying up torf_ros.py')
        rospy.loginfo ('tidying up torf_ros.py')
        self.cwssim.tidy_up()
        print ('tidy up torf_ros.py completed')
        rospy.logwarn('tidy up torf_ros.py completed')
        rosparam.set_param('cwssim_test_complete', "true")
        # todo - set flag once tidying up has happened, then if it is called again we won't get stuck here (there may
        #  be a better method than this, e.g. is there a pool status we can check?)


        # print ('##########################################################################')
        # print ('########### Closing torf node ##############################################')
        # print ('mission outcome was {} : {}'.format(self.cx_mission_outcome, Homing_outcome(self.cx_mission_outcome)))
        # print ('##########################################################################')
        #
        # rosparam.set_param('cx_test_complete', "true")  # so that the test scehduler knows this test is complete (may be able to replace with rospy shutdown signal)


    # ###########################################
    # # ROS callback functions
    # ###########################################
    def compass_hdg_callback(self, data):
        self.global_compass_hdg_deg = data.data
    # def optic_flow_callback(self, data):
    #     self.optic_flow_q.put([data.speed_left, data.speed_right, data.time_between_frames_s])


if __name__ == '__main__':


    node_namespace = 'cwssim_node'
    rospy.init_node(node_namespace, anonymous=True, log_level=rospy.DEBUG )
    args = sm_argparse(sys_argv_in=sys.argv)

    sm_flight_instructions = generate_sweep_mission(args_in=args)

    print ("Parsed flight instructions:")
    print (sm_flight_instructions)
    print ('sweep mission args', args)
    cwssim_class = Cwssim_homing_mission(
                            mission_args=args,
                            flight_instructions=sm_flight_instructions,
                            # height_mode_req=args.height_mode_req,
                            # takeoff_delay=args.takeoff_delay,
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