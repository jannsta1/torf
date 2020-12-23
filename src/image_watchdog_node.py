#!/usr/bin/env python2
from __future__ import division

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyx4_base.pyx4_base import *
bridge = CvBridge()


class ImageWatchdog(object):

    def __init__(self,
                 camera_topic_name="/resize_img/image",
                 ):  # sub class args
        """
        This module was created as the bluefox camera can fail - especially in cold conditions.
        :param camera_topic_name:
        """

        self.camera_topic_name = camera_topic_name

        self._run_rate = 1
        self.image_check_interval_s = 5
        self.latest_im_check_time_s = rospy.get_time()

        msg_received = False
        # get initial image to generate our pixel sampling location bbased on the image height/width
        while not msg_received and not rospy.is_shutdown():
            try:
                image_msg = rospy.wait_for_message(self.camera_topic_name, Image, timeout=20)
                im = bridge.imgmsg_to_cv2(image_msg)
                # get the characteristics of our image
                self.im_height = image_msg.height
                self.im_width = image_msg.width
                # initialise the pixel checking arrays
                self.pixels2check_ver = np.ceil(np.linspace(1, self.im_height-1, 10)).astype(np.uint8)
                self.pixels2check_hor = np.ceil(np.linspace(1, self.im_width-1, 10)).astype(np.uint8)
                self.pixel_vals_this = im[self.pixels2check_ver, self.pixels2check_hor]
                self.pixel_vals_previous = np.clip(self.pixel_vals_this + 10, 0, 255)
                msg_received = True
            except rospy.ROSException as e:
                rospy.logwarn_throttle(5, 'camera watchdog node timed out waiting for image message \
                                       - traceback was {}'.format(e))
            # except e:
            #     rospy.logwarn(('{} happened'.format(e)))

        self.downcam_sub = rospy.Subscriber(self.camera_topic_name, Image, self.downcam_callback, queue_size=5)

        ## todo - implement bottom clearance sensor checker - perhaps create another node for this?
        # add flag to set / unset altitude sensor check
        self.altitude_bottom_clearance = Float32()
        self.alt_sub = rospy.Subscriber('mavros/altitude', Altitude, self.altitude_callback, queue_size=5)


    def downcam_callback(self, data):

        # if we are initialised, removed this as we now initialise torf before this state can be read
        if (self.latest_im_check_time_s + self.image_check_interval_s) < rospy.get_time():
            self.latest_im_check_time_s = rospy.get_time()
            this_image = bridge.imgmsg_to_cv2(data)
            self.pixel_vals_this = this_image[self.pixels2check_ver, self.pixels2check_hor]
            if np.median(self.pixel_vals_this) > 250:
                rospy.logwarn('Image watchdog check fail - Think images might be saturated - is it cold?')
            if np.all(self.pixel_vals_previous == self.pixel_vals_this):
                rospy.loginfo('Image is the same as previous one - Check that video feed is moving')
            self.pixel_vals_previous = self.pixel_vals_this


    def altitude_callback(self, data):
        """
        In this callback we check if the lidar sensor is online - the bottom clearance data is set to nan if we are
        using barometric pressure
        :param data:
        :return:
        """
        self.altitude = data
        self.altitude_bottom_clearance = np.float64(data.bottom_clearance)
        if np.isnan(self.altitude_bottom_clearance):
            # message is often enough that it should alert the user but shouldn't swamp the console
            rospy.logwarn_throttle(0.5, 'bottom clearance is invalid type: {}'.format(self.altitude_bottom_clearance))


    def run(self):
        """
        Just stays alive while ros is running so that we can continue to process incoming images
        """
        rate = rospy.Rate(self._run_rate)
        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except:
                break


if __name__ == '__main__':

    node_namespace = 'image_watchdog_node'
    rospy.init_node(node_namespace, log_level=rospy.DEBUG)
    iwd = ImageWatchdog()
    iwd.run()
