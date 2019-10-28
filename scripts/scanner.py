#!/usr/bin/env python

import rospy
import time

from gclib_ros.msg import Position
from gclib_ros.srv import MultiAxisMotion
from std_srvs.srv import Trigger,TriggerResponse,TriggerRequest
from rospix.srv import Exposure
from long_cell_scanner.srv import SetString
from std_msgs.msg import Float64

class LongCellScanner:

    def mainTimer(self, event):

        rospy.loginfo_once('Main timer spinning')

        self.publisher_fov.publish(self.image_size)
        self.publisher_overlap.publish(self.overlap)
        self.publisher_width.publish(self.image_size*self.width)
        self.publisher_height.publish(self.image_size*self.height)

    def callbackStart(self, event):

        rospy.loginfo_once('Starting')

        total_time = self.height * self.width * (self.exposure_time_per_position + 5)

        rospy.logerr('TOTAL TIME WILL BE: {} s'.format(total_time))

        position_idx = 0
        self.service_client_rename("{}".format(position_idx))
        time.sleep(1.0)

        self.started = True

        rospy.loginfo('going to 0 0')

        self.service_client_goto.call([0, 0])

        rospy.loginfo('Starting exposures')

        self.service_client_start_exposures.call(self.exposure_time_per_image)

        rospy.loginfo('Exposures started')

        time.sleep(self.exposure_time_per_position)

        rospy.loginfo('Stopping exposures')

        self.service_client_interrupt_measurement.call()

        rospy.loginfo('Exposures stopped')

        time.sleep(1.0)

        for y in range(-(self.height-1)/2, (self.height-1)/2+1):
            for x in range(-(self.width-1)/2, (self.width-1)/2+1):

                if x == 0 and y == 0:
                    continue

                position_idx += 1
                self.service_client_rename("{}".format(position_idx))
                time.sleep(1.0)

                rospy.loginfo('')
                rospy.logwarn('Starting new iteration: {}'.format(position_idx))
                rospy.loginfo('')

                x_deg = (x * self.image_size * self.overlap)
                y_deg = (y * self.image_size * self.overlap)

                rospy.loginfo('x y {} {} [step] {} {} [deg]'.format(x, y, x_deg, y_deg))

                self.service_client_goto.call([x_deg, y_deg])

                rospy.loginfo('Starting exposures')

                self.service_client_start_exposures.call(self.exposure_time_per_image)

                rospy.loginfo('Exposures started')

                time.sleep(self.exposure_time_per_position)

                rospy.loginfo('Stopping exposures')

                self.service_client_interrupt_measurement.call()

                time.sleep(1.0)

                rospy.loginfo('Exposures stopped')

                rospy.logerr('')
                rospy.logerr('Remaining {} s'.format(total_time - (position_idx + 1) * self.exposure_time_per_position))
                rospy.logerr('')

        rospy.loginfo('finished going to 0 0')

        self.service_client_goto.call([0, 0])

        rospy.logerr('FINISHED')
        rospy.logerr('FINISHED')
        rospy.logerr('FINISHED')
        rospy.logerr('FINISHED')

        return TriggerResponse(True, "Done")

    def __init__(self):

        rospy.init_node('long_cell_scanner', anonymous=True)

        # parameters
        main_timer_rate = rospy.get_param('~main_timer_rate')

        self.image_size = rospy.get_param('~image_size')
        self.overlap = 1.0 - float(rospy.get_param('~overlap'))/100.0
        self.width = rospy.get_param('~width')
        self.height = rospy.get_param('~height')
        self.exposure_time_per_position = rospy.get_param('~exposure_time_per_position')
        self.exposure_time_per_image = rospy.get_param('~exposure_time_per_image')

        self.service_client_goto = rospy.ServiceProxy("~goto_out", MultiAxisMotion)
        self.service_client_start_exposures = rospy.ServiceProxy("~start_exposures_out", Exposure)
        self.service_client_interrupt_measurement = rospy.ServiceProxy("~interrupt_measurement_out", Trigger)
        self.service_client_rename = rospy.ServiceProxy("~rename_out", SetString)

        self.publisher_fov = rospy.Publisher("~fov_out", Float64, queue_size=1)
        self.publisher_overlap = rospy.Publisher("~overlap_out", Float64, queue_size=1)
        self.publisher_width = rospy.Publisher("~width_out", Float64, queue_size=1)
        self.publisher_height = rospy.Publisher("~height_out", Float64, queue_size=1)

        self.started = False

        rospy.Service("~start_in", Trigger, self.callbackStart)

        rospy.Timer(rospy.Duration(1.0/main_timer_rate), self.mainTimer)

        rospy.spin()

if __name__ == '__main__':
    try:
        long_cell_scanner = LongCellScanner()
    except rospy.ROSInterruptException:
        pass
