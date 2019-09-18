#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from laser_assembler.srv import AssembleScans2

class Laser2PC:
    def __init__(self):
        rospy.init_node("assemble_scans_to_cloud")
        rospy.loginfo("[L2PC] Laser to Point Cloud Running")

        self.lidar_scan = False
        self.main_rate  = rospy.Rate(60)

        ## Publisher
        self.g_point_cloud2_pub = rospy.Publisher("/robotis/sensor/assembled_scan",       PointCloud2, queue_size = 1)
        # self.g_final_pcl2_pub   = rospy.Publisher("/robotis/sensor/final_assembled_scan", PointCloud2, queue_size = 1)

        ## Subscriber
        rospy.Subscriber('/robotis/sensor/move_lidar', String, self.lidar_turn_callback)

        ## Service Client
        self.g_assemble_chest_laser_client = rospy.ServiceProxy("/robotis/sensor/service/assemble_scans2", AssembleScans2)

    def lidar_turn_callback(self, msg):
        if msg.data == "start":
            rospy.loginfo("[L2PC] Lidar scans start")
            self.lidar_scan = True
        elif msg.data == "end":
            rospy.loginfo("[L2PC] Lidar scans end")
            self.lidar_scan = False
  
    def assemble_laser_scans(self, start_time, end_time):
        rospy.wait_for_service("/robotis/sensor/service/assemble_scans2")
        try:
            assembler_output = self.g_assemble_chest_laser_client( start_time, end_time )
            self.g_point_cloud2_pub.publish(assembler_output.cloud)
        except rospy.ServiceException as e:
            rospy.logwarn("[L2PC] Service call failed {}".format(e))

    def run(self):
        # rospy.spin()
        while not rospy.is_shutdown():
            if self.lidar_scan:
                self.assemble_laser_scans(rospy.Time(0.0), rospy.get_rostime())
            self.main_rate.sleep()

if __name__ == '__main__':
    laser2pc = Laser2PC()
    laser2pc.run()