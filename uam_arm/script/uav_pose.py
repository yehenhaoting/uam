#!/usr/bin/env python

import rospy
import csv
from matplotlib import pyplot as plt
from geometry_msgs.msg import PoseStamped


UAV_POS = PoseStamped()


def pos_talker():

    file_pose = '/home/uav/uam_ws/src/uam_arm/script/vehicle_local_position.csv'
    with open(file_pose) as f_pose:
        reader = csv.reader(f_pose)
        header_row = next(reader)
        print (header_row)
        first_row = next(reader)
        print (first_row)
        i=1
        sumx, sumy, sumz = 0.0, 0.0, 0.0
        time, x, y, z = [], [], [], []
        for row in reader:
            t1 = int(row[0])
            if 7.98e8 < t1 <8.16e8:
                time.append(t1)
                x.append((float(row[4])+2.663)/5)
                y.append((float(row[5])+6.414)/5)
                z.append((float(row[6])+5.653)/5+0.05)
                sumx = sumx + float(row[4])
                sumy = sumy + float(row[5])
                sumz = sumz + float(row[6])
                i=i+1

    nn=1
    mm=1
    file_rot = '/home/uav/uam_ws/src/uam_arm/script/vehicle_attitude.csv'
    with open(file_rot) as f_rot:
        reader2 = csv.reader(f_rot)
        header2_row = next(reader2)
        print (header2_row)
        first2_row = next(reader2)
        print (first2_row)
        time2, rot_x, rot_y, rot_z, rot_w = [], [], [], [], []
        for row2 in reader2:
            t2 = int(row2[0])
            if 7.98e8 < t2 <8.16e8:
                if not nn % 3:
                    time2.append(t2)
                    rot_x.append((float(row2[5])))
                    rot_y.append((float(row2[6])))
                    rot_z.append((float(row2[7])))
                    rot_w.append((float(row2[4])))
                    mm=mm+1
                nn=nn+1

    pub = rospy.Publisher('/uav/uav_pose', PoseStamped, queue_size=10)
    rospy.init_node('LOG_pose', anonymous=True)
    rate = rospy.Rate(20) # 20hz
    ii=0
    while not rospy.is_shutdown():
        UAV_POS.pose.position.x = x[ii]
        UAV_POS.pose.position.y = y[ii]
        UAV_POS.pose.position.z = z[ii]
        UAV_POS.pose.orientation.x = rot_x[ii]
        UAV_POS.pose.orientation.y = rot_y[ii]
        UAV_POS.pose.orientation.z = rot_z[ii]
        UAV_POS.pose.orientation.w = rot_w[ii]
        ii = ii + 1
        if ii == (i-1):
            ii=0
        pub.publish(UAV_POS)
        rate.sleep()

if __name__ == '__main__':
    try:
        pos_talker()
    except rospy.ROSInterruptException:
        pass