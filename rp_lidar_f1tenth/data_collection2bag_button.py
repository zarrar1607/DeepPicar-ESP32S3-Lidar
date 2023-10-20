'''
Data from ackermann
lidar
'''
import rospy
import rosbag
import message_filters
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

import csv

drive = '/vesc/low_level/ackermann_cmd_mux/input/teleop'
joy = '/vesc/joy'
lid = '/scan_filtered' #'/scan_filtered' #'/scan'
#f_lid = '/scan_filtered' 
#odom = '/vesc/odom'

prev = 0
curr = 0
pressed = False
bag = None


print('Starting to collect data...')
rospy.init_node('receive_position')

def callback(ack_msg, ldr_msg):#, odm_msg):
    #global i
    #global bag
    #print(f'S:{ack_msg}')
    if(pressed and bag is not None):
        #print(f'{i} Pressed')
        
        print(f'S:{ack_msg}')
        bag.write('Ackermann',ack_msg)
        bag.write('Lidar',ldr_msg)
'''
def f_callback(ack_msg, ldr_msg):#, odm_msg):
    #global i
    #global bag
    #print(f'S:{ack_msg}')
    if(pressed and bag is not None):
        #print(f'{i} Pressed')

        print(f'S:{ack_msg}')
        bag.write('F_Ackermann',ack_msg)
        bag.write('F_Lidar',ldr_msg)
'''
def button_callback(j):
    global pressed
    global prev
    global curr
    global start
    global bag

    curr = j.buttons[0]
    if(curr == 1 and curr!=prev):
        pressed = not pressed
        #print(f'Pressed')
        if(pressed):
            #bag.close()
            bag = rosbag.Bag('rp_lidar_out.bag','w')
            print('Start')
        else:
            bag.close()
            print('Stop')
    prev = curr
    
drive_sub = message_filters.Subscriber(drive, AckermannDriveStamped)
lid_sub = message_filters.Subscriber(lid, LaserScan)

ts = message_filters.ApproximateTimeSynchronizer([drive_sub, lid_sub],\
        queue_size = 10, slop = 0.01, allow_headerless=True)

rospy.Subscriber(joy,Joy,button_callback)
ts.registerCallback(callback)

rospy.spin()
print('\n\n-----------Ended ROS Data Coll-----------')
#bag.close()
print("\n\n-----------Recording Completed-----------")
