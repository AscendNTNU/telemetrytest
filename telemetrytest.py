#!/usr/bin/env python

'''
check bandwidth of link
'''
from __future__ import print_function
from cmath import pi
import time

from pymavlink import mavutil

# create a mavlink serial instance
master = mavutil.mavlink_connection("/dev/ttyUSB1", 57600) #Connection for the transmitter (Ground station)
slave = mavutil.mavlink_connection("/dev/ttyUSB0", 57600)  #Connection for the receiver (On the drone)

t1 = time.time()
t1_2 = time.time()

s_counts = {}
m_counts= {}
master_bytes_recv=0
master_bytes_sent=0
slave_bytes_sent=0
slave_bytes_recv=0

time_u=1642006285
x_p= 0.12920182800292968
y_p= -0.1958575439453125
z_p= 0.21002363586425782
roll= 0.21002363456425782
pitch= 0.210023635645425782
yaw= 0.21002363586445382


while True:
    #print(m_counts, s_counts)
    #send some messages to the target system with dummy data
    t2_1 =  time.time()
    if t2_1- t1_2 > 0.1:  #timelimit
        t1_2 = t2
        for i in range(9): #package per timelimit 
            master.mav.vision_position_estimate_send(time_u,x_p,y_p,z_p,roll,pitch,yaw)
        

    #Check for incoming data on the serial port and count
    #how many messages of each type have been received
    while master.port.inWaiting() > 0:
        #recv_msg will try parsing the serial port buffer
        #and return a new message if available
        m = master.recv_msg()

        if m is None: break  #No new message
        
        if m.get_type() not in m_counts:
        #if no messages of this type received, add this type to the counts dict
            m_counts[m.get_type()] = 0

        m_counts[m.get_type()] += 1

    while slave.port.inWaiting()>0:
        s = slave.recv_msg()

        if s is None: break #No new message

        if s.get_type() not in s_counts:
            s_counts[s.get_type()] = 0

        s_counts[s.get_type()] += 1

    #Print statistics every second
    t2 = time.time()
    if t2 - t1 > 1.0:
        
        print("%u Sent by master, %u Received by slave, Packets lost: Master: %u, Slave: %u " %(
            master.mav.total_packets_sent,
            slave.mav.total_packets_received,
            master.mav_loss,
            slave.mav_loss))
        t1 = t2
