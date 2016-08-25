#!/usr/bin/env python

"""uwb_tracker_node.py: Streams tracked positions based on UWB multi-range messages."""

__author__      = "Manuel Stalder"
__email__ = "mastalde@ethz.ch"
__copyright__   = "Copyright 2016 Manuel Stalder"

import select
import sys
import numpy as np
import serial
import roslib
import scipy.stats
roslib.load_manifest('uwb')
import rospy
import tf

import uwb.msg


class PlotData(object):

    def __init__(self, plot, max_data_length=None):
        self.plot = plot
        self.curves = []
        self.data = []
        self.max_data_length = max_data_length

    def add_curve(self, pen, initial_data=None, **kwargs):
        self.curves.append(self.plot.plot(pen=pen, **kwargs))
        if initial_data is None:
            if self.max_data_length is None:
                initial_data = []
            else:
                initial_data = np.zeros((self.max_data_length,))
        self.data.append(initial_data)

    def add_point(self, index, value):
        assert(index < len(self.curves))
        if self.max_data_length is None:
            self.data[index].append(value)
        else:
            self.data[index][:-1] = self.data[index][1:]
            self.data[index][-1] = value
            if len(self.data[index]) > self.max_data_length:
                self.data[index] = self.data[index][-self.max_data_length:len(self.data[index])]
        self.curves[index].setData(self.data[index])

    def get_plot(self):
        return self.plot

    def __len__(self):
        return len(self.curves)

class TrackerBagHandler(object):
    
    def __init__(self):
        self.dummy = 0
        
    def handle_tracker_message():
        

def main():

    rospy.init_node('uwb_tracker_bag_parser')

    show_plots = rospy.get_param('~show_plots', False)
    uwb_tracker_topic = rospy.get_param('~tracker_topic', '/uwb/tracker')
    
    rospy.Subscriber(uwb_tracker_topic, uwb.msg.UWBTracker, handle_tracker_message)



  
    rospy.spin()



if __name__ == '__main__':
    main()
