#!/usr/bin/env python

__author__      = "Manuel Stalder"
__email__       = "masstalde@gmail.com"
__copyright__   = "Copyright 2016 Manuel Stalder"

import select
import numpy as np
import roslib
roslib.load_manifest('uwb')
import rospy


import uwb.msg
import std_msgs


class AttitudeEst(object):

    class Agent(object):
        
        def __init__(self, agent_number):
            self.agent_number = agent_number;
            vector_to_previous = [0.0, 0.0]
            vector_to_next = [0.0, 0.0]
            self.vectors = [vector_to_previous, vector_to_next]
            self.master_module_offset = [0.0, 0.0]
        

    def __init__(self):
        self._read_configuration()
        self.agent_topics = []
        self.agent_list = []
        self.all_agents_init = 0
        
        
        for i in range(1, self.number_of_agents + 1):
            agent = self.Agent(i)
            self.agent_list.append(agent)   
            # ROS publishers
            agent_i_topic = self.agents_topic_templ.replace("*", str(i))
            agent_i_pub = rospy.Publisher(agent_i_topic, std_msgs.msg.Float32MultiArray, queue_size=3)
            self.agent_topics.append(agent_i_pub)
                 
        self.tracker_sub = rospy.Subscriber(self.uwb_tracker_topic, uwb.msg.UWBTracker, self.handle_tracker_message)
        
        
    def _read_configuration(self):
        """Initialize configuration from ROS parameters."""        
        self.number_of_agents = rospy.get_param('~number_of_agents', 3)
        self.uwb_tracker_topic = rospy.get_param('~tracker_topic', '/uwb/tracker')
        self.agents_topic_templ = rospy.get_param('~agents_topic_templ', '/z*vec')


    def handle_tracker_message(self, tracker_msg):
        agent_number = tracker_msg.address
        remote_address = tracker_msg.remote_address
        agent_state = tracker_msg.state
        
        agent = self.agent_list.pop(agent_number - 1)
        
        if remote_address == (agent_number%self.number_of_agents + 1):
            agent.vectors[1] = agent_state[0:2]
        if (remote_address%self.number_of_agents) == ((agent_number - 1)%self.number_of_agents):
            agent.vectors[0] = agent_state[0:2]
            
        self.agent_list.insert(agent_number-1, agent)   
        
        if self.all_agents_init:
            self.perform_kabsch()
        else:
            self.all_agents_init = 1
            for ag in self.agent_list:
                if ag.vectors[0] == [0, 0] or ag.vectors[1] == [0, 0]:
                    self.all_agents_init = 0
       
               
    
    def perform_kabsch(self):
         rospy.loginfo("Performing Kabsch")
    
    def exec_(self):
        rospy.spin()


    def stop(self):
        rospy.signal_shutdown('User request')
        
        
def main():
    import signal

    rospy.init_node('awarm_attitude_node')
    est = AttitudeEst()

    def sigint_handler(sig, _):
        if sig == signal.SIGINT:
            est.stop()
    signal.signal(signal.SIGINT, sigint_handler)

    try:
        est.exec_()
    except (rospy.ROSInterruptException, select.error):
        rospy.logwarn("Interrupted... Stopping.")


if __name__ == '__main__':
    main()
