#!/usr/bin/env python

__author__      = "Manuel Stalder"
__email__       = "masstalde@gmail.com"
__copyright__   = "Copyright 2016 Manuel Stalder"

import select
import numpy as np
import roslib
roslib.load_manifest('uwb')
import rospy

from numpy.linalg import inv


import uwb.msg
import std_msgs


class AttitudeEst(object):

    class Agent(object):
        
        def __init__(self, agent_number):
            self.agent_number = agent_number;
            self.vectors_updated = np.array([0, 0])
            vector_to_previous = np.array([0.0, 0.0])
            vector_to_next = np.array([0.0, 0.0])
            self.vectors = np.array([vector_to_previous, vector_to_next])
            self.master_module_offset = np.array([0.0, 0.0])
        

    def __init__(self):
        self._read_configuration()
        self.agent_topics = []
        self.agent_list = []
        self.all_agents_init = 0
        self.R21 = np.array([[1, 0], [0, 1]])
        self.R31 = np.array([[1, 0], [0, 1]])
        self.R12 = inv(self.R21)
        self.R13 = inv(self.R31)
        
        
        for i in range(1, self.number_of_agents + 1):
            agent = self.Agent(i)
            
            #TEST
            agent.master_module_offset = np.array([-0.2, 0.2])
            
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
        
        agent = self.agent_list[agent_number - 1]
        
        #Correct the vector to point to the COG
        #First, rotate into inertial frame
        R = np.array([[1, 0], [0, 1]])
        if remote_address == 3:
            R = self.R31.dot(R)
        elif remote_address == 2:
            R = self.R21.dot(R)
        #If required, rotate to respective body frame
        if agent_number == 2:
            R = self.R12.dot(R)
        elif agent_number == 3:
            R = self.R13.dot(R)
        
        agent_state -= R.dot(agent.master_module_offset)
        
        rospy.loginfo('Corrected Vector r{}{}'.format(str(agent_number), str(remote_address)))
        rospy.loginfo(agent_state)
                
        if remote_address == (agent_number%self.number_of_agents + 1):
            agent.vectors[1] = agent_state[0:2]
            agent.vectors_updated[1] = 1
        if (remote_address%self.number_of_agents) == ((agent_number - 1)%self.number_of_agents):
            agent.vectors[0] = agent_state[0:2]
            agent.vectors_updated[0] = 1
            
        
        
        if self.all_agents_init:
            if agent.vectors_updated.all():
                self.perform_kabsch()
                agent.vectors_updated = np.array([0, 0])
        else:
            self.all_agents_init = 1
            for ag in self.agent_list:
                if not ag.vectors[0].any() or not ag.vectors[1].any():
                    self.all_agents_init = 0
       
               
    
    def perform_kabsch(self):
         rospy.loginfo("Performing Kabsch")
         
         #Preparing Matrices
         P = np.array([[0, 0], self.agent_list[0].vectors[1], self.agent_list[0].vectors[0]])
         Q21 =  np.array([self.agent_list[1].vectors[0], [0, 0], self.agent_list[1].vectors[1]])
         Q31 =  np.array([self.agent_list[2].vectors[1], self.agent_list[2].vectors[0], [0, 0]])
         
         #Calculate centroids
         P_c = self.centroid(P)
         Q21_c = self.centroid(Q21)
         Q31_c = self.centroid(Q31)
         
         P -= P_c
         Q21 -= Q21_c
         Q31 -= Q31_c
         
         self.R21 = self.kabsch(P, Q21)
         self.R31 = self.kabsch(P, Q31)
         
         self.R12 = inv(self.R21)
         self.R13 = inv(self.R31)
           

         self.publish_rotated_vectors()  
          
         
    def publish_rotated_vectors(self):
        z1 = (self.agent_list[0].vectors[1] - self.R21.dot(self.agent_list[1].vectors[0])) / 2
        z2 = (self.R21.dot(self.agent_list[1].vectors[1]) - self.R31.dot(self.agent_list[2].vectors[0])) / 2 
        z3 = (self.agent_list[0].vectors[0] - self.R31.dot(self.agent_list[2].vectors[1])) / 2
        

        
        rospy.loginfo(z1)
        rospy.loginfo(z2)
        rospy.loginfo(z3)
        
     
    def centroid(self, X):
        """
        Calculate the centroid from a vectorset X
        """
        C = sum(X)/len(X)
        return C
        
    def kabsch(self, P, Q):
        """
        The optimal rotation matrix U is calculated and then used to rotate matrix
        P unto matrix Q so the minimum root-mean-square deviation (RMSD) can be
        calculated.
        Using the Kabsch algorithm with two sets of paired point P and Q,
        centered around the center-of-mass.
        Each vector set is represented as an NxD matrix, where D is the
        the dimension of the space.
        The algorithm works in three steps:
        - a translation of P and Q
        - the computation of a covariance matrix C
        - computation of the optimal rotation matrix U
        http://en.wikipedia.org/wiki/Kabsch_algorithm
        Parameters:
        P -- (N, number of points)x(D, dimension) matrix
        Q -- (N, number of points)x(D, dimension) matrix
        Returns:
        U -- Rotation matrix
        """

        # Computation of the covariance matrix
        C = np.dot(np.transpose(P), Q)

        # Computation of the optimal rotation matrix
        # This can be done using singular value decomposition (SVD)
        # Getting the sign of the det(V)*(W) to decide
        # whether we need to correct our rotation matrix to ensure a
        # right-handed coordinate system.
        # And finally calculating the optimal rotation matrix U
        # see http://en.wikipedia.org/wiki/Kabsch_algorithm
        V, S, W = np.linalg.svd(C)
        d = (np.linalg.det(V) * np.linalg.det(W)) < 0.0

        if d:
            S[-1] = -S[-1]
            V[:, -1] = -V[:, -1]

        # Create Rotation matrix U
        U = np.dot(V, W)

        return U
         
         
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
