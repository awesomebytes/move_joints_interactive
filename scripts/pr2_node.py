#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 30/11/14

@author: Sam Pfeiffer

"""
# System imports
import random
import threading
import copy
import subprocess

# ROS imports
import rospy
from interactive_markers.interactive_marker_server import *
# messages
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.msg import ControllerState
# services
#from controller_manager_msgs.srv import ListControllers, ListControllersRequest

# services
from pr2_mechanism_msgs.srv import ListControllers, ListControllersRequest#, ListControllersResponse
from control_msgs.srv import QueryTrajectoryState, QueryTrajectoryStateRequest, QueryTrajectoryStateResponse


# local imports
# Borrowed from Adolfo Rodriguez Tsouroukdissian
# at https://github.com/ros-controls/ros_controllers/blob/indigo-devel/rqt_joint_trajectory_controller/src/rqt_joint_trajectory_controller/joint_limits_urdf.py
# as the urdf_dom_py interface gives errors
from joint_limits_urdf import get_joint_limits
from im_for_link_class import LinkInteractiveMarker

CONTROLLER_MNGR_SRV = "/pr2_controller_manager/list_controllers"

def get_joints_of_controller(controller_name):
    query_state_srv = "/" + controller_name + "/query_state"
    rospy.loginfo("Getting joints from: " + query_state_srv)
    query_srv = rospy.ServiceProxy(query_state_srv, QueryTrajectoryState)
    query_srv.wait_for_service()
    req = QueryTrajectoryStateRequest()
    req.time = rospy.Time.now() + rospy.Duration(999.9) # must be very in the future for some reason
    resp = query_srv.call(req)
    #resp = QueryTrajectoryStateResponse()
    joint_names = resp.name
    return joint_names


class InteractiveJointTrajCtrl():
    def __init__(self):
        rospy.loginfo("Initializing InteractiveJointTrajCtrl")
        ctl_mngr_srv = rospy.ServiceProxy(CONTROLLER_MNGR_SRV, ListControllers)
        rospy.loginfo("Connecting to " + CONTROLLER_MNGR_SRV)
        ctl_mngr_srv.wait_for_service()
        rospy.loginfo("Connected.")
        
        self.rviz_config_str = ""
        
        req = ListControllersRequest()
        resp = ctl_mngr_srv.call(req)
        #ListControllersResponse()
        ctl_mngr_srv.close()
        joints_dict = get_joint_limits()
        publishers_dict = {}
        # dictionary that contains
        # key: head_controller
        # value:
        #   joints: ['head_1_joint', 'head_2_joint']
        #   publisher: pub
        #   ims: [interactive_marker_server_head_1_joint, interactive_marker_server_head_2_joint]
        
        for cs in resp.controllers: # For every controller, create a random spammer
            # cs is a string with the name of the controller, now check if has command interface
            #published_topics = rostopic.rospy.get_published_topics() #for some reason is not the same than
            # doing the bash call
            ps = subprocess.Popen(["rostopic", "list" ], stdout=subprocess.PIPE)
            controller_related_topics = subprocess.check_output(('grep', cs), stdin=ps.stdout) #, stdout=subprocess.PIPE)
            #print "controller_related_topics to " + cs + ": " + str(controller_related_topics)
            for topic_name in controller_related_topics.split():
                #print "topic_name: " + topic_name
                if "command" in topic_name:
                    # We need to check if there is a query state service to ask him
                    # what joints does this controller control
                    ps2 = subprocess.Popen(["rosservice", "list"], stdout=subprocess.PIPE)
                    with_query_controllers = subprocess.check_output(('grep', 'query_state'), stdin=ps2.stdout)
                    #print "services with query_state: " + str(with_query_controllers)
                    joint_names = []
                    for srv_name in with_query_controllers.split():
                        #print "service: " + srv_name
                        if cs in srv_name:
                            print "Controller '" + cs + "' has topic /command interface and service /query_state interface!"
                            joint_names = get_joints_of_controller(cs)
                    if not joint_names:
                        continue
                    else:
                        publishers_dict[cs] = {}
                        publishers_dict[cs]['joints'] = joint_names
                        cmd_topic = "/" + cs + "/command"
                        publishers_dict[cs]['pub'] = rospy.Publisher(cmd_topic, JointTrajectory)
                        publishers_dict[cs]['ims'] = []
                        for joint_name in publishers_dict[cs]['joints']:
                            publishers_dict[cs]['ims'].append( LinkInteractiveMarker(joint_name, cs, joint_names))
                            curr_im_rviz_cfg = self.create_im_config_rviz_block(joint_name)
                            self.rviz_config_str += curr_im_rviz_cfg
        
        self.save_rviz_config(self.rviz_config_str)
        
    
    def create_im_config_rviz_block(self, joint_name):
        ims_str = "ims_" + joint_name.replace('_joint', '')
        template = """        - Class: rviz/InteractiveMarkers
          Enable Transparency: true
          Enabled: true
          Name: """ + ims_str + """
          Show Axes: false
          Show Descriptions: false
          Show Visual Aids: false
          Update Topic: /""" + ims_str + """/update
          Value: true\n"""
        return template
    
    def save_rviz_config(self, rviz_cfg):
        import rospkg
        rp = rospkg.RosPack()
        path = rp.get_path("move_joints_interactive")
        rviztemplate = open(path + "/rviz/template_config.rviz", "r")
        full_file = rviztemplate.read()
        rviztemplate.close()
        #full_file =""
        final_file = full_file.replace("TEMPLATE_TO_FILL", rviz_cfg)
        output_f = open(path + "/rviz/current_rviz.rviz", "w")
        output_f.write(final_file)
        output_f.close()
        rospy.loginfo("\nRviz configuration file created at: " + path + "/rviz/current_rviz.rviz")
    
    
    def run(self):
        rospy.loginfo("Running!")
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        
if __name__ == '__main__':
    rospy.init_node('InteractiveJointTrajCtrl_')
    
    node = InteractiveJointTrajCtrl()
    node.run()
    
    