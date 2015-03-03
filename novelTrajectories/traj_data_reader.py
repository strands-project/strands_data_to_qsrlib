#!/usr/bin/env python

#__author__      = "Paul Duckworth"
#__copyright__   = "Copyright 2015, University of Leeds"
from __future__ import print_function
import rospy
import timeit
import ConfigParser
try:
    import cPickle as pickle
except ImportError:
    import pickle
import csv
import argparse
import sys
import os
import itertools
import numpy as np
from utilities import merge_world_qsr_traces

from qsrlib.qsrlib import QSRlib_Request_Message
from qsrlib_io.world_trace import Object_State, World_Trace
from qsrlib_ros.qsrlib_ros_client import QSRlib_ROS_Client


options = {"rcc3": "rcc3_rectangle_bounding_boxes_2d",
           "qtcb": "qtc_b_simplified",
           "qtcc": "qtc_c_simplified",
           "qtcbc": "qtc_bc_simplified",
           "rcc3a": "rcc3_rectangle_bounding_boxes_2d"}


def list_condition((item1, item2)):
    return item1 != item2

class Trajectory_Data_Reader(object):

    def __init__(self, objects=[], trajectories=[], config_filename="config.ini"):

        print("Initializing...")
        self.list1 = objects
        self.list2 = trajectories
        self.spatial_relations = {}

        config_parser = ConfigParser.SafeConfigParser()
        print(config_parser.read(config_filename))
        if len(config_parser.read(config_filename)) == 0:
            raise ValueError("Config file not found, please provide a config.ini file as described in the documentation")
        config_section = "trajectory_data_reader"
        try:
            self.date = config_parser.get(config_section, "date")
            qsr = config_parser.get(config_section, "qsr")
            q = config_parser.get(config_section, "q")
            v = config_parser.get(config_section, "v")
            n = config_parser.get(config_section, "n")
            self.params = (qsr, q, v, n)
        except ConfigParser.NoOptionError:
            raise


        if len(self.list1) == 0:
            print("No object list provided")
        elif len(self.list2) == 0:
            print("    No second list provided.\n    All pairwise relations in list 1 being generated...")
            pairwise_objects = {}
            for (obj1, obj2) in filter(list_condition, itertools.product(self.list1, self.list1)):
                print(obj1, obj2)
        else:
            self.apply_qsr_lib()
   

    def apply_qsr_lib(self):
      
        objects = self.list1
        trajectories = self.list2
        print(self.params)
        which_qsr = options[self.params[0]]

        for uuid, poses in trajectories.items():
            world_traj_qsrs = []
            worlds = self.get_qsrlib_world(uuid, poses, objects)
            self.spatial_relations[uuid] = {}

            for (uuid, obj), world in worlds.items():
                qsrlib_request_message = QSRlib_Request_Message(which_qsr=which_qsr, \
                       input_data=world, include_missing_data=True)
                cln = QSRlib_ROS_Client()
                req = cln.make_ros_request_message(qsrlib_request_message)
                res = cln.request_qsrs(req)
                out = pickle.loads(res.data)
                world_traj_qsrs.append(out.qsrs)
            self.spatial_relations[uuid] = merge_world_qsr_traces(world_traj_qsrs)

                # for t in out.qsrs.get_sorted_timestamps():
                #     if t not in self.spatial_relations[uuid]:
                #         self.spatial_relations[uuid][t] = {}
                #
                #     relations = str(out.qsrs.trace[t].qsrs.values()[0].qsr)
                #     self.spatial_relations[uuid][t][(uuid, obj)] = relations




    def get_qsrlib_world(self, uuid, t_poses, objects):
        (qsr, _q,v,n) =  self.params
        q = float('0.' + _q.split('_')[1])  #convert q from string to float

        o1 = []          #object 1 is always the trajectory
        o2_dic = {}      #object 2 is always the SOMA object
        worlds = {}

        for obj in objects:
            o2_dic[obj] = []
            key = (uuid, obj)
            worlds[key] = World_Trace()

        for frame, (x,y,z) in enumerate(t_poses):
            #print(frame, x, y)
            o1.append(Object_State(name="trajectory", timestamp = frame, x=x, y=y, \
                    quantisation_factor=q, validate=v, no_collapse=n))

            for obj in objects:            
                (x,y) = objects[obj]
                o2_dic[obj].append(Object_State(name=obj, timestamp = frame, x=x, y=y, \
                    quantisation_factor=q, validate=v, no_collapse=n))

        for obj, o2 in o2_dic.items():
            worlds[(uuid, obj)].add_object_state_series(o1)
            worlds[(uuid, obj)].add_object_state_series(o2)

        #print(worlds[('7d638405-b2f8-55ce-b593-efa8e3f2ff2e', u'Printer (photocopier)_2')])
        return worlds




if __name__ == "__main__":
    rospy.init_node("trajectory_data_reader")

    #LOAD SOME DATA#
    data_dir='/home/strands/STRANDS'
    obj_file  = os.path.join(data_dir, 'obj_dump.p')
    traj_file = os.path.join(data_dir, 'traj_dump.p')
    objects_in_roi = pickle.load(open(obj_file))
    trajectory_poses = pickle.load(open(traj_file))

    print(type(objects_in_roi))
    print(type(trajectory_poses))

    #Options File:
    config_path = '/home/strands/STRANDS/config.ini'

    #TODO: Input Two lists. 
       #When two lists are passed, products of the two lists are computed
       #When only one is passed, all pairwise combinations are computed
    reader = Trajectory_Data_Reader(objects_in_roi, trajectory_poses, config_path)

    print(reader.spatial_relations['93a66b0e-a9ae-50b8-89a4-be112db759f8'].trace[1].qsrs['Printer (photocopier)_5,trajectory'].qsr)

    #for (uuid, i) in reader.spatial_relations.items():
    #    print(uuid)
    #    print(i)
        #print(i[uuid][0].qsrs.keys())
        #print(key)
        #if frame == 5: sys.exit(1)
    #reader = Trajectory_Data_Reader(objects_in_roi)




