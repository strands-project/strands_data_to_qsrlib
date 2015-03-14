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

from utilities.utilities import merge_world_qsr_traces
from qsrlib.qsrlib import QSRlib_Request_Message
from qsrlib_io.world_trace import Object_State, World_Trace
from qsrlib_ros.qsrlib_ros_client import QSRlib_ROS_Client

options = {"rcc3": "rcc3_rectangle_bounding_boxes_2d",
           "qtcb": "qtc_b_simplified",
           "qtcc": "qtc_c_simplified",
           "qtcbc": "qtc_bc_simplified",
           "rcc3a": "rcc3_rectangle_bounding_boxes_2d"}


def qsr_setup(data_dir, params, date):
    params_tag = map(str, params)
    params_tag = '__'.join(params_tag)
    qsr_tag = params_tag + date
    qsr_dir = os.path.join(data_dir, 'qsr_dump/')
    return qsr_dir, qsr_tag


def list_condition((item1, item2)):
    return item1 != item2



class Trajectory_Data_Reader(object):

    def __init__(self, objects=[], trajectories=[], config_filename="config.ini"):
        
        print("Initializing Data Reader...")
        self.list1 = objects
        self.list2 = trajectories

        self.spatial_relations = {}
        self.config = config_filename

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
            print("No object list provided. Pass reader to QSR_keeper class.")
        elif len(self.list2) == 0:
            print(" No second list provided.\n All pairwise relations in list 1 being generated...")
            pairwise_objects = {}
            for (obj1, obj2) in filter(list_condition, itertools.product(self.list1,    self.list1)):
                print(obj1, obj2)
        else:
            self.apply_qsr_lib()


    def apply_qsr_lib(self):
        objects = self.list1
        trajectories = self.list2
        print("params = ", self.params, '\n')
        self.which_qsr = options[self.params[0]]

        print("Number of qsrlib calls = ", len(trajectories)*len(objects), '\n')

        for uuid, poses in trajectories.items():
            world_traj_qsrs = []
            worlds = self.get_qsrlib_world(uuid, poses, objects)
            self.spatial_relations[uuid] = {}

            for (uuid, obj), world in worlds.items():
                qsrlib_request_message = QSRlib_Request_Message(which_qsr=self.which_qsr, \
                       input_data=world, include_missing_data=True)
                cln = QSRlib_ROS_Client()
                req = cln.make_ros_request_message(qsrlib_request_message)
                res = cln.request_qsrs(req)
                out = pickle.loads(res.data)
                world_traj_qsrs.append(out.qsrs)
            self.spatial_relations[uuid] = merge_world_qsr_traces(world_traj_qsrs)


    def get_qsrlib_world(self, uuid, t_poses, objects):
        o1 = []          #object 1 is always the trajectory
        o2_dic = {}      #object 2 is always the SOMA object
        worlds = {}
        (qsr, _q,v,n) =  self.params
        q = float('0.' + _q.split('_')[1])  #convert q from string to float

        for obj in objects:
            o2_dic[obj] = []
            key = (uuid, obj)
            worlds[key] = World_Trace()

        for frame, (x,y,z) in enumerate(t_poses):
            #print(frame, x, y)
            o1.append(Object_State(name="trajectory", timestamp = frame, x=x, y=y, \
                    quantisation_factor=q, validate=v, no_collapse=n))

            for obj in objects:      
                if len(objects[obj])==2:
                    (x,y) = objects[obj] 
                else:
                    (x,y,z) = objects[obj] 

                o2_dic[obj].append(Object_State(name=obj, timestamp = frame, x=x, y=y, \
                     quantisation_factor=q, validate=v, no_collapse=n))

        for obj, o2 in o2_dic.items():
            worlds[(uuid, obj)].add_object_state_series(o1)
            worlds[(uuid, obj)].add_object_state_series(o2)

        return worlds





class Trajectory_QSR_Keeper(object):
    def __init__(self, description="", objects=[], trajectories=[],
                reader=None, load_from_file="", dir=""):

        print("Initializing Data Keeper...")
        self.reader = reader
        self.list1 = objects
        self.list2 = trajectories
    
        if load_from_file is not None and load_from_file != "":
            self.load(dir, load_from_file)
        else:
            if type(self.reader) is not Trajectory_Data_Reader:
                raise TypeError("Provide a Trajectory_Data_Reader object")
            
            if len(self.list1) == 0:
                raise TypeError("No object list provided.")
            elif len(self.list2) == 0:
                raise TypeError("    No second list provided.\n    All pairwise relations in list 1 being generated...")
                pairwise_objects = {}
                for (obj1, obj2) in filter(list_condition, itertools.product(self.list1,    self.list1)):
                    print(obj1, obj2)
            else:
                self.reader = Trajectory_Data_Reader(self.list1, self.list2, reader.config)


    def save(self, path):
        print("Saving...")
        qsr_dir, tag = qsr_setup(path, self.reader.params, self.reader.date)
        filename  = os.path.join(qsr_dir, 'all_qsrs_' + tag + '.p')
        print(filename)

        foo = {"which_qsr": self.reader.params, \
              "world_qsr_traces":self.reader.spatial_relations}
        with open(filename, "wb") as f:
            pickle.dump(foo, f)
        print("success")


    def load(self, dir, filename):
        path  = os.path.join(dir, 'qsr_dump/' + filename)
        print("Loading QSRs from", path)
        
        with open(path, "rb") as f:
            foo = pickle.load(f)

        self.reader.params = foo["which_qsr"]
        self.reader.spatial_relations = foo["world_qsr_traces"]
        print("success")





class Episodes(object):

    def __init__(self, reader=None, load_from_file="", dir=""):
        
        self.reader=reader
        self.all_episodes = {}
        if load_from_file is not None and load_from_file != "":
            self.load(dir, load_from_file)


    def get_episodes(self, noise_thres=3, out=False):
        from data_processing_utils import  compute_episodes, filter_intervals

        cnt=0
        NOISE_THRESHOLD = noise_thres

        for (uuid, qsr_world_trace) in self.reader.spatial_relations.items():
            key, epi  = compute_episodes(uuid, qsr_world_trace)
            if out: print(cnt, "  ", key)
            # Filter the episods to remove very short transitions that are noise
            fepi = {}
            for e in epi:
                fepi[e] = filter_intervals(epi[e], NOISE_THRESHOLD)

            # Add filtered episodes to all_episodes
            self.all_episodes[key] = fepi
            cnt+=1
        

    def save(self, data_dir):
        print("Saving...")
        filename  = os.path.join(data_dir, 'episodes_dump/all_episodes.p')
        print(filename)

        with open(filename, "wb") as f:
            pickle.dump(self.all_episodes, f)
        print("success")


    def load(self, dir, filename):
        path  = os.path.join(dir, 'episodes_dump/' + filename)
        print("Loading Episodes from", path)
        with open(path, "rb") as f:
            self.all_episodes = pickle.load(f)
        print("success")




if __name__ == "__main__":
    rospy.init_node("trajectory_data_reader")

    #LOAD SOME DATA#
    data_dir='/home/strands/STRANDS/object_dump'
    obj_file  = os.path.join(data_dir, 'obj_dump.p')
    traj_file = os.path.join(data_dir, 'traj_dump.p')
    objects_in_roi = pickle.load(open(obj_file))
    trajectory_poses = pickle.load(open(traj_file))

    print(type(objects_in_roi))
    print(type(trajectory_poses))

    #Options File:
    config_path = '/home/strands/STRANDS/config.ini'
   
    #Create data_reader class - needs config file.
    reader = Trajectory_Data_Reader(config_filename = config_path)

    #data_reader can be ran in stand-alone:
    reader = Trajectory_Data_Reader(objects_in_roi, trajectory_poses, config_path)    

    #Or create data_keeper class
    #keeper = Trajectory_QSR_Keeper(objects=objects_in_roi, \
    #                trajectories=trajectory_poses, reader=reader)

    #Data_keeper can be used to save and load from pickle files
    #keeper.save(data_dir)
    #print(len(keeper.reader.spatial_relations))

    #test_load = 'all_qsrs_qtcb__0_01__False__True__03_03_2015.p'
    #keeper.load(data_dir, test_load)
    #print(len(keeper.reader.spatial_relations))

    ##More Tests:
    #keeper= Trajectory_QSR_Keeper(reader=reader, load_from_file = test_load, dir=data_dir) 

    #Create Episodes from QSRs:
    #ep = Episodes(reader=keeper.reader)
    #ep.get_episodes(noise_thres=3)
    #ep.save(data_dir)

    #Test Load:
    ep_test = Episodes(keeper.reader, 'all_episodes.p', data_dir)
    #print(ep_test.all_episodes.keys())
    

    print(ep_test.all_episodes['d6c54902-3259-5ff4-b1ca-9ed5132df53d__1__103'].keys())

    for i in ep_test.all_episodes['d6c54902-3259-5ff4-b1ca-9ed5132df53d__1__103']:
        print(ep_test.all_episodes['d6c54902-3259-5ff4-b1ca-9ed5132df53d__1__103'][i])


    #TODO: Input Two lists. 
       #When two lists are passed, products of the two lists are computed
       #When only one is passed, all pairwise combinations are computed

    """#prints one trajectories data
    for uuid, data in reader.spatial_relations.items():
        print(type(data))
        print(type(data.trace))
        print(uuid)
        for timepoint in data.trace:
            print(timepoint)
            for key in data.trace[timepoint].qsrs:
                print(key)
                print(data.trace[timepoint].qsrs[key].qsr)


        sys.exit(1)
    """





