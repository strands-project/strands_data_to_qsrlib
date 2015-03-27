#!/usr/bin/env python

#__author__      = "Paul Duckworth"
#__copyright__   = "Copyright 2015, University of Leeds"
from __future__ import print_function
import rospy
import time
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
    if isinstance(params[1], float): 
        temp = str(params[1]).replace(".", "_")
        params = (params[0], temp, params[2], params[3])
    #print(params)
    params_tag = map(str, params)
    params_tag = '__'.join(params_tag)
    qsr_tag = params_tag + date
    qsr_dir = os.path.join(data_dir, 'qsr_dump/')
    return qsr_dir, qsr_tag


def list_condition((item1, item2)):
    return item1 != item2



class Trajectory_Data_Reader(object):
    '''
    Trajectory Data Reader class:
    To be used to pass data into QSRLib, and return qsrs.

    Three different methods of use:
    1. Pass objects dictionary only - will return all pairwise object relations.
    2. Pass objects and trajectory dictionaries - will return each trajectory with each object
    3. Pass objects and trajectory dictionaries along with a mapping between the two - will return only these pairs.
    '''
    def __init__(self, objects=[], trajectories={}, config_filename="config.ini",
                    roi="", objs_to_traj_map = {}, load_from_file="", dir=""):
        

        if load_from_file is not None and load_from_file != "":
            self.load(filename=load_from_file, dir = dir)
            return

        print("Initializing Data Reader...")
        t0 = time.time()
        self.dict1 = objects
        self.dict2 = trajectories
        self.obs_to_traj = objs_to_traj_map

        self.spatial_relations = {}
        self.config = config_filename
        self.roi = roi

        config_parser = ConfigParser.SafeConfigParser()
        print(config_parser.read(config_filename))

        if len(config_parser.read(config_filename)) == 0:
            raise ValueError("Config file not found, please provide a config.ini file as described in the documentation")
        config_section = "trajectory_data_reader"
        try:
            self.date = config_parser.get(config_section, "date")
            qsr = config_parser.get(config_section, "qsr")
        except ConfigParser.NoOptionError:
            raise
        if qsr == "qtcb": 
            print("QTCb requested")
            try:
                q = config_parser.getfloat(config_section, "q")
                v = config_parser.getboolean(config_section, "v")
                n = config_parser.getboolean(config_section, "n")
                self.params = (qsr, q, v, n)
            except ConfigParser.NoOptionError:
                raise

        #If no input data - return
        if len(self.dict1) == 0:
            raise ValueError("No object list provided.")

        #If only one list provided - generate all pairwise relations
        elif len(self.dict2) == 0:
            print("    No second list provided.\n All pairwise relations in object list being generated...")
            flg=1
            self.pairs=[]
            for (obj1, obj2) in filter(list_condition, itertools.product(self.dict1, self.dict1)):
                self.pairs.append((obj1, obj2))
            
        elif self.obs_to_traj != {}: flg=2
        else: flg=3

        if flg==2: print("    Mapping provided. Pairs being generated...")
        if flg==3: print("    All pairs between trajectories and objects being generated...")

        self.apply_qsr_lib(flg)
        t = time.time()-t0
        print("Done. Took %f seconds." % t)

    def apply_qsr_lib(self, flg=1):

        print("    params = ", self.params, '\n')
        self.which_qsr = options[self.params[0]]

        ##For each trajcetory:
        for uuid, poses in self.dict2.items():

            #print(uuid, "  # Poses = ", len(poses))
            if self.obs_to_traj != {}:
                objects={}
                for i in self.obs_to_traj[uuid]:
                    objects[i] = self.dict1[i]
            else: objects = self.dict1  #Else, use all objects for each trajectory

            world_traj_qsrs = []
            worlds = self.get_qsrlib_world(uuid, poses, objects)
            self.spatial_relations[uuid] = {}

            ##for each object to pair up with the trajectory:            
            for (uuid, obj), world in worlds.items():
                #print("world.trace keys = ", world.trace.keys())
                qsrlib_request_message = QSRlib_Request_Message(which_qsr=self.which_qsr, \
                       input_data=world, include_missing_data=True)
                cln = QSRlib_ROS_Client()
                req = cln.make_ros_request_message(qsrlib_request_message)
                res = cln.request_qsrs(req)
                out = pickle.loads(res.data)
                world_traj_qsrs.append(out.qsrs)

                #for t in out.qsrs.get_sorted_timestamps():
                #    foo = str(t) + ": "
                #    for k, v in zip(out.qsrs.trace[t].qsrs.keys(), out.qsrs.trace[t].qsrs.values()):
                #        foo += str(k) + ":" + str(v.qsr) + "; "
                #    print(foo)
                #print("out.qsr.trace length ", len(out.qsrs.trace), "\n")

                self.spatial_relations[uuid] = merge_world_qsr_traces(world_traj_qsrs)
        return


    def get_qsrlib_world(self, uuid, t_poses, objects):
        o1 = []          #object 1 is always the trajectory
        o2_dic = {}      #object 2 is always the SOMA object
        worlds = {}
        (qsr, q,v,n) =  self.params

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



    def save(self, path):
        print("Saving...")

        roi = self.roi
        qsr_dir, tag = qsr_setup(path, self.params, self.date)
        filename  = os.path.join(qsr_dir, roi + '_qsrs_' + tag + '.p')
        print(filename)

        foo = {"ROI": roi,
               "which_qsr": self.params, \
               "world_qsr_traces":self.spatial_relations}
        with open(filename, "wb") as f:
            pickle.dump(foo, f)
        print("success")


    def load(self, filename, dir=""):
        if dir == "": path = filename
        else: path  = os.path.join(dir, 'qsr_dump/' + filename)

        print("Loading QSRs from", path)    
        with open(path, "rb") as f:
            foo = pickle.load(f)
        self.roi = foo["ROI"]
        self.params = foo["which_qsr"]
        self.spatial_relations = foo["world_qsr_traces"]
        print("success")



class Episodes(object):

    def __init__(self, reader=None, noise = 3, load_from_file="", dir=""):
        t0 = time.time()
        self.reader=reader
        self.all_episodes = {}
        if load_from_file is not None and load_from_file != "":
            self.load(dir, load_from_file)
            return
        else:
            print("Generating Episodes...")
            self.get_episodes(noise_thres=noise)
            t = time.time()-t0
            print("Done. Took %f seconds." % t)
    
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
        filename  = os.path.join(data_dir, 'episodes_dump/' + self.reader.roi \
            + '_episodes.p')
        print(filename)
        foo = {"ROI": self.reader.roi,
               "episodes": self.all_episodes}
        with open(filename, "wb") as f:
            pickle.dump(foo, f)
        print("success")


    def load(self, dir, filename):
        path  = os.path.join(dir, 'episodes_dump/' + filename)
        print("Loading Episodes from", path)
        with open(path, "rb") as f:
            foo = pickle.load(f)
        self.reader.roi = foo["ROI"]
        self.all_episodes = foo["episodes"]
        print("success")




if __name__ == "__main__":
    rospy.init_node("trajectory_data_reader")

    ##LOAD SOME DATA#
    base_dir='/home/strands/STRANDS/'
    data_dir='/home/strands/STRANDS/object_dump'
    obj_file  = os.path.join(data_dir, 'obj_dump.p')
    traj_file = os.path.join(data_dir, 'traj_dump.p')
    objects_in_roi = pickle.load(open(obj_file))
    trajectory_poses = pickle.load(open(traj_file))

    print(type(objects_in_roi), "of objects")
    print(type(trajectory_poses), "of trajectory poses")

    ##Options File:
    config_path='/home/strands/catkin_ws/src/trajectory_behaviours/relational_learner/config.ini'
   
    #qsr_reader = Trajectory_Data_Reader(objects=objects_in_roi, \
    #                            trajectories=trajectory_poses, \
    #                            config_filename=config_path, \
    #                            roi=str(12))
    #qsr_reader.save(data_dir)
    
    test_load = '20_qsrs_qtcb__0_01__False__True__03_03_2015.p'
    qsr_reader = Trajectory_Data_Reader(load_from_file = test_load, dir = base_dir)
    print(len(qsr_reader.spatial_relations))
    print(qsr_reader.params)

    ##Create Episodes from QSRs:
    ep = Episodes(reader=qsr_reader, noise=3)
    key = ep.all_episodes.keys()[0]
    print(ep.all_episodes[key])

    ep.save(base_dir)

    #Test Load:
    #ep_test = Episodes(keeper.reader, 'all_episodes.p', data_dir)
    #print(ep_test.all_episodes.keys())
    

    #print(ep_test.all_episodes['d6c54902-3259-5ff4-b1ca-9ed5132df53d__1__103'].keys())

    #for i in ep_test.all_episodes['d6c54902-3259-5ff4-b1ca-9ed5132df53d__1__103']:
    #    print(ep_test.all_episodes['d6c54902-3259-5ff4-b1ca-9ed5132df53d__1__103'][i])





