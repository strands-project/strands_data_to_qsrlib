#!/usr/bin/env python

"""data_processing_utils.py"""

__author__      = "Paul Duckworth"
__copyright__   = "Copyright 2015, University of Leeds"

import os, sys
import pickle
import numpy as np

import qsrlib_io.world_trace

#**************************************************************#
#     Compute Episode Representation of the Spatial Relations  #
#**************************************************************#
#print keeper.reader.spatial_relations['7d638405-b2f8-55ce-b593-efa8e3f2ff2e'].trace[1].qsrs['Printer (photocopier)_5,trajectory'].qsr

def compute_episodes(uuid, spatial_relations):
    episodes  = {}
    obj_relation_data = {}
    #print(type(spatial_relations))

    if isinstance(spatial_relations, qsrlib_io.world_qsr_trace.World_QSR_Trace):
        frames = spatial_relations.trace.keys()
        frames.sort()
        for frame in frames:
            for obj_pair in spatial_relations.trace[frame].qsrs:
                if obj_pair not in obj_relation_data:
                    obj_relation_data[obj_pair] = []
                qsr = "%s" % spatial_relations.trace[frame].qsrs[obj_pair].qsr  #qsr needs to be a string
                obj_relation_data[obj_pair].append((frame, qsr))
                
    elif type(spatial_relations) == dict:
        frames = spatial_relations.keys()
        frames.sort()
        for frame in frames:
            for obj_pair in spatial_relations[frame]:
                if obj_pair not in obj_relation_data:
                    obj_relation_data[obj_pair] = []
                obj_relation_data[obj_pair].append((frame, spatial_relations[frame][obj_pair]))


    for obj_pair in obj_relation_data:
        obj_0_id = uuid
        obj_0_type = 'traj'

        obj = obj_pair.replace(',trajectory', ' ')
        obj_1_id = obj
        obj_1_type = obj.split('_')[0]

        episodes[(obj_0_id, obj_0_type, obj_1_id, obj_1_type)] = []
        epi_start = obj_relation_data[obj_pair][0][0]
        epi_end   = obj_relation_data[obj_pair][0][0]
        epi_value = '%s' % obj_relation_data[obj_pair][0][1]

        for (frame, relation) in obj_relation_data[obj_pair]:
            rel = '%s' % relation
            epi_value_str = '%s' % epi_value

            if rel == epi_value_str:
                epi_end = frame
            else:
                episodes[(obj_0_id, obj_0_type, obj_1_id, obj_1_type)].append((obj_0_id, obj_0_type,\
                                                                               obj_1_id, obj_1_type,\
                                                                               epi_value, epi_start, epi_end))
                epi_start = epi_end = frame
                epi_value = relation
        else:
            episodes[(obj_0_id, obj_0_type, obj_1_id, obj_1_type)].append((obj_0_id, obj_0_type,\
                                                                           obj_1_id, obj_1_type,\
                                                                           epi_value, epi_start, epi_end))

    key = '__'.join([uuid, repr(min(frames)), repr(max(frames))])
    return key, episodes    



def filter_intervals(intv_list_dup, noise_threshold):
    intv_list = intv_list_dup[:]
    filtered_intv_list = []
    next_relation_threshold = noise_threshold
    
    newf = None
    obj1_id   = 0
    obj1_type = 1
    obj2_id   = 2
    obj2_type = 3
    rel       = 4
    start     = 5
    e         = 6
    for f in intv_list:
        #PD: This doesn't check whether the first f is below the threshold value
        if newf is None:
            newf = list(f[:])
        elif str(newf[rel]) == str(f[rel]):
            # both intervals have same relations, just merge
            newf[e] = f[e]
        else:
            # intervals have different relations
            if f[e] - f[start] < next_relation_threshold: 
                # interval of relation is too small, so merge with previous one
                newf[e] = f[e]
            else:
                # This interval is reasonably big, so don't merge it.
                filtered_intv_list.append(tuple(newf))
                newf = list(f[:])
    else:            
        # Reached the end of list. so finish by adding the last interval (newf not f)
        filtered_intv_list.append(tuple(newf))

    #remove first episode if < threshold in duration.
    #ToDo: why not merge the (small) first episode onto the (longer) second episode
    if len(filtered_intv_list) > 1 and \
        filtered_intv_list[0][e]-filtered_intv_list[0][start] < noise_threshold:
        filtered_intv_list.pop(0)
    return filtered_intv_list    




if __name__ == '__main__':
    base_data_dir = '/home/strands/STRANDS/'
    qsr_dir = os.path.join(base_data_dir, 'qsr_dump/')
    spatial_relations = pickle.load(open(os.path.join(qsr_dir +\
                            'all_qsrs_qtcb__0_01__False__True__03_03_2015.p')))
    cnt=0
    print len(spatial_relations['world_qsr_traces'])
    print type(spatial_relations['world_qsr_traces'])

    NOISE_THRESHOLD = 3
    all_episodes = {}

    for (uuid, qsr_world_trace) in spatial_relations['world_qsr_traces'].items():

        key, epi  = compute_episodes(uuid, qsr_world_trace)
        print cnt, key

        # Filter the episods to remove very short transitions that are noise
        fepi = {}
        for e in epi:
            fepi[e] = filter_intervals(epi[e], NOISE_THRESHOLD)

        # Add filtered episodes to all_episodes
        all_episodes[key] = fepi

        cnt+=1


