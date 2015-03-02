#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Usage example.

:Author: Yiannis Gatsoulis <y.gatsoulis@leeds.ac.uk>
:Organization: University of Leeds
"""

from __future__ import print_function, division
import sys
import argparse
from cad120_data_reader import CAD120_Data_Reader
from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message

if __name__ == '__main__':
    options = {"rcc3": "rcc3_rectangle_bounding_boxes_2d",
               "sg1": "sg1"}
    parser = argparse.ArgumentParser()
    parser.add_argument("qsr", help="choose qsr: %s" % options.keys(), action='store', type=str)
    args = parser.parse_args()
    try:
        which_qsr = options[args.qsr]
    except (IndexError, KeyError):
        print("ERROR: qsr not found")
        print("keywords:", options.keys())
        sys.exit(1)

    qsrlib = QSRlib()
    reader = CAD120_Data_Reader()
    foo = reader.world_traces.keys()
    bar = [s for s in foo if "Subject5" in s and "arranging_objects" in s]
    world_trace = reader.world_traces[bar[0]]
    request_message = QSRlib_Request_Message(which_qsr=which_qsr, input_data=world_trace, include_missing_data=True)
    out = qsrlib.request_qsrs(request_message=request_message)

    # some print some nice data
    print("Request was made at ", str(out.timestamp_request_made) + " and received at " + str(out.timestamp_request_received) + " and computed at " + str(out.timestamp_qsrs_computed) )
    for t in out.qsrs.get_sorted_timestamps():
        foo = str(t) + ": "
        for k, v in zip(out.qsrs.trace[t].qsrs.keys(), out.qsrs.trace[t].qsrs.values()):
            foo += str(k) + ":" + str(v.qsr) + "; "
        print(foo)
    print("-- THE END --")
