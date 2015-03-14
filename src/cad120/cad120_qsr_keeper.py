# -*- coding: utf-8 -*-
"""
CAD120 data reader that is compatible with QSRlib.

:Author: Yiannis Gatsoulis <y.gatsoulis@leeds.ac.uk>
:Organization: University of Leeds
"""

from __future__ import print_function, division
import sys
import argparse
import timeit
import ConfigParser
try:
    import cPickle as pickle
except ImportError:
    import pickle
from cad120_data_reader import CAD120_Data_Reader
from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
from utilities import *


class CAD120_QSR_Keeper(object):
    def __init__(self, description="", reader=None, qsrlib=None, which_qsr="", load_from_file=""):
        start = timeit.default_timer()
        print("\n--", self.__class__.__name__)
        print("Generating QSRs...")

        self.description = description
        self.reader = reader
        self.qsrlib = qsrlib
        self.which_qsr = which_qsr
        self.world_qsr_traces = {}

        if load_from_file is not None and load_from_file != "":
            self.load(load_from_file)
        else:
            if type(self.reader) is not CAD120_Data_Reader:
                raise TypeError("Provide a CAD120_Data_Reader object")
            if type(self.qsrlib) is not QSRlib:
                raise TypeError("Provide a QSRlib object")
            if self.which_qsr == "":
                raise ValueError("Provide an appropriate QSR")
            self.make()

        stop = timeit.default_timer()
        print("QSRs generated in: %.2f secs" % (stop - start))

    def make(self, qsrlib=None):
        if qsrlib:
            self.qsrlib = qsrlib
        if self.qsrlib is None:
            raise TypeError("Pass a QSRlib object")
        for k, world_trace in zip(self.reader.world_traces.keys(), self.reader.world_traces.values()):
            request_message = QSRlib_Request_Message(which_qsr=self.which_qsr, input_data=world_trace, include_missing_data=True)
            # out = self.qsrlib.request_qsrs(request_message=request_message)
            self.world_qsr_traces[k] = self.qsrlib.request_qsrs(request_message=request_message)

    def save(self, filename):
        print("Saving...")
        foo = {"description": self.description, "which_qsr": self.which_qsr, "world_qsr_traces": self.world_qsr_traces}
        with open(filename, "wb") as f:
            pickle.dump(foo, f)
        print_success()

    def load(self, filename):
        print("Loading QSRs from", filename, end="")
        with open(filename, "rb") as f:
            foo = pickle.load(f)
        self.description = foo["description"]
        self.which_qsr = foo["which_qsr"]
        self.world_qsr_traces = foo["world_qsr_traces"]
        print_success()


if __name__ == '__main__':
    start = timeit.default_timer()

    options = {"sg1": "sg1",
               "rcc3": "rcc3_rectangle_bounding_boxes_2d"}
    parser = argparse.ArgumentParser(description="CAD120 QSR keeper in QSRlib format")
    parser_group = parser.add_mutually_exclusive_group(required=True)
    parser.add_argument("-s", "--save", help="filename to save qsrs", type=str)
    parser_group.add_argument("-l", "--load", help="filename to load qsrs from instead of creating them from data", type=str)
    parser_group.add_argument("--qsr", help="choose qsr: %s" % options.keys(), type=str)
    args = parser.parse_args()

    if args.load is None:
        config_parser = ConfigParser.SafeConfigParser()
        if len(config_parser.read("local.ini")) == 0:
            raise ValueError("'local.ini' not found")
        try:
            reader_ini = config_parser.get("local", "reader_ini")
            reader_load = config_parser.getboolean("local", "reader_load")
        except ConfigParser.NoOptionError:
            raise

        try:
            which_qsr = options[args.qsr]
        except (IndexError, KeyError) as e:
            parser.print_help()
            sys.exit(1)

        qsrlib = QSRlib()
        reader = CAD120_Data_Reader(config_filename=reader_ini, load_from_files=reader_load)
        print()
        keeper = CAD120_QSR_Keeper(description="description", reader=reader, qsrlib=qsrlib, which_qsr=which_qsr)
        # optional saving
        if args.save:
            keeper.save(filename=args.save)
    else:
        keeper = CAD120_QSR_Keeper(load_from_file=args.load)

    stop = timeit.default_timer()
    print("Total execution time: %.2f secs" % (stop - start))
