from __future__ import print_function, division

from colorama import Fore
from qsrlib_io.world_qsr_trace import World_QSR_Trace
from qsrlib_io.world_trace import World_Trace, Object_State
from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message

def print_success():
    print("\t\t" + Fore.GREEN + "done" + Fore.RESET)

def print_fail():
    print("\t\t" + Fore.RED + "fail" + Fore.RESET)

def cprint(s, color, reset=Fore.RESET):
    print(s + color + reset)

def colorify(color, s, reset=Fore.RESET):
    return color + s + reset

def merge_world_qsr_traces(traces, qsr_type=None):
    """
    Merge a list of traces into one world_qsr_trace. It offers no protection versus overwriting previously
    existing relation.
    :param traces: list of World_QSR_Trace objects
    :param qsr_type: the qsr_type of the returned merged World_QSR_Trace object; if nothing given it is retrieved
    from the qsr_type of the first object in the traces list
    :return: a World_QSR_Trace that is the merge of all World_QSR_Trace objects in traces
    """
    if len(traces) == 0:
        raise ValueError("'traces' can't be of 0 length")
    if qsr_type is None:
        qsr_type = traces[0].qsr_type
    world_qsr_trace = World_QSR_Trace(qsr_type=qsr_type)
    for trace in traces:
        for t, s in zip(trace.trace.keys(), trace.trace.values()):
            for k, qsr in zip(s.qsrs.keys(), s.qsrs.values()):
                world_qsr_trace.add_qsr(qsr, t)
    return world_qsr_trace



### TESTS
def test_merge_world_qsr_traces():
    which_qsr = "rcc3_rectangle_bounding_boxes_2d"
    traj = [Object_State(name="traj", timestamp=0, x=1., y=1., width=5., length=8.),
            Object_State(name="traj", timestamp=1, x=1., y=2., width=5., length=8.)]
    o1 = [Object_State(name="o1", timestamp=0, x=11., y=1., width=5., length=8.),
          Object_State(name="o1", timestamp=1, x=11., y=2., width=5., length=8.)]
    o2 = [Object_State(name="o2", timestamp=0, x=11., y=1., width=5., length=8.),
          Object_State(name="o2", timestamp=1, x=11., y=2., width=5., length=8.)]
    world_trace1 = World_Trace()
    world_trace1.add_object_state_series(traj)
    world_trace1.add_object_state_series(o1)
    world_trace2 = World_Trace()
    world_trace2.add_object_state_series(traj)
    world_trace2.add_object_state_series(o2)
    world_traces = [world_trace1, world_trace2]

    qsrlib = QSRlib()

    world_qsr_traces = []
    for world_trace in world_traces:
        request_message = QSRlib_Request_Message(which_qsr=which_qsr, input_data=world_trace, include_missing_data=True)
        qsrlib_res = qsrlib.request_qsrs(request_message=request_message)
        world_qsr_traces.append(qsrlib_res.qsrs)

    world_qsr_trace = merge_world_qsr_traces(world_qsr_traces, which_qsr)

    print("Response is:")
    for t in world_qsr_trace.get_sorted_timestamps():
        foo = str(t) + ": "
        for k, v in zip(world_qsr_trace.trace[t].qsrs.keys(), world_qsr_trace.trace[t].qsrs.values()):
            foo += str(k) + ":" + str(v.qsr) + "; "
        print(foo)


if __name__ == '__main__':
    test_merge_world_qsr_traces()