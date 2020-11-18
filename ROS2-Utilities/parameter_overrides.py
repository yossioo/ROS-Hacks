#!/usr/bin/python3
import sys, time, datetime
import collections

from yaml import load, dump

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

filenames = "__params:=/home/yossi/goshawk_ws/install/goshawk/share/goshawk/params/vision_temp_params_remove_when_not_needed.yaml __params:=/home/yossi/goshawk_ws/install/goshawk/share/goshawk/params/gimbal_controller.params.yaml __params:=/home/yossi/goshawk_ws/install/goshawk/share/goshawk/params/general_params.yaml __params:=/home/yossi/goshawk_ws/install/goshawk/share/goshawk/params/mc.params.yaml __params:=/home/yossi/goshawk_ws/install/goshawk/share/goshawk/params/simulation.params.yaml __params:=/home/yossi/goshawk-private-parameters/local_params_override.yaml".split()
filenames = [fn.replace("__params:=", "") for fn in filenames]
# filenames = [
#     "/home/yossi/goshawk_ws/install/goshawk/share/goshawk/params/mc.params.yaml",
#     "/home/yossi/goshawk-private-parameters/local_params_override.yaml",
# ]


def flatten(d, parent_key="", sep="_"):
    items = []
    for k, v in d.items():
        new_key = parent_key + sep + k if parent_key else k
        if isinstance(v, collections.MutableMapping):
            items.extend(flatten(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)


def get_params_from_file(filename):
    with open(filename) as f:
        data = f.read()
    data_dict = load(data, Loader=Loader)
    parameters = flatten(data_dict["/**"]["ros__parameters"], sep=".")
    return parameters

all_parameters = {}
for f in filenames:
    parameters_from_file = get_params_from_file(f)
    print(f"============== File : {f} ==============")
    for p, v in parameters_from_file.items():
#         print(f"{p:<50}")
        if p in all_parameters.keys():
            print(f"{p:<50}")
            print(f"\tFrom: {all_parameters[p]}")
            print(f"\tTo:   {v}")
            prev_value = all_parameters[p]
#         print(f"{p:<50} {prev_value:>20} -> {str(v):<20}")
#         print(f"\tTo:   {v}")
        all_parameters[p] = v
        

# for p,fs in params_from_files.items():
#     print(f"{p}: ")
#     for f in fs:
#         print(f"\t->{f}")