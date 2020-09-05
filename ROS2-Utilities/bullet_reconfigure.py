#!/usr/bin/python3
import os, sys, time, threading
from bullet import Bullet, SlidePrompt, Check, Input, YesNo, Numbers
from bullet import styles
from bullet import colors
from typing import Tuple, Any
from rclpy.node import Node
import rclpy
from rcl_interfaces.srv import (
    ListParameters,
    GetParameters,
    GetParameterTypes,
    DescribeParameters,
    SetParameters,
)
from rcl_interfaces.msg import (
    ListParametersResult,
    Parameter,
    ParameterValue,
    ParameterDescriptor,
    ParameterType,
)


def yes_no(prompt, default_yes=False):
    cli = SlidePrompt([YesNo(prompt, default="y" if default_yes else "n")])
    result = cli.launch()
    for p, ans in result:
        if prompt in p:
            return ans


def select_bullet(prompt, choice_list):
    cli = SlidePrompt(
        [
            Bullet(
                prompt,
                choices=choice_list,
                bullet=" >",
                margin=2,
            ),
        ]
    )
    result = cli.launch()
    for p, ans in result:
        if p == prompt:
            return ans


def get_string(prompt, default_string=" "):
    cli = SlidePrompt(
        [
            Input(
                prompt, default=default_string, word_color=colors.foreground["yellow"]
            ),
        ]
    )
    result = cli.launch()
    for p, ans in result:
        if p == prompt:
            if ans == default_string:
                return False
            return ans


def get_param_type_and_value(pv: ParameterValue):
    type_name = "UNSUPPORTED"
    value = 0
    if pv.type == ParameterType.PARAMETER_NOT_SET:
        type_name = "NOT SET"
    elif pv.type == ParameterType.PARAMETER_BOOL:
        type_name = "BOOL"
        value = pv.bool_value
    elif pv.type == ParameterType.PARAMETER_INTEGER:
        type_name = "INTEGER"
        value = pv.integer_value
    elif pv.type == ParameterType.PARAMETER_DOUBLE:
        type_name = "DOUBLE"
        value = pv.double_value
    elif pv.type == ParameterType.PARAMETER_STRING:
        type_name = "STRING"
        value = pv.string_value
        pass
    return type_name, value


def is_value_param_valid(new_value, pv) -> Tuple[bool, Any]:
    if pv.type == ParameterType.PARAMETER_NOT_SET:
        return False, None
    elif pv.type == ParameterType.PARAMETER_BOOL:
        if new_value.lower() in ["true", "false", "t", "f", "yes", "no", "y", "n"]:
            return True, new_value.lower() in [
                "true",
                "t",
                "yes",
                "y",
            ]
        return False, None
    elif pv.type == ParameterType.PARAMETER_INTEGER:
        return new_value == int(new_value), int(new_value)
    elif pv.type == ParameterType.PARAMETER_DOUBLE:
        try:
            return True, float(new_value)
        except ValueError:
            return False, None
    elif pv.type == ParameterType.PARAMETER_STRING:
        return True, new_value
    else:
        return False, None


def get_parameter_value(p_type, value):
    p_value = ParameterValue()
    p_value.type = p_type
    if p_type == ParameterType.PARAMETER_BOOL:
        p_value.bool_value = value
    elif p_type == ParameterType.PARAMETER_INTEGER:
        p_value.integer_value = value
    elif p_type == ParameterType.PARAMETER_DOUBLE:
        p_value.double_value = value
    elif p_type == ParameterType.PARAMETER_STRING:
        p_value.string_value = value
    return p_value


rclpy.init(args=sys.argv)
node = Node("param_cli_setter")
time.sleep(1)

done = False
while not done:
    nodes = [
        ((ns + "/") if ns != "/" else "/") + name
        for name, ns in node.get_node_names_and_namespaces()
    ]
    nodes = [
        fqn for fqn in nodes if "ros2cli" not in fqn and node.get_name() not in fqn
    ]
    if not nodes:
        print("No nodes found, trying again",end="\r")
        time.sleep(1)
        continue
    req_node = select_bullet("Select a node to examine parameters", nodes)
    list_params_client = node.create_client(
        ListParameters, req_node + "/list_parameters"
    )
    get_params_client = node.create_client(GetParameters, req_node + "/get_parameters")
    set_params_client = node.create_client(SetParameters, req_node + "/set_parameters")
    time.sleep(0.2)
    if not list_params_client.service_is_ready():
        print(f"Service {list_params_client.srv_name} is not ready, waiting a second")
        time.sleep(1)
    if not get_params_client.service_is_ready():
        print(f"Service {get_params_client.srv_name} is not ready, waiting a second")
        time.sleep(1)
    if not set_params_client.service_is_ready():
        print(f"Service {set_params_client.srv_name} is not ready, waiting a second")
        time.sleep(1)
    print(f"Requesting parameter list for {req_node}")
    f = list_params_client.call_async(ListParameters.Request())
    req_time = time.time()
    while not f.done():
        rclpy.spin_once(node, timeout_sec=0.1)
    all_param_names = f.result().result.names
    print("Parameters received")

    # result_params = result_params.result
    params_found = False
    while not params_found:
        query_param_name = get_string("Enter a parameter name (or part of it): ")
        if query_param_name:
            filtered_params = [p for p in all_param_names if query_param_name in p]
        else:
            filtered_params = all_param_names
        if filtered_params:
            params_found = True
        else:
            print(f"Failed to found paramteres with mask *{query_param_name}*")
    req = GetParameters.Request()
    req.names = filtered_params
    f = get_params_client.call_async(req)
    while not f.done():
        rclpy.spin_once(node, timeout_sec=0.1)
    all_param_values = f.result().values
    parameters_prompt = []
    l = 0
    for p_name in filtered_params:
        if len(p_name) > l:
            l = len(p_name)
    names_values = {}
    names_types = {}
    for name, value in zip(filtered_params, all_param_values):
        t_str, v = get_param_type_and_value(value)
        names_types[name] = t_str
        names_values[name] = value
        line = f"{name:{l}s}\t{t_str}\t{v}"
        parameters_prompt.append(line)
    req_params = select_bullet("Select desired parameter", parameters_prompt)
    param_name = req_params.split()[0]
    p = f"Enter new value for parameter '{param_name}' of type {names_types[param_name]} "
    new_value_str = get_string(p)
    valid, new_value = is_value_param_valid(new_value_str, names_values[param_name])
    # print(valid, new_value_str, new_value)
    if valid:
        print(f"You've entered '{new_value_str}'' which translates to '{new_value}'")
        req = SetParameters.Request()
        param = Parameter()
        param.name = param_name
        param.value = get_parameter_value(names_values[param_name].type, new_value)
        # print("---------")
        # print(param.value)
        # print("---------")
        req.parameters.append(param)
        f = set_params_client.call_async(req)
        req_time = time.time()
        while not f.done():
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - req_time > 3.0:
                print("Timedout requesting set_parameters")
                break
        if f.done():
            result = f.result().results[0]
            if result.successful:
                print("Changed the parameter successfully :)")
            else:
                print("Failed to change parameter :(")
    else:
        print(
            f"Parameter value '{new_value_str}' invalid for type {names_types[param_name]}"
        )

    done = not yes_no("Start over? ")


node.destroy_node()
rclpy.shutdown()