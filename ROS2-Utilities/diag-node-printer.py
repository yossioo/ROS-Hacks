#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rosidl_runtime_py import message_to_csv

import sys, time, threading, os
import datetime


try:
    from msgs_and_srvs.msg import BirdsDiagnostic as diag_msg
except Exception as e:
    from diagnostic_msgs.msg import DiagnosticArray as diag_msg
    from diagnostic_msgs.msg import *


frame_id = ""
namespace = "/"

for arg in sys.argv:
    vals = arg.split(":=")
    if len(vals) < 2:
        continue
    if vals[0] in ("target", "t"):
        frame_id = vals[1]

csv = False

for a in sys.argv:
    if "__ns:=" in a:
        namespace = a.replace("__ns:=", "")
        break


class Colors:
    RESET = "\033[0m"
    BLINK = "\033[5m"
    INVERT = "\033[7m"
    BLACK = "\033[30m"
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"
    MAGENTA = "\033[35m"
    CYAN = "\033[36m"
    WHITE = "\033[37m"
    BOLDBLACK = "\033[1m\033[30m"
    BOLDRED = "\033[1m\033[31m"
    BOLDGREEN = "\033[1m\033[32m"
    BOLDYELLOW = "\033[1m\033[33m"
    BOLDBLUE = "\033[1m\033[34m"
    BOLDMAGENTA = "\033[1m\033[35m"
    BOLDCYAN = "\033[1m\033[36m"
    BOLDWHITE = "\033[1m\033[37m"


timer = None


def diag_cb(msg: diag_msg):
    if frame_id in msg.header.frame_id:
        timer.reset()
        if csv:
            print(message_to_csv(msg))  # ,end="\r")
        else:
            now = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
            dt = datetime.datetime.fromtimestamp(now)
            s_now = dt.strftime("%H:%M:%S.%f")[:-3]
            print(
                f"[{s_now:}] {Colors.BOLDMAGENTA}<{msg.header.frame_id}>{Colors.RESET}"
            )
            s: DiagnosticStatus
            for s in msg.status:
                color = Colors.INVERT
                s_level = "STALE"
                if s.level == DiagnosticStatus.OK:
                    color, s_level = Colors.BOLDGREEN, "OK"
                elif s.level == DiagnosticStatus.WARN:
                    color, s_level = Colors.BOLDYELLOW, "WARN"
                elif s.level == DiagnosticStatus.ERROR:
                    color, s_level = Colors.BOLDRED, "ERROR"
                line = f"\t{color}[ {s.name:<22}:{s_level:>5} ]{Colors.RESET}"
                print(line)
                if s.message:
                    print(f"\t↳ {Colors.BOLDCYAN}{s.message}{Colors.RESET}")
                max_k_len = 0
                for v in s.values:
                    if len(v.key) > max_k_len:
                        max_k_len = len(v.key)
                for v in s.values:
                    print(f"\t  {v.key:<{max_k_len+1}}= {v.value}")
            print("-----")


def timer_cb():
    print(f"\nDiagnostics timeout from [{frame_id}]\n")
    timer.cancel()


rclpy.init(args=sys.argv)
node = Node("diag_listener", namespace=namespace)
sub = node.create_subscription(diag_msg, "diagnostics", diag_cb, 10)
timer = node.create_timer(2, timer_cb)
timer.reset()
print(f"Listening for {str(type(diag_msg))} from [{frame_id}]")
while rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.2)

node.destroy_node()
rclpy.shutdown()
