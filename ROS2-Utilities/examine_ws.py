import sys, os
from collections import defaultdict

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    YELLOW = "\033[33m"
    RESET = "\033[0m"
    BLINK = "\033[5m"
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

def get_overriding_wss(ws_name, wss):
    ind = wss.index(ws_name)
    overriding = wss[ind:]
    overriding.remove(ws_name)
    if overriding:
        return overriding
    else:
        return []


def get_overriden_wss(ws_name, wss):
    ind = wss.index(ws_name)
    overriden = wss[:ind]
    if overriden:
        return overriden
    else:
        return []


def get_other_wss(ws_name, wss):
    return get_overriden_wss(ws_name, wss), get_overriding_wss(ws_name, wss)


package_paths = os.environ["AMENT_PREFIX_PATH"].split(":")
package_paths.reverse()

underlay = package_paths[0]
package_paths.remove(underlay)
ws_and_packages = defaultdict(lambda: [])
packages_and_ws = defaultdict(lambda: [])
for p in package_paths:
    words = p.split("/")
    package_name = words[-1]
    ws_name = words[-3]
    ws_and_packages[ws_name].append(package_name)
    packages_and_ws[package_name].append(ws_name)

print("\n" + bcolors.BOLDMAGENTA + "=== PACKAGES ===" + bcolors.ENDC)

for ws, pkgs in ws_and_packages.items():
    print("\n" + bcolors.BOLDCYAN + f"==> {ws} <==" + bcolors.ENDC)
    for p in pkgs:
        under, over = [], []
        if len(packages_and_ws[p]) > 1:
            under, over = get_other_wss(ws, packages_and_ws[p])
        icon = "↦"
        color = ""
        if under:
            icon = "🗹"
            color = bcolors.YELLOW
        if over:
            icon = "☒"
            color = bcolors.RED
        print(f"  {color}{icon} {p}{bcolors.ENDC}")
        for u in under:
            print(f"\t  {color}⚠ Overrides [{p}] in [{u}]{bcolors.ENDC}")
        for o in over:
            print(f"\t  {color}⚠ Overriden by [{p}] in [{o}]{bcolors.ENDC}")
