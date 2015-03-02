from __future__ import print_function, division

from colorama import Fore

def print_success():
    print("\t\t" + Fore.GREEN + "done" + Fore.RESET)

def print_fail():
    print("\t\t" + Fore.RED + "fail" + Fore.RESET)

def cprint(s, color, reset=Fore.RESET):
    print(s + color + reset)

def colorify(color, s, reset=Fore.RESET):
    return color + s + reset