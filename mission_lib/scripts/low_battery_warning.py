#!/usr/bin/env python
from Tkinter import *
from tkMessageBox import *
import sys

main_window = Tk()
main_window.withdraw()
result = showwarning(title="Battery low", message="Drone "+sys.argv[1]+" returning home.")  # SYSTEM (Ubuntu) crashes here when executing all... problem of the system, not even the easiest dialog works.
main_window.destroy()
