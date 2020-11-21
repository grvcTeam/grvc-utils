#!/usr/bin/env python
from Tkinter import *
from tkMessageBox import *

main = Tk()
main.withdraw()
result = showwarning(title="Battery low", message="Returning home.")
main.destroy()
