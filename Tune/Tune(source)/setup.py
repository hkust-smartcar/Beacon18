import sys
import os
from cx_Freeze import setup, Executable
import matplotlib
'''
# Dependencies are automatically detected, but it might need fine tuning.
additional_mods = ['tkinter.filedialog']
build_exe_options = {"packages":["tkinter","matplotlib"],'includes': additional_mods,"include_files": [r".\log.txt"]}
'''
build_exe_options = {"include_files": [r".\log.txt"]}

# GUI applications require a different base on Windows (the default is for a
# console application).
base = None
if sys.platform == 'win32':
    base = 'Win32GUI'
executables = [
    Executable('debug.py', base=base)
]
os.environ['TCL_LIBRARY'] = r'C:\Users\Sheldon\Anaconda3\tcl\tcl8.6'
os.environ['TK_LIBRARY'] = r'C:\Users\Sheldon\Anaconda3\tcl\tk8.6'

setup(  name = "Tune",
        version = "1.0",
        description = "To cook good Bacon",
        options = {"build_exe": build_exe_options},
        executables = executables)
