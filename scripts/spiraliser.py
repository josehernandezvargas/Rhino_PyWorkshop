#! python3

"""Grasshopper Script

Provides a scripting component.
    Inputs:
        x: The x script variable
        y: The y script variable
    Output:
        a: The a output variable
"""
__author__ = "jose hernandez vargas"
__version__ = "2024-06-26"

import System
import Rhino
import Grasshopper



class Spiraliser(Grasshopper.Kernel.GH_ScriptInstance):

    ghenv.Component.Name = ""
    def RunScript(self, x:int, y: int, z:int):
        a = x+y
        print(a)
   
        return a