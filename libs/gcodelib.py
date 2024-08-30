# libs/gcodelib.py
import time
import rhinoscriptsyntax as rs
import json
import os
import math

class GCodeLib:
    def __init__(self, filename, machine_file):
        """
        Initialize the GcodeHandler with a filename to write the Gcode commands.
        
        Parameters:
        filename (str): The name of the Gcode file to write.
        machine (str): The name of the machine configuration file (without extension).
        """
        self.filename = filename

        # Locate the path of the current file (gcodelib.py)
        current_dir = os.path.dirname(os.path.abspath(__file__))
        parent_dir = os.path.dirname(current_dir) 

        # Define the fixed machine_properties folder relative to gcodelib.py
        machine_folder = 'machine_settings'

        # Construct the full path to the machine JSON file
        machine_file = os.path.join(parent_dir, machine_folder, f'{machine_file}.json')
        
        # Load machine properties
        self.machine = self.load_machine_properties(machine_file)
        self.header = []
        self.commands = []
        self.minx = self.miny = self.minz = self.maxx = self.maxy = self.maxz = None
    
    def load_machine_properties(self, machine_file: str) -> dict:
        """
        Load machine properties from a JSON file.

        Parameters:
        machine_file (str): The path to the JSON file with machine properties.

        Returns:
        dict: The machine properties.
        """
        try:
            with open(machine_file, 'r') as file:
                machine_properties = json.load(file)
            return machine_properties
        except (FileNotFoundError, json.JSONDecodeError) as e:
            raise ValueError(f"Error loading machine properties: {e}")

    def add_comment(self, comment):
        """
        Add a comment to the Gcode file.
        
        Parameters:
        comment (str): The comment to add.
        """
        self.commands.append(f"; {comment}")
    
    def get_part_dims(self, geometry):
        """
        Also computes the min and max coordinates for the part
        """
        self.part = geometry

        bbox = rs.BoundingBox(geometry)
        if bbox:
            self.minx = bbox[0][0]
            self.miny = bbox[0][1]
            self.minz = bbox[0][2]
            self.maxx = bbox[6][0]
            self.maxy = bbox[6][1]
            self.maxz = bbox[6][2]
    
    def check_print(self):

        if self.part is None:
            raise ValueError("Part dimensions not set. Please run get_part_dims() before adding a header.")

        
        if self.minx < 0 or self.miny < 0 or self.maxx > 223 or self.maxy > 223:
            warning = "Out of the buildplate!"
            ghenv.Component.AddRuntimeMessage(
                gh.Kernel.GH_RuntimeMessageLevel.Error, warning)

        if self.maxz > maxheight:
            warning = "MAX HEIGHT EXCEDDED"
            ghenv.Component.AddRuntimeMessage(
                gh.Kernel.GH_RuntimeMessageLevel.Error, warning)
        if self.minz >= 1:
            warning = "Flying model! (not attached to the buildplate)"
            ghenv.Component.AddRuntimeMessage(gh.Kernel.GH_RuntimeMessageLevel.Warning, warning)


    def add_header(self, nozzle, flow):

        if self.part is None:
            raise ValueError("Part dimensions not set. Please run get_part_dims() before adding a header.")

        
        timestamp = time.strftime("%Y%m%d")  # adds a timestamp with the date
        hourstamp = " at " + time.strftime("%X")  # a timestamp with the hour

        self.header = [
        f";FLAVOR:{self.machine['gcode_flavour']}",
        ";MATERIAL:1",
        f";TARGET_MACHINE.NAME:{self.machine['machine_name']}",
        f";NOZZLE_DIAMETER:{nozzle}",
        f";MINX: {self.minx}",
        f";MINY: {self.miny}",
        f";MINZ: {self.minz}",
        f";MAXX: {self.maxx}",
        f";MAXY: {self.maxy}",
        f";MAXZ: {self.maxz}",
        ";Generated with Python / GH",
        f";File created {timestamp}",
        f";at {hourstamp}",
        f";OVERFLOW: {flow}",
        "M82 ;absolute extrusion mode",
        ";END_OF_HEADER"
        ]

    def save(self):
        """
        Save the Gcode commands to the file
        """
        extension = '.gcode'
        timestamp = time.strftime("%Y%m%d")  # adds a timestamp with the date

        file += '\\' + timestamp + "_" + self.filename + extension
        
        with open(file, 'w') as file:
            for line in self.header:
                file.write(f"{line}\n")
            for command in self.commands:
                file.write(f"{command}\n")

