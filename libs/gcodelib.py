# libs/gcodelib.py
import time

class GCodeLib:
    def __init__(self, filename, machine):
        """
        Initialize the GcodeHandler with a filename to write the Gcode commands.
        
        Parameters:
        filename (str): The name of the Gcode file to write.
        """
        self.filename = filename
        self.machine = machine
        self.header = []
        self.commands = []

    def add_comment(self, comment):
        """
        Add a comment to the Gcode file.
        
        Parameters:
        comment (str): The comment to add.
        """
        self.commands.append(f"; {comment}")
    
    def check_print(self, machine, part):
        
        bbox = rs.BoundingBox(part)

        minx = bbox[0][0]
        miny = bbox[0][1]
        minz = bbox[0][2]
        maxx = bbox[6][0]
        maxy = bbox[6][1]
        maxz = bbox[6][2]

        if minx < 0 or miny < 0 or maxx > 223 or maxy > 223:
            warning = "Out of the buildplate!"
            ghenv.Component.AddRuntimeMessage(
                gh.Kernel.GH_RuntimeMessageLevel.Error, warning)

        if maxz > maxheight:
            warning = "MAX HEIGHT EXCEDDED"
            ghenv.Component.AddRuntimeMessage(
                gh.Kernel.GH_RuntimeMessageLevel.Error, warning)
        if minz >= 1:
            warning = "Flying model! (not attached to the buildplate)"
            ghenv.Component.AddRuntimeMessage(gh.Kernel.GH_RuntimeMessageLevel.Warning, warning)


    def add_header(self, machine, nozzle, part):

        timestamp = time.strftime("%Y%m%d")  # adds a timestamp with the date
        hourstamp = " at " + time.strftime("%X")  # a timestamp with the hour

        self.header.append(";FLAVOR:UltiGCode")
        self.header.append(";MATERIAL:1")
        self.header.append(";TARGET_MACHINE.NAME:Ultimaker 2+")
        self.header.append(";NOZZLE_DIAMETER:" + str(nozzle))
        self.header.append(";MINX:" + str(minx))
        self.header.append(";MINY:" + str(miny))
        self.header.append(";MINZ:" + str(minz))
        self.header.append(";MAXX:" + str(maxx))
        self.header.append(";MAXY:" + str(maxy))
        self.header.append(";MAXZ:" + str(maxz))
        self.header.append(";Generated with Python / GH")
        self.header.append(";File created " + timestamp )
        self.header.append(";" + hourstamp)
        self.header.append(";OVERFLOW: " + str(flow))
        self.header.append("M82 ;absolute extrusion mode")
        self.header.append(";END_OF_HEADER")

    def save(self):
        """
        Save the Gcode commands to the file.
        """
        with open(self.filename, 'w') as file:
            for command in self.commands:
                file.write(f"{command}\n")

