#! python

"""Grasshopper Script
"""

__author__ = "jose hernandez vargas"
__version__ = "2024-06-26"

class KukaKRL:

    def __init__(self, name):
        self.name = name
        self.base = None
        self.tool = None
        self.code = []

    def open_fold(self, comment: str):
        """Opens a fold with a comment."""
        self.code.append(f";FOLD {comment}")

    def close_fold(self):
        """Closes a fold."""
        self.code.append(";ENDFOLD")

    def add_comment(self, text: str):
        """Add a comment to the code."""
        self.code.append(f";{text}")
    
    def set_velocity(self, velocity: float):
        """Set the velocity in m/s. Raises a warning if velocity is unusually high."""
        if velocity > 1:
            print(
                f"Speed is defined in m/s. Provided speed is {velocity} m/s. Please check the units")
        self.code.append(f"$VEL.CP={velocity}")
    
    def set_tool(self, number):
        """Defines the tool number. Accepts an integer between 1-16."""
        if not 1 <= number <= 16:
            raise ValueError("Tool number must be between 1 and 16.")
        self.tool = number
    
    def set_base(self, number):
        """Defines the base number. Accepts an integer between 1-16."""
        if not 1 <= number <= 16:
            raise ValueError("Tool number must be between 1 and 16.")
        self.base = number

    def krl_header(self, startposition):
        """
        Writes a KRL header into self.code
        requires self.tool and self.base to be defined
        """    
        if not self.tool or not self.base:
            raise Exception(
                'You need to define a tool and a base first.')

            if len(startposition) == 6 and all(isinstance(i, (int, float)) for i in startposition):
                A1, A2, A3, A4, A5, A6 = startposition
            else:
                raise Exception(
                    "Start position should be a tuple with angles for each robot axis")
            # An Array that will contain all of the commands
            self.code = []
            base = self.base
            tool = self.tool

            # header from template
            self.code.append("&ACCESS RVP")
            self.code.append("&REL 1")
            self.code.append("&PARAM TEMPLATE = C:\KRC\Roboter\Template\\vorgabe")
            self.code.append("&PARAM EDITMASK = *")

            # add some initial setup stuff
            self.code.append("DEF "+str(self.name)+" ( )")
            self.code.append(";FOLD INI")
            self.code.append(";FOLD BASISTECH INI")
            self.code.append(
                "GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )")

            """
                INTERRUPT

                Description Executes one of the following actions:
                    - Activates an interrupt.
                    - Deactivates an interrupt.
                    - Disables an interrupt.
                    - Enables an interrupt.
                Up to 16 interrupts may be active at any one time
                
            """
            self.code.append("INTERRUPT ON 3")
            self.code.append("BAS (#INITMOV,0 )")
            self.code.append(";ENDFOLD (BASISTECH INI)")
            self.code.append(";ENDFOLD (INI)")

            self.code.append(";FOLD STARTPOSITION - BASE IS {}, TOOL IS {}, SPEED IS 100%, POSITION IS A1 {},A2 {},A3 {},A4 {},A5 {},A6 {},E1 0,E2 0,E3 0,E4 0".format(
                base, tool, A1, A2, A3, A4, A5, A6))
            self.code.append("$BWDSTART = FALSE")
            self.code.append("PDAT_ACT = {VEL 100,ACC 20,APO_DIST 50}")
            self.code.append(
                "FDAT_ACT = {{TOOL_NO {},BASE_NO {},IPO_FRAME #BASE}}".format(tool, base))
            self.code.append("BAS (#PTP_PARAMS,100)")
            self.code.append("PTP  {{A1 {},A2 {},A3 {},A4 {},A5 {},A6 {},E1 0,E2 0,E3 0,E4 0}}".format(
                A1, A2, A3, A4, A5, A6))
            self.code.append(";ENDFOLD")

            # self.code.append("$APO.CDIS = 0.5000")
            # self.code.append("BAS (#INITMOV,0)")
            # self.code.append("BAS (#VEL_PTP,20)")
            # self.code.append("BAS (#ACC_PTP,20)")
            # self.code.append("")

            """
                Advance run
                The advance run is the maximum number of motion blocks that the robot controller calculates and plans in advance during program execution. The actual
                number is dependent on the capacity of the computer.
                The advance run refers to the current position of the block pointer. It is set via
                the system variable $ADVANCE:
                    - Default value: 3
                    - Maximum value: 5
                The advance run is required, for example, in order to be able to calculate approximate positioning motions. If $ADVANCE = 0 is set, approximate positioning is not possible.
                Certain statements trigger an advance run stop. These include statements
                that influence the periphery, e.g. OUT statements
            """
            self.code.append("$ADVANCE=3")

    def set_output(self, output_number: int, state: bool):
        """Set the state of a specified output."""
        self.code.append(f"$OUT[{output_number}] = {state}")

    def wait(self, wait_time: float):
        """Wait for a specified amount of time."""
        self.code.append(f"WAIT SEC {wait_time}")

    def ptp(self, position: tuple):
        """Point-to-point motion to a specified position with optional external axes."""
        if len(position) != 6:
            raise ValueError("Position must be a tuple with 6 values (X, Y, Z, A, B, C).")
        a1, a2, a3, a4, a5, a6 = position
        self.code.append(
            f"PTP {{A1 {a1}, A2 {a2}, A3 {a3}, A4 {a4}, A5 {a5}, A6 {a6}, E1 {e1}, E2 {e2}, E3 0, E4 0, E5 0, E6 0}}"
        )

    def lin(self, position: tuple):
        """Linear motion to a specified position with optional external axes."""
        if len(position) != 6:
            raise ValueError("Position must be a tuple with 6 values (X, Y, Z, A, B, C).")
        x, y, z, a, b, c = position
        self.code.append(
            f"LIN {{X {x:.1f}, Y {y:.1f}, Z {z:.1f}, A {a:.2f}, B {b:.2f}, C {c:.2f}, E1 {e1}, E2 {e2}}} C_DIS"
        )

    def saveFile(self, filename):

        if (self.TOOL_IS_DEFINED == False):
            Exception('You must define a tool')
            return
        if (self.BASE_IS_DEFINED == False):
            Exception('You must define a base')
            return

        # Since we are done adding lines to the program, we will END it
        self.code.append("END")

        # Write each line of the KUKA src program to the specified file
        fileOut = open(filename, "w")
        for line in range(len(self.code)-1):
            fileOut.write(self.code[line] + "\n")

        fileOut.write(self.code[-1])
