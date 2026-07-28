#! python

__author__ = "jose hernandez vargas"
__version__ = "2026-01-05"

import os
import warnings

import iolib as io

class KukaKRL:

    MAX_LIN_VELOCITY_MPS = 0.25  # hardcoded safety cap: 250 mm/s, LIN moves must never exceed this
    PTP_WARN_PERCENT = 10.0  # hardcoded safety threshold: warn above this PTP speed
    PTP_MAX_PERCENT = 25.0  # hardcoded safety cap: PTP speed must never exceed this

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
        """Set the LIN velocity in m/s. Hard-capped at MAX_LIN_VELOCITY_MPS for safety."""
        if velocity > self.MAX_LIN_VELOCITY_MPS:
            warnings.warn(
                f"LIN velocity {velocity} m/s exceeds the hardcoded "
                f"{self.MAX_LIN_VELOCITY_MPS * 1000:.0f} mm/s safety cap; clamping.",
                UserWarning,
            )
            velocity = self.MAX_LIN_VELOCITY_MPS
        self.code.append(f"$VEL.CP={velocity}")

    def _validate_percent(self, value: float, label: str, min_value: float = 0, max_value: float = 100):
        """Validate percent-like inputs and return a float."""
        return io.validate_scalar(label, value, min_value=min_value, max_value=max_value, allow_zero=False)

    def _sanitize_ptp_velocity_percent(self, value: float):
        """
        Validate PTP speed percent for safer motion.

        Warn above PTP_WARN_PERCENT and hard-cap at PTP_MAX_PERCENT.
        """
        value = self._validate_percent(value, "PTP velocity percent", min_value=0, max_value=100)
        if value > self.PTP_WARN_PERCENT:
            warnings.warn(
                f"PTP velocity is {value}%; values above {self.PTP_WARN_PERCENT:.0f}% "
                "should be double-checked.",
                UserWarning,
            )
        if value > self.PTP_MAX_PERCENT:
            warnings.warn(
                f"PTP velocity {value}% exceeds the hardcoded safety cap; "
                f"clamping to {self.PTP_MAX_PERCENT:.0f}%.",
                UserWarning,
            )
            value = self.PTP_MAX_PERCENT
        return value
    
    def _validate_tool_or_base_number(self, number, label):
        """Validate a tool/base number is within the KUKA-supported 1-16 range."""
        io.validate_scalar(label, number, min_value=1, max_value=16, allow_zero=True)

    def set_tool(self, number):
        """Defines the tool number. Accepts an integer between 1-16."""
        self._validate_tool_or_base_number(number, "Tool number")
        self.tool = number

    def set_base(self, number):
        """Defines the base number. Accepts an integer between 1-16."""
        self._validate_tool_or_base_number(number, "Base number")
        self.base = number

    def krl_header(
        self,
        startposition,
        ptp_velocity_percent: float = 20,
        ptp_acc_percent: float = 20,
        ptp_apo_dist: float = 50,
    ):
        """
        Writes a KRL header into self.code
        requires self.tool and self.base to be defined

        Args:
            startposition: Iterable with A1..A6 start joint angles.
            ptp_velocity_percent: PTP speed (%) for PDAT and BAS PTP params.
            ptp_acc_percent: PTP acceleration (%) for PDAT_ACT.
            ptp_apo_dist: Approximation distance for PDAT_ACT.
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
        base = self.base
        tool = self.tool
        ptp_velocity_percent = self._sanitize_ptp_velocity_percent(ptp_velocity_percent)
        ptp_acc_percent = self._validate_percent(
            ptp_acc_percent, "PTP acceleration percent", min_value=0, max_value=100
        )
        if not isinstance(ptp_apo_dist, (int, float)) or ptp_apo_dist < 0:
            raise ValueError("PTP APO distance must be a numeric value >= 0.")

        # header from template
        self.code.append("&ACCESS RVP")
        self.code.append("&REL 1")
        self.code.append("&PARAM TEMPLATE = C:\\KRC\\Roboter\\Template\\vorgabe")
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

        self.code.append(";FOLD STARTPOSITION - BASE IS {}, TOOL IS {}, SPEED IS {:.1f}%, POSITION IS A1 {},A2 {},A3 {},A4 {},A5 {},A6 {},E1 0,E2 0,E3 0,E4 0".format(
            base, tool, ptp_velocity_percent, A1, A2, A3, A4, A5, A6))
        self.code.append("$BWDSTART = FALSE")
        self.code.append(
            "PDAT_ACT = {{VEL {vel:.1f},ACC {acc:.1f},APO_DIST {apo:.1f}}}".format(
                vel=ptp_velocity_percent, acc=ptp_acc_percent, apo=ptp_apo_dist
            )
        )
        self.code.append(
            "FDAT_ACT = {{TOOL_NO {},BASE_NO {},IPO_FRAME #BASE}}".format(tool, base))
        self.code.append("BAS (#PTP_PARAMS,{:.1f})".format(ptp_velocity_percent))
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

    def ptp(self, position: tuple, e1: float = 0, e2: float = 0):
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
            f"LIN {{X {x:.1f}, Y {y:.1f}, Z {z:.1f}, A {a:.2f}, B {b:.2f}, C {c:.2f}, E1 {0}, E2 {0}}} C_DIS"
        )

    def _truncate_filename(self, filename: str, max_length: int = 24):
        base_name = os.path.basename(filename)
        if len(base_name) <= max_length:
            return filename, None

        root, ext = os.path.splitext(base_name)
        max_root_len = max_length - len(ext)
        if max_root_len <= 0:
            truncated_base = base_name[:max_length]
        else:
            truncated_base = root[:max_root_len] + ext

        warnings.warn(
            f'Filename "{base_name}" exceeds {max_length} characters; truncating to "{truncated_base}".',
            UserWarning,
        )

        truncated_path = os.path.join(os.path.dirname(filename), truncated_base)
        return truncated_path, base_name

    def write_file(self, filename):

        if len(os.path.basename(filename)) > 24:
            truncated_filename, _original_filename = self._truncate_filename(filename)
            self.add_comment(f"FULLNAME: {filename}")
        else:
            truncated_filename = filename
            

        # Since we are done adding lines to the program, we will END it
        self.code.append("END")

        # Write each line of the KUKA src program to the specified file
        with open(truncated_filename, "w") as fileOut:
            for line in range(len(self.code)-1):
                fileOut.write(self.code[line] + "\n")

            fileOut.write(self.code[-1])
