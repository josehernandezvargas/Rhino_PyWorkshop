#! python

__author__ = "jose hernandez vargas"
__version__ = "2026-07-28"

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
        # Local subprograms, each a full "DEF x ( ) ... END" block. They are
        # written after the main program's END - see full_program().
        self.subprograms = []
        self._target = None  # open subprogram line list, or None for the main program
        self._decl_index = None  # where KRL declarations get spliced (set by krl_header)

    def _emit(self, line: str):
        """Append a line to the open subprogram, or to the main program."""
        if self._target is None:
            self.code.append(line)
        else:
            self._target.append(line)

    def add_line(self, line: str):
        """Append a raw KRL line to the current target (main program or subprogram)."""
        self._emit(line)

    def add_declaration(self, line: str):
        """Insert a declaration into the main program's declaration block.

        KRL requires every declaration (DECL, INTERRUPT DECL) to appear before
        the first executable instruction, so these are spliced in right after
        the "DEF <name> ( )" line rather than appended. Declarations are kept
        in call order.
        """
        if self._decl_index is None:
            raise Exception("Call krl_header() before adding declarations.")
        self.code.insert(self._decl_index, line)
        self._decl_index += 1

    def start_subprogram(self, name: str):
        """Open a local subprogram; every emitter call is routed into it until
        end_subprogram() is called."""
        if self._target is not None:
            raise Exception("A subprogram is already open; call end_subprogram() first.")
        if not isinstance(name, str) or not name.strip():
            raise Exception("Subprogram name should be a non-empty string.")
        self._target = [f"DEF {name} ( )"]
        self.subprograms.append(self._target)

    def end_subprogram(self):
        """Close the open subprogram with its END and go back to the main program."""
        if self._target is None:
            raise Exception("No subprogram is open.")
        self._target.append("END")
        self._target = None

    def full_program(self):
        """Assemble main program + END + local subprograms.

        Does not mutate self.code, so it is safe to call more than once.
        """
        if self._target is not None:
            raise Exception("A subprogram is still open; call end_subprogram() first.")
        lines = list(self.code)
        lines.append("END")
        for block in self.subprograms:
            lines.append("")
            lines.extend(block)
        return lines

    def open_fold(self, comment: str):
        """Opens a fold with a comment."""
        self._emit(f";FOLD {comment}")

    def close_fold(self):
        """Closes a fold."""
        self._emit(";ENDFOLD")

    def add_comment(self, text: str):
        """Add a comment to the code."""
        self._emit(f";{text}")

    def set_velocity(self, velocity: float):
        """Set the LIN velocity in m/s. Hard-capped at MAX_LIN_VELOCITY_MPS for safety."""
        if velocity > self.MAX_LIN_VELOCITY_MPS:
            warnings.warn(
                f"LIN velocity {velocity} m/s exceeds the hardcoded "
                f"{self.MAX_LIN_VELOCITY_MPS * 1000:.0f} mm/s safety cap; clamping.",
                UserWarning,
            )
            velocity = self.MAX_LIN_VELOCITY_MPS
        self._emit(f"$VEL.CP={velocity}")

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
        # KRL wants all declarations before the first instruction; add_declaration()
        # splices them in here.
        self._decl_index = len(self.code)
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
        """Set the state of a specified output. KRL wants TRUE/FALSE, not Python's
        True/False."""
        self._emit(f"$OUT[{output_number}]={'TRUE' if state else 'FALSE'}")

    def wait(self, wait_time: float):
        """Wait for a specified amount of time."""
        self._emit(f"WAIT SEC {wait_time}")

    def wait_for_input(self, input_number: int, state: bool = True):
        """Hold the program until a digital input reaches the given state."""
        self._emit(f"WAIT FOR $IN[{input_number}]=={'TRUE' if state else 'FALSE'}")

    def ptp(self, position: tuple, e1: float = 0, e2: float = 0):
        """Point-to-point motion to a specified position with optional external axes."""
        if len(position) != 6:
            raise ValueError("Position must be a tuple with 6 values (X, Y, Z, A, B, C).")
        a1, a2, a3, a4, a5, a6 = position
        self._emit(
            f"PTP {{A1 {a1}, A2 {a2}, A3 {a3}, A4 {a4}, A5 {a5}, A6 {a6}, E1 {e1}, E2 {e2}, E3 0, E4 0, E5 0, E6 0}}"
        )

    def lin(self, position: tuple, approx: str = "C_DIS"):
        """Linear motion to a specified position with optional external axes.

        approx is the approximation suffix; pass None/"" for exact positioning
        (required, for instance, on the first motion after a RESUME).
        """
        if len(position) != 6:
            raise ValueError("Position must be a tuple with 6 values (X, Y, Z, A, B, C).")
        x, y, z, a, b, c = position
        line = (
            f"LIN {{X {x:.1f}, Y {y:.1f}, Z {z:.1f}, A {a:.2f}, B {b:.2f}, C {c:.2f}, E1 {0}, E2 {0}}}"
        )
        if approx:
            line += f" {approx}"
        self._emit(line)

    def lin_rel(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, frame: str = "#BASE"):
        """Relative linear motion from wherever the robot currently stands.

        Used when the position is not known at generation time (e.g. lifting
        clear of the part after an interrupt aborted a move mid-path). Always
        exact positioning - LIN_REL is not approximated here.
        """
        offsets = [(label, value) for label, value in (("X", x), ("Y", y), ("Z", z)) if value]
        if not offsets:
            raise ValueError("lin_rel needs at least one non-zero offset.")
        components = ", ".join(f"{label} {value:.1f}" for label, value in offsets)
        line = f"LIN_REL {{{components}}}"
        if frame:
            line += f" {frame}"
        self._emit(line)

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

        # full_program() closes the main program with END and appends any local
        # subprograms after it.
        lines = self.full_program()

        # Write each line of the KUKA src program to the specified file
        with open(truncated_filename, "w") as fileOut:
            fileOut.write("\n".join(lines))
