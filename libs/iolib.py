"""Simple helpers to read and write CSV/XLSX files, plus shared input validation."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any, Callable, Iterable


def _as_path(path: str | Path) -> Path:
    return path if isinstance(path, Path) else Path(path)


def read_csv(path: str | Path, encoding: str = "utf-8") -> list[dict[str, Any]]:
    """Read a CSV file and return a list of row dictionaries."""
    file_path = _as_path(path)
    with file_path.open("r", newline="", encoding=encoding) as csvfile:
        return list(csv.DictReader(csvfile))


def write_csv(
    path: str | Path,
    rows: Iterable[dict[str, Any]],
    fieldnames: list[str] | None = None,
    encoding: str = "utf-8",
) -> Path:
    """Write rows to CSV and return the output path."""
    file_path = _as_path(path)
    file_path.parent.mkdir(parents=True, exist_ok=True)

    row_list = list(rows)
    if not row_list:
        raise ValueError("rows cannot be empty")

    if fieldnames is None:
        fieldnames = list(row_list[0].keys())

    with file_path.open("w", newline="", encoding=encoding) as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(row_list)

    return file_path


def read_xlsx(path: str | Path, sheet_name: str | None = None) -> list[dict[str, Any]]:
    """Read an XLSX sheet and return a list of row dictionaries."""
    try:
        from openpyxl import load_workbook
    except ImportError as exc:
        raise ImportError("openpyxl is required for XLSX support") from exc

    file_path = _as_path(path)
    workbook = load_workbook(filename=file_path, data_only=True, read_only=True)

    if sheet_name is None:
        sheet = workbook.active
    else:
        if sheet_name not in workbook.sheetnames:
            workbook.close()
            raise ValueError(f"Sheet '{sheet_name}' not found in '{file_path}'.")
        sheet = workbook[sheet_name]

    rows = list(sheet.iter_rows(values_only=True))
    workbook.close()

    if not rows:
        return []

    headers = [str(cell) if cell is not None else "" for cell in rows[0]]
    data_rows = rows[1:]
    return [dict(zip(headers, row)) for row in data_rows]


def write_xlsx(
    path: str | Path,
    rows: Iterable[dict[str, Any]],
    sheet_name: str = "Sheet1",
) -> Path:
    """Write rows to XLSX and return the output path."""
    try:
        from openpyxl import Workbook
    except ImportError as exc:
        raise ImportError("openpyxl is required for XLSX support") from exc

    file_path = _as_path(path)
    file_path.parent.mkdir(parents=True, exist_ok=True)

    row_list = list(rows)
    if not row_list:
        raise ValueError("rows cannot be empty")

    headers = list(row_list[0].keys())

    workbook = Workbook()
    sheet = workbook.active
    sheet.title = sheet_name

    sheet.append(headers)
    for row in row_list:
        sheet.append([row.get(col) for col in headers])

    workbook.save(file_path)
    workbook.close()
    return file_path


# ---------------------------------------------------------------------------
# Shared input validation
#
# These helpers centralize the numeric/type/list validation that was
# previously scattered and duplicated across geometrylib, curvelib, printlib,
# srflib, and kukalib. Other libs modules delegate to these internally while
# keeping their own public signatures unchanged.
# ---------------------------------------------------------------------------


class ValidationError(ValueError):
    """Raised when a validated value fails a domain/type/range check.

    Subclasses ValueError so existing ``except ValueError`` call sites
    continue to work unchanged.
    """


def report_issue(message: str, level: str = "warning", component: Any = None) -> None:
    """Report a message via a Grasshopper component if available, else print it.

    Mirrors the graceful-degradation pattern already used by
    ``scripts/kuka.py``'s ``_warn()``: when a live GH ``component`` (typically
    ``ghenv.Component``) is supplied, emit a runtime message; otherwise (e.g.
    when called from plain Python or a headless test) fall back to ``print``.
    """
    if component is not None:
        try:
            from Grasshopper.Kernel import GH_RuntimeMessageLevel as _RML

            level_map = {
                "remark": _RML.Remark,
                "warning": _RML.Warning,
                "error": _RML.Error,
            }
            component.AddRuntimeMessage(level_map.get(level, _RML.Warning), message)
            return
        except Exception:
            pass
    print(message)


def is_number(value: Any) -> bool:
    """Return True when value is an int or float, excluding bool."""
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def require_list(name: str, value: Any) -> list | tuple:
    """Ensure value is a list/tuple and return it, else raise ValidationError."""
    if value is None or not isinstance(value, (list, tuple)):
        raise ValidationError("{} should be a list or tuple.".format(name))
    return value


def require_nonempty_list(name: str, value: Any) -> list | tuple:
    """Ensure value is a non-empty list/tuple and return it."""
    value = require_list(name, value)
    if len(value) == 0:
        raise ValidationError("{} should not be empty.".format(name))
    return value


def validate_scalar(
    name: str,
    value: Any,
    min_value: float | None = None,
    max_value: float | None = None,
    allow_zero: bool = True,
) -> float:
    """Validate a numeric scalar and optionally enforce lower/upper bounds.

    Returns the value coerced to float. ``allow_zero`` controls whether
    ``min_value`` itself is an accepted value (``>=``) or excluded (``>``).
    """
    if not is_number(value):
        raise ValidationError("{} must be numeric.".format(name))
    value = float(value)
    if min_value is not None:
        if allow_zero:
            if value < min_value:
                raise ValidationError("{} must be >= {}.".format(name, min_value))
        elif value <= min_value:
            raise ValidationError("{} must be > {}.".format(name, min_value))
    if max_value is not None and value > max_value:
        raise ValidationError("{} must be <= {}.".format(name, max_value))
    return value


def validate_type(
    name: str,
    value: Any,
    expected_type: type,
    component: Any = None,
    allow_none: bool = False,
    coercer: Callable[[Any], Any] | None = None,
) -> Any:
    """Validate a value's type (with optional coercion), reporting via report_issue.

    Raises ValidationError when a required value is missing, TypeError when
    it has the wrong type or fails coercion — matching the exception types
    previously raised by ``geometrylib.validate_input``.
    """
    if value is None:
        if allow_none:
            return None
        report_issue("{} is missing.".format(name), level="warning", component=component)
        raise ValidationError("{} is missing".format(name))

    if coercer is not None:
        try:
            value = coercer(value)
        except Exception:
            report_issue(
                "{} could not be coerced to {}.".format(name, expected_type),
                level="warning",
                component=component,
            )
            raise

    if not isinstance(value, expected_type):
        report_issue(
            "{} must be {}. Got {}.".format(name, expected_type, type(value).__name__),
            level="warning",
            component=component,
        )
        raise TypeError("{} has wrong type".format(name))

    return value
