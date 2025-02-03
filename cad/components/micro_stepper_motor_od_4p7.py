"""Model of the micro stepper motor."""

import copy
import json
from dataclasses import dataclass
from datetime import UTC, datetime
from itertools import product
from pathlib import Path

import build123d as bd
import build123d_ease as bde
import git
from build123d_ease import show
from loguru import logger


@dataclass(kw_only=True)
class MainSpec:
    """Specification for a micro stepper motor."""

    motor_od: float = 4.7
    gearbox_od: float = 5.9  # Create a tiny lip.

    motor_length: float = 6.5

    shaft_od: float = 0.7
    shaft_length: float = 1.0  # Excluding the shaft lip.

    shaft_lip_od: float = 2.0
    shaft_lip_length: float = 0.4

    pin_od: float = 0.7
    pin_length: float = 1.0
    pin_spacing_x: float = 2.0
    pin_spacing_y: float = 3.1

    def __post_init__(self) -> None:
        """Post initialization checks."""
        data = {}
        logger.info(json.dumps(data, indent=2))

    def deep_copy(self) -> "MainSpec":
        """Copy the current spec."""
        return copy.deepcopy(self)


def make_micro_stepper_motor_od_4p7(spec: MainSpec) -> bd.Part:
    """Make a micro stepper motor with OD of 4.7mm.

    https://aliexpress.com/item/1005005998173233.html
    """
    p = bd.Part(None)

    # Motor body.
    p += bd.Pos(Z=-spec.shaft_lip_length) * bd.Cylinder(
        spec.motor_od / 2,
        spec.motor_length,
        align=bde.align.ANCHOR_TOP,
    )

    # Draw the shaft lip.
    p += bd.Cylinder(
        spec.shaft_lip_od / 2,
        spec.shaft_lip_length,
        align=bde.align.ANCHOR_TOP,
    )

    # Draw the shaft.
    p += bd.Cylinder(
        spec.shaft_od / 2,
        spec.shaft_length,
        align=bde.align.ANCHOR_BOTTOM,
    )

    for pin_x, pin_y in product(
        bde.evenly_space_with_center(count=2, spacing=spec.pin_spacing_x),
        (0.5, 0.5 + spec.pin_spacing_y),
    ):
        p += bd.Pos(
            X=pin_x, Z=(pin_y - spec.motor_length - spec.shaft_lip_length)
        ) * bd.Cylinder(
            spec.pin_od / 2,
            spec.pin_length + spec.motor_od / 2,
            align=bde.align.ANCHOR_BOTTOM,
        ).rotate(axis=bd.Axis.X, angle=90)

    return p


if __name__ == "__main__":
    start_time = datetime.now(UTC)
    py_file_name = Path(__file__).name
    logger.info(f"Running {py_file_name}")

    parts = {
        "micro_stepper_motor_od_4p7": show(make_micro_stepper_motor_od_4p7(MainSpec())),
    }

    logger.info("Saving CAD model(s)...")

    repo_dir = git.Repo(__file__, search_parent_directories=True).working_tree_dir
    assert repo_dir
    (export_folder := Path(repo_dir) / "build" / Path(__file__).stem).mkdir(
        exist_ok=True, parents=True
    )
    for name, part in parts.items():
        bd.export_stl(part, str(export_folder / f"{name}.stl"))
        bd.export_step(part, str(export_folder / f"{name}.step"))

    logger.info(f"Done running {py_file_name} in {datetime.now(UTC) - start_time}")
