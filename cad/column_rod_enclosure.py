"""Enclosure for the column rod design family, like the `dot_column_cam_rod_octagon`.

This family is the set of designs that looks like foosball.

## Concept

* Exterior housing (solid relative to user; PCB moves 1mm inside).
* Use screws to raise and lower the PCB inside the fixed enclosure. In each corner:
    * Anchor an M2 or M3 screw through the top and bottom enclosures.
    * Use a fixed M2 or M3 bolt through a tapped metal spur gear.
    * Use a solder-on or glue-on tab over the spur gear to axially constrain the spur
        gear to the PCB.
    * Zeroing with magnet or conductive tin foil tab (or conductive ESD sponge) from
        top-side to the PCB.
"""

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
    """Specification for braille enclosure."""

    dot_pitch_x: float = 2.5
    dot_pitch_y: float = 2.5
    cell_pitch_x: float = 6
    cell_pitch_y: float = 10
    cell_count_x: int = 4
    dot_hole_id: float = 1.7

    pcb_length_x: float = 60
    pcb_length_y: float = 50
    # PCB + Housing + Screw_Heads -or- PCB + Raiser_Motor_OD + Raiser_Motor_Clip
    pcb_and_housing_thickness_z: float = 1.6 + 6.0 + 1.0
    pcb_travel_z: float = 2
    pcb_extra_z: float = 3  # Extra dist in Z, for housing-to-PCB screw heads.

    pcb_raiser_screw_sep_x: float = 50
    pcb_raiser_screw_sep_y: float = 40
    pcb_raiser_screw_diameter: float = 2  # M2

    enclosure_total_x: float = 100
    enclosure_total_y: float = 60
    enclosure_wall_thickness_xy: float = 2.2
    enclosure_wall_thickness_bottom: float = 2
    enclosure_wall_thickness_top: float = 2

    # Bolts for joining the top/bottom halves of the enclosure securely.
    joiner_bolt_d: float = 3  # M3.
    joiner_bolt_margin: float = 7  # Dist from outer wall of enclosure.
    joiner_bolt_wall_thickness: float = 3

    enclosure_fillet_radius_xy: float = 2

    def __post_init__(self) -> None:
        """Post initialization checks."""
        data = {
            "enclosure_total_z": self.enclosure_total_z,
        }
        logger.info(json.dumps(data, indent=2))

    def deep_copy(self) -> "MainSpec":
        """Copy the current spec."""
        return copy.deepcopy(self)

    @property
    def enclosure_total_z(self) -> float:
        """Total thickness of the enclosure."""
        return (
            self.enclosure_wall_thickness_bottom
            + self.pcb_and_housing_thickness_z
            + self.pcb_travel_z
            + self.enclosure_wall_thickness_top
        )

    def get_cell_center_x_values(self) -> list[float]:
        """Get the X coordinate of the center of each cell."""
        return bde.evenly_space_with_center(
            count=self.cell_count_x,
            spacing=self.cell_pitch_x,
        )


def make_enclosure_top(spec: MainSpec) -> bd.Part:
    """Make the top half of the enclosure."""
    p = bd.Part(None)

    outer_box = bd.Box(
        spec.enclosure_total_x,
        spec.enclosure_total_y,
        spec.enclosure_total_z,
        align=bde.align.ANCHOR_BOTTOM,
    )
    p += bd.fillet(
        outer_box.edges().filter_by(bd.Axis.Z), radius=spec.enclosure_fillet_radius_xy
    )

    # Remove the inside of the enclosure.
    inner_box = bd.Box(
        spec.enclosure_total_x - 2 * spec.enclosure_wall_thickness_xy,
        spec.enclosure_total_y - 2 * spec.enclosure_wall_thickness_xy,
        spec.enclosure_total_z - spec.enclosure_wall_thickness_top,
        align=bde.align.ANCHOR_BOTTOM,
    )
    p -= bd.fillet(
        inner_box.edges().filter_by(bd.Axis.Z), radius=spec.enclosure_fillet_radius_xy
    )

    # Add the `joiner_bolt` (top-to-bottom connections).
    for x_sign, y_sign in product((1, -1), (1, -1)):
        bottom_of_joiner_bolt_pos = bd.Pos(
            x_sign * (spec.enclosure_total_x / 2 - spec.joiner_bolt_margin),
            y_sign * (spec.enclosure_total_y / 2 - spec.joiner_bolt_margin),
            spec.enclosure_wall_thickness_bottom,
        )
        p += bottom_of_joiner_bolt_pos * bd.Cylinder(
            radius=spec.joiner_bolt_d / 2 + spec.joiner_bolt_wall_thickness,
            height=(
                spec.enclosure_total_z
                - spec.enclosure_wall_thickness_top
                - spec.enclosure_wall_thickness_bottom
            ),
            align=bde.align.ANCHOR_BOTTOM,
        )
        p -= bottom_of_joiner_bolt_pos * bd.Cylinder(
            radius=spec.joiner_bolt_d / 2,
            height=(spec.enclosure_total_z - spec.enclosure_wall_thickness_bottom),
            align=bde.align.ANCHOR_BOTTOM,
        )

    # Remove the holes for the `pcb_raiser_screw`.
    for x_sign, y_sign in product((1, 0, -1), (1, -1)):
        pos = bd.Pos(
            x_sign * spec.pcb_raiser_screw_sep_x / 2,
            y_sign * spec.pcb_raiser_screw_sep_y / 2,
            spec.enclosure_wall_thickness_bottom,
        )
        p -= pos * bd.Cylinder(
            radius=spec.pcb_raiser_screw_diameter / 2,
            height=spec.enclosure_total_z,
            align=bde.align.ANCHOR_BOTTOM,
        )

    # Remove the braille dot holes.
    for cell_x, rod_offset_x, dot_offset_y in product(
        spec.get_cell_center_x_values(),
        bde.evenly_space_with_center(count=2, spacing=spec.dot_pitch_x),
        bde.evenly_space_with_center(count=3, spacing=spec.dot_pitch_y),
    ):
        dot_x = cell_x + rod_offset_x
        dot_y = dot_offset_y

        p -= bd.Pos(dot_x, dot_y) * bd.Cylinder(
            radius=spec.dot_hole_id / 2,
            height=spec.enclosure_total_z,
            align=bde.align.ANCHOR_BOTTOM,
        )

    return p


def make_enclosure_bottom(spec: MainSpec) -> bd.Part:
    """Make the bottom plate of the enclosure."""
    p = bd.Part(None)

    # Create the bottom plate as the dimensions of the inner part of the box.
    inner_box = bd.Box(
        spec.enclosure_total_x - 2 * spec.enclosure_wall_thickness_xy,
        spec.enclosure_total_y - 2 * spec.enclosure_wall_thickness_xy,
        spec.enclosure_wall_thickness_bottom,
        align=bde.align.ANCHOR_BOTTOM,
    )
    p += bd.fillet(
        inner_box.edges().filter_by(bd.Axis.Z), radius=spec.enclosure_fillet_radius_xy
    )

    # Add the `joiner_bolt` (top-to-bottom connections).
    for x_sign, y_sign in product((1, -1), (1, -1)):
        bottom_of_joiner_bolt_pos = bd.Pos(
            x_sign * (spec.enclosure_total_x / 2 - spec.joiner_bolt_margin),
            y_sign * (spec.enclosure_total_y / 2 - spec.joiner_bolt_margin),
        )
        p -= bottom_of_joiner_bolt_pos * bd.Cylinder(
            radius=spec.joiner_bolt_d / 2,
            height=spec.enclosure_wall_thickness_bottom,
            align=bde.align.ANCHOR_BOTTOM,
        )

    # Remove the holes for the `pcb_raiser_screw`.
    for x_sign, y_sign in product((1, 0, -1), (1, -1)):
        pos = bd.Pos(
            x_sign * spec.pcb_raiser_screw_sep_x / 2,
            y_sign * spec.pcb_raiser_screw_sep_y / 2,
        )
        p -= pos * bd.Cylinder(
            radius=spec.pcb_raiser_screw_diameter / 2,
            height=spec.enclosure_total_z,
            align=bde.align.ANCHOR_BOTTOM,
        )

    return p


if __name__ == "__main__":
    start_time = datetime.now(UTC)
    py_file_name = Path(__file__).name
    logger.info(f"Running {py_file_name}")

    parts = {
        "enclosure_top": show(make_enclosure_top(MainSpec())),
        "enclosure_bottom": (make_enclosure_bottom(MainSpec())),
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
