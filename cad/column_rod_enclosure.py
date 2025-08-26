"""Enclosure for the column rod design family, like the `dot_column_cam_rod_octagon`.

This family is the set of designs that looks like foosball.

## Concept

* Exterior housing (solid relative to user; PCB moves 1mm inside).
* Use `pcb_raiser_cam` to raise and lower the PCB inside the fixed enclosure.
* In each corner:
    * Anchor an M2 or M3 screw through the top and bottom enclosures.
    * Use fixed metal rods/bolts in each corner of the enclosure.
    * Use springs pushing down on the PCB to keep the dots in the "lowered" position for
        most CAM positions. Do this because the spring is bulky, and therefore should go
        on the top side.
    * Use the cams to push the PCB up against the springs (cams push against the bottom
        plate) to raise into the "feel the dots now" state.
"""

import copy
import json
import math
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
class Spec:
    """Specification for braille enclosure."""

    dot_pitch_x: float = 2.5
    dot_pitch_y: float = 2.5
    cell_pitch_x: float = 6
    cell_pitch_y: float = 10
    cell_count_x: int = 4
    dot_hole_id: float = 1.7

    pcb_length_x: float = 95
    pcb_length_y: float = 99

    # Settings controlling the Z travel of the PCB inside in the enclosure.
    pcb_thickness: float = 0.8
    cam_motor_od: float = 6.0
    cam_motor_clamp_top_thickness_plus_bolt_heads: float = 1.5 + 4
    pcb_travel_z: float = 1.2
    cam_min_od: float = 7.4
    cam_max_od: float = 8.6
    cam_avg_od: float = 8.0

    # PCB layout - corner screws.
    pcb_raiser_screw_sep_x: float = 42.6
    pcb_raiser_screw_sep_y_1: float = 88.0
    pcb_raiser_screw_sep_y_2: float = 70.0
    pcb_raiser_screw_diameter: float = 3  # M3

    # PCB layout - where is the braille display.
    # _pcb_center_y = (75.5 + 174.5) / 2 = 125 (which is also the center of the dots).
    braille_display_y_offset_from_center: float = 0.0
    # _pcb_center_x = (84 + 179) / 2 = 131.5
    # _dots_center_x = 110.25
    # _pcb_to_dots_x = 110.25 - 131.5 = -21.25
    braille_display_x_offset_from_center: float = -21.25

    # PCB layout - where is the thick RP2040-Zero and USB port to remove from the top?
    rp2040_cutout_x_range: tuple[float, float] = (18, 400)
    rp2040_cutout_y_range: tuple[float, float] = (-26, 15)

    # PCB layout - where do the parts that interface with the cams go?
    # These cam interfaces are drawn on the inside of the bottom wall.
    cam_interface_x_sep: float = 64.2  # 163.6 - 99.4
    cam_interface_y_sep: float = 79.0  # 164 - 85
    cam_interface_width_x: float = 2.0
    cam_interface_width_y: float = 8.0

    # PCB layout - where do the spring posts go?
    # These spring posts are drawn on the inside of the top wall.
    spring_post_od = 3.0
    spring_post_margin_from_pcb_edge: float = 4.5  # Dist: PCB edge to center of post.

    # Overall enclosure dimensions.
    enclosure_total_x: float = 130
    enclosure_total_y: float = 125
    enclosure_wall_thickness_xy: float = 2.2
    enclosure_wall_thickness_bottom: float = 2
    enclosure_wall_thickness_top: float = 2
    enclosure_bottom_wall_standoff_height: float = 4.0  # Avoid THT solder points, nuts.
    enclosure_bottom_wall_standoff_od: float = 6

    # Bolts for joining the top/bottom halves of the enclosure securely.
    joiner_bolt_d: float = 3  # M3.
    joiner_bolt_margin: float = 7  # Dist from outer wall of enclosure.
    joiner_bolt_wall_thickness: float = 3

    enclosure_fillet_radius_xy: float = 2

    bottom_debugging_hole_od: float = 60.0

    def __post_init__(self) -> None:
        """Post initialization checks."""
        data = {
            "pcb_and_housing_thickness_z": self.pcb_and_housing_thickness_z,
            "enclosure_total_z": self.enclosure_total_z,
            "spring_post_length": self.spring_post_length,
        }
        logger.info(json.dumps(data, indent=2))

        assert math.isclose(
            self.cam_max_od - self.cam_min_od,
            self.pcb_travel_z,
        )

        assert math.isclose(
            (self.cam_min_od + self.cam_max_od) / 2,
            self.cam_avg_od,
        )

    def deep_copy(self) -> "Spec":
        """Copy the current spec."""
        return copy.deepcopy(self)

    @property
    def pcb_and_housing_thickness_z(self) -> float:
        """Max thickness from the bottom of the PCB to the inside of the enclosure."""
        # TODO(KilowattSynthesis): Update to be max(housing_height, raiser_height)

        # housing_height = PCB + Housing + Screw_Heads
        # raiser_height = PCB + Raiser_Motor_OD + Raiser_Motor_Clip (this one)
        raiser_height = (
            self.cam_motor_clamp_top_thickness_plus_bolt_heads
            + self.cam_motor_od
            + self.pcb_thickness
        )
        return raiser_height

    @property
    def enclosure_total_z(self) -> float:
        """Total thickness of the enclosure."""
        return (
            self.enclosure_wall_thickness_bottom
            + self.enclosure_bottom_wall_standoff_height
            + self.pcb_and_housing_thickness_z
            + self.pcb_travel_z
            + self.enclosure_wall_thickness_top
        )

    @property
    def spring_post_length(self) -> float:
        """Length of the spring posts."""
        return self.pcb_and_housing_thickness_z - self.pcb_thickness

    def get_cell_center_x_values(self) -> list[float]:
        """Get the X coordinate of the center of each cell."""
        return bde.evenly_space_with_center(
            count=self.cell_count_x,
            spacing=self.cell_pitch_x,
        )

    def get_pcb_raiser_screw_coordinates(self) -> list[tuple[float, float]]:
        """Get the coordinates of the PCB raiser screws."""
        return [
            (
                x_sign * self.pcb_raiser_screw_sep_x / 2,
                y_sign * y_sep / 2,
            )
            for x_sign, y_sign in product((1, -1), (1, -1))
            for y_sep in (self.pcb_raiser_screw_sep_y_1, self.pcb_raiser_screw_sep_y_2)
        ]


def box_from_ranges(
    range_x: tuple[float, float],
    range_y: tuple[float, float],
    range_z: tuple[float, float],
) -> bd.Box:
    """Create a box with the given dimensions."""
    assert range_x[1] > range_x[0]
    assert range_y[1] > range_y[0]
    assert range_z[1] > range_z[0]

    return bd.Box(
        range_x[1] - range_x[0],
        range_y[1] - range_y[0],
        range_z[1] - range_z[0],
        align=(bd.Align.MIN, bd.Align.MIN, bd.Align.MIN),
    ).translate((range_x[0], range_y[0], range_z[0]))


def make_enclosure_top(spec: Spec) -> bd.Part | bd.Compound:
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
    inside_of_box_z_val = spec.enclosure_total_z - spec.enclosure_wall_thickness_top
    inner_box = bd.Box(
        spec.enclosure_total_x - 2 * spec.enclosure_wall_thickness_xy,
        spec.enclosure_total_y - 2 * spec.enclosure_wall_thickness_xy,
        inside_of_box_z_val,
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
    for x_val, y_val in spec.get_pcb_raiser_screw_coordinates():
        p -= bd.Pos(
            x_val,
            y_val,
            spec.enclosure_wall_thickness_bottom,
        ) * bd.Cylinder(
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
        dot_x = spec.braille_display_x_offset_from_center + cell_x + rod_offset_x
        dot_y = spec.braille_display_y_offset_from_center + dot_offset_y

        p -= bd.Pos(dot_x, dot_y) * bd.Cylinder(
            radius=spec.dot_hole_id / 2,
            height=spec.enclosure_total_z,
            align=bde.align.ANCHOR_BOTTOM,
        )

    # Remove the RP2040 cutout.
    p -= box_from_ranges(
        range_x=spec.rp2040_cutout_x_range,
        range_y=spec.rp2040_cutout_y_range,
        range_z=(-50, 50),
    )

    # Add spring posts.
    spring_post_y_max = inside_of_box_z_val
    # spring_post_y_min = (  # Old complex logic before simplification by substitution.
    #     spec.enclosure_wall_thickness_bottom
    #     + spec.enclosure_bottom_wall_standoff_height
    #     + spec.pcb_thickness
    #     + spec.pcb_travel_z
    # )
    for x_corner, y_corner in product((1, -1), (1, -1)):
        for x_small_multiplier, y_small_multiplier in ((1, 0), (0, 1)):
            p += bd.Pos(
                x_corner
                * (
                    spec.pcb_length_x / 2
                    - spec.spring_post_margin_from_pcb_edge * x_small_multiplier
                ),
                y_corner
                * (
                    spec.pcb_length_y / 2
                    - spec.spring_post_margin_from_pcb_edge * y_small_multiplier
                ),
                spring_post_y_max,
            ) * bd.Cylinder(
                radius=spec.spring_post_od / 2,
                height=spec.spring_post_length,
                align=bde.align.ANCHOR_TOP,
            )

        # Add a larger post/stopper right in the corner.
        p += bd.Pos(
            x_corner * (spec.pcb_length_x / 2 + 2.0),
            y_corner * (spec.pcb_length_y / 2 + 2.0),
            spring_post_y_max,
        ) * bd.Cylinder(
            radius=7.0 / 2,
            height=spec.spring_post_length,
            align=bde.align.ANCHOR_TOP,
        )

    return p


def make_enclosure_bottom(spec: Spec) -> bd.Part | bd.Compound:
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

    # Remove the `joiner_bolt` holes (sturdy top-to-bottom connections) in corners.
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

    # Add the stand offs from the bottom wall.
    for x_val, y_val in spec.get_pcb_raiser_screw_coordinates():
        p += bd.Pos(x_val, y_val, spec.enclosure_wall_thickness_bottom) * bd.Cylinder(
            radius=spec.enclosure_bottom_wall_standoff_od / 2,
            height=spec.enclosure_bottom_wall_standoff_height,
            align=bde.align.ANCHOR_BOTTOM,
        )

    # Remove the holes for the `pcb_raiser_screw`.
    for x_val, y_val in spec.get_pcb_raiser_screw_coordinates():
        p -= bd.Pos(x_val, y_val, 0) * bd.Cylinder(
            radius=spec.pcb_raiser_screw_diameter / 2,
            height=spec.enclosure_total_z,
            align=bde.align.ANCHOR_BOTTOM,
        )

    # Add the cam interfaces.
    # Set this height such that, at the chosen cam state, it is statically determinate
    # against the standoffs on the bottom.
    cam_interface_z_min = spec.enclosure_wall_thickness_bottom
    cam_interface_height = (
        spec.enclosure_bottom_wall_standoff_height
        + spec.pcb_thickness
        + spec.cam_motor_od / 2
        - spec.cam_max_od / 2  # TODO(KS): Not sure what to pick. Min or max for sure.
    )
    for x_sign, y_sign in product((1, -1), (1, -1)):
        p += bd.Pos(
            x_sign * spec.cam_interface_x_sep / 2,
            y_sign * spec.cam_interface_y_sep / 2,
            cam_interface_z_min,
        ) * bd.Box(
            spec.cam_interface_width_x,
            spec.cam_interface_width_y,
            cam_interface_height,
            align=bde.align.ANCHOR_BOTTOM,
        )

    # Remove giant debugging hole.
    if spec.bottom_debugging_hole_od > 0.2:  # noqa: PLR2004
        p -= bd.Cylinder(
            radius=spec.bottom_debugging_hole_od / 2,
            height=10,
        )

    return p


def preview_both_enclosure_parts(spec: Spec) -> bd.Part | bd.Compound:
    """Preview both the top and bottom enclosure parts."""
    p = bd.Part(None)

    enclosure_top = make_enclosure_top(spec).translate((0, 0, 3))
    enclosure_bottom = make_enclosure_bottom(spec).translate((0, 0, -13))

    # Debugging: Add a breakpoint on the next line.
    p += enclosure_top
    p += enclosure_bottom

    return p


if __name__ == "__main__":
    start_time = datetime.now(UTC)
    py_file_name = Path(__file__).name
    logger.info(f"Running {py_file_name}")

    parts = {
        "preview_both_enclosure_parts": show(preview_both_enclosure_parts(Spec())),
        "enclosure_top": (make_enclosure_top(Spec())),
        "enclosure_bottom": (make_enclosure_bottom(Spec())),
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
