"""Housing which can work with any column rod design.

## Spec for Column Rod Interface:
    * An interesting part of the rod is inserted into a housing, and the
        interface bearing rod ends are added.
    * The column rod must be in the Z-axis.
    * The rod's positioning does not matter. It will be centered by its bounding box.
    * The rod will be rotated such that max-Z is at the back (max-Y).

## Order Variant Notes
* W202502241729529 -> motor_shaft_grip_length = 3.2
* W202502241736554 -> motor_shaft_grip_length = 3.7
"""

import copy
import json
import sys
from dataclasses import dataclass
from datetime import UTC, datetime
from itertools import product
from pathlib import Path
from typing import Literal

import build123d as bd
import build123d_ease as bde
import git
from build123d_ease import show
from loguru import logger

# Prepare for imports from the parent directory.
sys.path.insert(0, str(Path(__file__).parent.parent.absolute()))
from cad import dot_column_cam_rod_octagon


@dataclass(kw_only=True)
class GenericRodProperties:
    """Properties of a generic rod part being inserted into the assembly.

    Used to gather information about the ball screw rod, cam rods, etc.
    """

    min_od: float
    max_od: float
    od_top_end: float  # Top end becomes the back.
    od_bottom_end: float  # Bottom end becomes the front.

    # Length of the "interesting" part of the rod.
    length: float

    # TODO(KilowattSynthesis): Add property about holes in the ends, maybe.

    def __post_init__(self) -> None:
        """Post initialization checks."""
        if self.min_od > self.max_od:
            msg = "min_od must be less than max_od."
            raise ValueError(msg)

        if not (self.min_od <= self.od_top_end <= self.max_od):
            msg = "od_top_end must be between min_od and max_od."
            raise ValueError(msg)
        if not (self.min_od <= self.od_bottom_end <= self.max_od):
            msg = "od_bottom_end must be between min_od and max_od."
            raise ValueError(msg)

    def deep_copy(self) -> "GenericRodProperties":
        """Copy the current properties."""
        return copy.deepcopy(self)

    def __str__(self) -> str:
        """Return string representation of the properties, as JSON."""
        return json.dumps(self.__dict__, indent=2)

    # TODO(KilowattSynthesis): Add a method for validating that a part conforms.


@dataclass(kw_only=True)
class HousingSpec:
    """Specification for braille cell housing."""

    rod_part: bd.Part | bd.Compound
    rod_props: GenericRodProperties

    # Important settings.
    # Theory: 4.7mm motor OD/2 + 0.7mm motor leg length
    # Measured: About 3.15 to 3.2mm.
    rod_center_z_from_bottom: float = 3.15

    # Amount of slop in the radial direction. Radial clearance is half of this.
    rod_slop_diameter: float = 0.32

    # Amount of slop in axial direction. Clearance is half of this.
    rod_slop_axial: float = 0.6

    # Rod extension from the fancy rod edge to the outer wall.
    rod_extension_od: float = 2.2

    # Sticks through the wall on the non-motor side.
    rod_od_through_wall_pivot_side: float = 1.3
    rod_length_into_wall_pivot_side: float = 1.5  # Less than WT.

    rod_length_past_outside_wall_to_motor_coupler: float = 0.4

    # ##### Resume less-important settings (common mostly common across designs).

    dot_pitch_x: float = 2.5
    dot_pitch_y: float = 2.5
    cell_pitch_x: float = 6
    cell_pitch_y: float = 10

    # Diameter of dot holes on the top.
    dot_hole_id: float = 1.3

    rod_pitch_x: float = 2.5  # Very closely related to the dot pitch.
    dist_rod_max_od_to_top_face: float = 1.5

    # JLC: Wall thickness>1.2mm, thinnest part≥0.8mm, hole size≥1.5mm.
    top_face_thickness: float = 3
    left_right_wall_thickness: float = 1.2
    front_back_wall_thickness: float = 2.0

    # Distance from outer dots to mounting holes. PCB property.
    x_dist_dots_to_mounting_holes: float = 5.0
    mounting_hole_spacing_y: float = 3
    mounting_hole_id: float = 1.85  # Thread-forming screws from bottom.
    mounting_hole_peg_od: float = 2
    mounting_hole_peg_length: float = 1.5
    margin_around_mounting_holes: float = 5.0  # Sets box size. Radius.

    cell_count_x: int = 4

    motor_shaft_hole_id_outer: float = 0.65
    motor_shaft_hole_id_inner: float = 0.4
    motor_shaft_hole_depth: float = 2  # 1.4mm nominally.
    motor_shaft_grip_length: float = 3.7  # Fits magnet within.
    motor_shaft_grip_od: float = 3.2

    zeroing_magnet_od: float = 2.0
    zeroing_magnet_height: float = 0.7  # Sink it a bit more.

    def __post_init__(self) -> None:
        """Post initialization checks."""
        magnet_coord_y = (
            self.total_y / 2
            + self.rod_slop_axial / 2
            + self.rod_length_past_outside_wall_to_motor_coupler
            + self.motor_shaft_grip_length / 2
        )

        motor_shaft_gripper_max_y = (
            self.total_y / 2
            + self.rod_slop_axial / 2
            + self.rod_length_past_outside_wall_to_motor_coupler
            + self.motor_shaft_grip_length
        )

        data = {
            "mounting_hole_spacing_x": self.mounting_hole_spacing_x,
            "inner_cavity_size_x": self.inner_cavity_size_x,
            "total_x": self.total_x,
            "total_y": self.total_y,
            "total_z": self.total_z,
            "get_cell_center_x_values()": self.get_cell_center_x_values(),
            "magnet_coord_y": magnet_coord_y,  # PCB design.
            "motor_shaft_gripper_max_y": motor_shaft_gripper_max_y,  # PCB design.
        }
        logger.info(json.dumps(data, indent=2))

        # TODO(KilowattSynthesis): Validate the slop diameters.

    def deep_copy(self) -> "HousingSpec":
        """Copy the current spec."""
        return copy.deepcopy(self)

    @property
    def inner_cavity_size_x(self) -> float:
        """Size of the inner cavity in the X direction."""
        return (
            self.cell_pitch_x * (self.cell_count_x - 1)
            + self.dot_pitch_x
            + (self.x_dist_dots_to_mounting_holes * 2)
            - 3.5  # TODO(KilowattSynthesis): Tweak this.
        )

    @property
    def inner_cavity_size_y(self) -> float:
        """Size of the inner cavity in the Y direction."""
        return self.total_y - 2 * self.front_back_wall_thickness

    @property
    def total_x(self) -> float:
        """Total X width of the housing."""
        return (
            (self.cell_pitch_x * (self.cell_count_x - 1))
            + (self.dot_pitch_x)
            + (self.x_dist_dots_to_mounting_holes * 2)
            + (self.margin_around_mounting_holes * 2)
            + (2 * self.left_right_wall_thickness)
        )

    @property
    def total_y(self) -> float:
        """Total Y width of the housing."""
        return (
            self.front_back_wall_thickness * 2
            + self.rod_props.length
            + self.rod_slop_axial
            + 1  # Just a tad extra.
        )

    @property
    def total_z(self) -> float:
        """Total Z height of the housing."""
        return (
            self.rod_center_z_from_bottom
            + self.rod_props.max_od / 2
            + self.dist_rod_max_od_to_top_face
            + self.top_face_thickness
        )

    @property
    def mounting_hole_spacing_x(self) -> float:
        """Spacing between the mounting holes, in X axis."""
        return (
            self.x_dist_dots_to_mounting_holes * 2
            + self.cell_pitch_x * (self.cell_count_x - 1)
            + self.dot_pitch_x
        )

    def get_cell_center_x_values(self) -> list[float]:
        """Get the X coordinate of the center of each cell."""
        return bde.evenly_space_with_center(
            count=self.cell_count_x,
            spacing=self.cell_pitch_x,
        )


def make_octagon_rod(
    spec: HousingSpec,
    motor_side: Literal["+Z", "-Z"] = "+Z",
) -> bd.Part | bd.Compound:
    """Make the complete rod, with the holder plates and extra length.

    Output remains in the Z axis.
    """
    p = bd.Part(None)

    # Center the rod.
    p += (
        bd.Pos(
            (spec.rod_part.bounding_box().max.X + spec.rod_part.bounding_box().min.X)
            / -2,
            (spec.rod_part.bounding_box().max.Y + spec.rod_part.bounding_box().min.Y)
            / -2,
            (spec.rod_part.bounding_box().max.Z + spec.rod_part.bounding_box().min.Z)
            / -2,
        )
        * spec.rod_part
    )

    if motor_side == "-Z":
        # Rotate the cam rod 180, then rotate back at the end.
        p = p.rotate(bd.Axis.Y, angle=180)

    # Add on the rod extensions.
    # TODO(KilowattSynthesis): Could use Cone to grow it out to the inner holder plate.

    # -Y = -Z, extension from rod to wall, minus the slop.
    p += bd.Cylinder(
        radius=spec.rod_extension_od / 2,
        height=(
            spec.inner_cavity_size_y / 2
            - spec.rod_slop_axial
            - spec.rod_props.length / 2
        ),
        align=bde.align.ANCHOR_TOP,
    ).translate((0, 0, -spec.rod_props.length / 2))

    # -Y = -Z, extension into and beyond the wall.
    p += bd.Cylinder(
        radius=spec.rod_od_through_wall_pivot_side / 2,
        height=spec.rod_length_into_wall_pivot_side,
        align=bde.align.ANCHOR_TOP,
    ).translate((0, 0, p.bounding_box().min.Z))

    # +Y = +Z, extension into the wall and beyond, toward motor.
    p += bd.Cylinder(
        radius=spec.rod_extension_od / 2,
        height=(spec.total_y / 2 - spec.rod_props.length / 2),
        align=bde.align.ANCHOR_BOTTOM,
    ).translate((0, 0, spec.rod_props.length / 2))

    # Add on the driving motor interface.
    # +Z - Past outside_wall, toward the motor interface.
    p += bd.Cylinder(
        radius=spec.rod_extension_od / 2,
        height=spec.rod_length_past_outside_wall_to_motor_coupler,
        align=bde.align.ANCHOR_BOTTOM,
    ).translate((0, 0, p.bounding_box().max.Z))

    # Add the motor shaft coupler and hole.
    p += bd.Cylinder(
        radius=spec.motor_shaft_grip_od / 2,
        height=spec.motor_shaft_grip_length,
        align=bde.align.ANCHOR_BOTTOM,
    ).translate((0, 0, p.bounding_box().max.Z))

    # Remove the motor shaft hole from the motor coupler.
    p -= (
        bd.Cone(
            top_radius=spec.motor_shaft_hole_id_outer / 2,
            bottom_radius=spec.motor_shaft_hole_id_inner / 2,
            height=spec.motor_shaft_hole_depth,
            align=bde.align.ANCHOR_TOP,
        )
        # Make it a D-shaft.
        & bd.Box(5, 5, 5, align=bde.align.ANCHOR_LEFT).translate((-0.15, 0, 0))
    ).translate((0, 0, p.bounding_box().max.Z))

    # Remove a hole for the the zeroing magnet on each rod.
    p -= (
        bd.Cylinder(
            radius=spec.zeroing_magnet_od / 2,
            height=spec.zeroing_magnet_height * 2,
        )
        .rotate(bd.Axis.X, angle=90)
        .translate(
            (
                0,
                -spec.motor_shaft_grip_od / 2,
                (p.bounding_box().max.Z - spec.motor_shaft_grip_length / 2),
            )
        )
    )

    if motor_side == "-Z":
        p = p.rotate(bd.Axis.Y, angle=180)

    return p


def make_housing(spec: HousingSpec) -> bd.Part | bd.Compound:
    """Make the housing that the screw fits into.

    Args:
        spec: The specification for the housing.
        enable_add_rods: Whether to add all the rods.
        enable_print_in_place: Whether to print the screws in place.

    """
    p = bd.Part(None)

    # Create the main outer shell.
    p += bd.Box(
        spec.total_x,
        spec.total_y,
        spec.total_z,
        align=bde.align.ANCHOR_BOTTOM,
    )

    # Remove the box inside the walls.
    p -= bd.Box(
        spec.inner_cavity_size_x,
        spec.inner_cavity_size_y,
        spec.total_z - spec.top_face_thickness,
        align=bde.align.ANCHOR_BOTTOM,
    )

    # Remove the mounting holes (center holes).
    for x_val in bde.evenly_space_with_center(
        count=2,
        spacing=spec.mounting_hole_spacing_x,
    ):
        p -= bd.Pos(X=x_val) * bd.Cylinder(
            radius=spec.mounting_hole_id / 2,
            height=spec.total_z,
            align=bde.align.ANCHOR_BOTTOM,
        )
    # Add the mounting pegs (corner holes).
    for x_val, y_val in product(
        bde.evenly_space_with_center(
            count=2,
            spacing=spec.mounting_hole_spacing_x,
        ),
        bde.evenly_space_with_center(
            count=2,
            spacing=spec.mounting_hole_spacing_y * 2,
        ),
    ):
        p += bd.Pos(x_val, y_val) * bd.Cylinder(
            radius=spec.mounting_hole_peg_od / 2,
            height=spec.mounting_hole_peg_length,
            align=bde.align.ANCHOR_TOP,
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
            height=spec.total_z,
            align=bde.align.ANCHOR_BOTTOM,
        )

    # For each `rod_x`.
    # Remove the rod holes.
    for cell_x, z_rot_val in product(
        spec.get_cell_center_x_values(),
        (0, 180),
    ):
        rod_x = cell_x

        # Draw everything in here as the very left-most rod, extending in the +Y, with
        # its motor in the very very top-left. Then, it gets rotated about the center of
        # the cell.
        # Draw it at its final Y/Z position, but assuming that the rod is at X=0.

        p_neg_rod = bd.Part(None)
        # -Y, stick into the wall part way.
        # Removed because it's not needed and makes very thin walls.
        # p_neg_rod += (
        #     bd.Cylinder(
        #         radius=spec.rod_extension_od / 2 + spec.rod_slop_diameter / 2,
        #         height=(
        #             spec.inner_cavity_size_y / 2
        #             + spec.rod_dist_into_wall
        #             + spec.rod_slop_axial
        #         ),
        #         align=bde.align.ANCHOR_BOTTOM,
        #     )
        #     .rotate(axis=bd.Axis.X, angle=90)
        #     .translate((0, 0, spec.rod_center_z_from_bottom))
        # )

        # -Y, stick through the wall, small diameter.
        p_neg_rod += (
            bd.Cylinder(
                radius=spec.rod_od_through_wall_pivot_side / 2
                + spec.rod_slop_diameter / 2,
                height=(spec.inner_cavity_size_y),
                align=bde.align.ANCHOR_BOTTOM,
            )
            .rotate(axis=bd.Axis.X, angle=90)
            .translate((0, 0, spec.rod_center_z_from_bottom))
        )

        # +Y, stick through the wall, insertion point for entire rod.
        # This side is where the magnet+motor is.
        p_neg_rod += (
            bd.Cylinder(
                radius=spec.rod_extension_od / 2 + spec.rod_slop_diameter / 2,
                height=(spec.inner_cavity_size_y),
                align=bde.align.ANCHOR_BOTTOM,
            )
            .rotate(axis=bd.Axis.X, angle=-90)
            .translate((0, 0, spec.rod_center_z_from_bottom))
        )

        # Do the rotation and X move.
        p -= (
            p_neg_rod.translate(((-spec.rod_pitch_x / 2), 0, 0))
            .rotate(axis=bd.Axis.Z, angle=z_rot_val)
            .translate((rod_x, 0, 0))
        )

    return p


def make_final_octagon_cam_rod(
    motor_side: Literal["+Z", "-Z"] = "+Z",
) -> bd.Part | bd.Compound:
    """Make the octagon cam rod."""
    cam_spec = dot_column_cam_rod_octagon.MainSpec()

    rod_part = dot_column_cam_rod_octagon.make_cam_rod(cam_spec)

    housing_spec = HousingSpec(
        rod_part=rod_part,
        rod_props=GenericRodProperties(
            min_od=cam_spec.cam_rod_diameter_major,
            max_od=cam_spec.cam_rod_diameter_major,
            od_top_end=cam_spec.cam_rod_diameter_major,
            od_bottom_end=cam_spec.cam_rod_diameter_major,
            length=cam_spec.cam_rod_length,
        ),
    )

    full_rod_part = make_octagon_rod(housing_spec, motor_side=motor_side)

    return full_rod_part


def make_octagon_cam_housing_no_rods() -> bd.Part | bd.Compound:
    """Make the octagon cam housing in place."""
    # TODO(KilowattSynthesis): Could add arg - housing_spec_overrides: dict[str, Any]
    cam_spec = dot_column_cam_rod_octagon.MainSpec()

    rod_part = dot_column_cam_rod_octagon.make_cam_rod(cam_spec)

    housing_spec = HousingSpec(
        rod_part=rod_part,
        rod_props=GenericRodProperties(
            min_od=cam_spec.cam_rod_diameter_major,
            max_od=cam_spec.cam_rod_diameter_major,
            od_top_end=cam_spec.cam_rod_diameter_major,
            od_bottom_end=cam_spec.cam_rod_diameter_major,
            length=cam_spec.cam_rod_length,
        ),
    )

    print_pcb_column_x_coords(housing_spec)

    housing_part = make_housing(housing_spec)

    return housing_part


def make_octagon_cam_housing_assembly_preview() -> bd.Part | bd.Compound:
    """Make a preview of the housing assembly, with the rods installed."""
    rod_part_pos_z = make_final_octagon_cam_rod("+Z")
    rod_part_neg_z = make_final_octagon_cam_rod("-Z")

    cam_spec = dot_column_cam_rod_octagon.MainSpec()
    housing_spec = HousingSpec(
        rod_part=dot_column_cam_rod_octagon.make_cam_rod(cam_spec),
        rod_props=GenericRodProperties(
            min_od=cam_spec.cam_rod_diameter_major,
            max_od=cam_spec.cam_rod_diameter_major,
            od_top_end=cam_spec.cam_rod_diameter_major,
            od_bottom_end=cam_spec.cam_rod_diameter_major,
            length=cam_spec.cam_rod_length,
        ),
    )

    p = bd.Part(None)
    p += make_octagon_cam_housing_no_rods()

    # Add the rods.
    for rod_num, (cell_x, rod_offset_x) in enumerate(
        product(
            housing_spec.get_cell_center_x_values(),
            bde.evenly_space_with_center(count=2, spacing=housing_spec.rod_pitch_x),
        )
    ):
        rod_x = cell_x + rod_offset_x
        rod_part = rod_part_pos_z if rod_num % 2 == 0 else rod_part_neg_z
        p += bd.Pos(X=rod_x, Z=housing_spec.rod_center_z_from_bottom) * rod_part.rotate(
            axis=bd.Axis.X,
            # -90 puts the top-side of the rod at the back.
            angle=-90,
        )

    return p


def print_pcb_column_x_coords(
    spec: HousingSpec,
    *,
    pcb_housing_center_x: float = 110.25,
    pcb_housing_center_y: float = 125,
) -> None:
    """Print the X coordinates of the column cells."""
    _pcb_housing_center_y = pcb_housing_center_y

    cell_center_x_values = spec.get_cell_center_x_values()

    back_side_x_vals: list[float] = []  # Towards +Y.
    front_side_x_vals: list[float] = []  # Towards -Y.

    for cell_x in cell_center_x_values:
        for rod_x_sign in (-1, 1):
            rod_x = pcb_housing_center_x + cell_x + (rod_x_sign * spec.rod_pitch_x / 2)
            if rod_x_sign == -1:
                back_side_x_vals.append(rod_x)
            elif rod_x_sign == 1:
                front_side_x_vals.append(rod_x)
            else:
                msg = "Invalid rod_x_sign."
                raise ValueError(msg)

    logger.info(f"Back Side X Vals: {back_side_x_vals}")
    logger.info(f"Front Side X Vals: {front_side_x_vals}")

    assert len(back_side_x_vals) == len(front_side_x_vals)
    assert (
        len(back_side_x_vals) + len(front_side_x_vals) == len(cell_center_x_values) * 2
    )


def preview_all() -> bd.Part | bd.Compound:
    """Quick preview of all the parts."""
    housing_part = make_octagon_cam_housing_no_rods()
    rod_part = make_final_octagon_cam_rod().translate((0, -10, 0))

    housing_and_rod_assembly = make_octagon_cam_housing_assembly_preview().translate(
        (0, 25, 0)
    )

    p = housing_part + rod_part + housing_and_rod_assembly

    if isinstance(p, list):
        p = bd.Compound(p)

    return p


def printable_cam_rods() -> bd.Part | bd.Compound:
    """Make a row of cam rods, for ordering."""
    rod_part = make_final_octagon_cam_rod()

    pitch_x = 12
    part_count = 5
    rail_parallel_length = 25

    _shift_z = 3

    p = bd.Part(None)
    for i in range(part_count):
        # Add the part itself.
        p += rod_part.translate((i * pitch_x, 0, -_shift_z))

    # Add rigid bodies parallel to the rods.
    for i in range(-1, part_count):
        p += bd.Cylinder(
            radius=3.5 / 2,
            height=rail_parallel_length,
        ).translate((i * pitch_x + pitch_x / 2, 0))

    # Join the rigid bodies.
    for side in [1, -1]:
        p += (
            bd.Cylinder(
                radius=4 / 2,
                height=pitch_x * (part_count + 1),
                align=bde.align.ANCHOR_BOTTOM,
            )
            .rotate(bd.Axis.Y, angle=90)
            .translate((-pitch_x, 0, side * rail_parallel_length / 2))
        )

    # On the motor side, connect sideways.
    p += (
        bd.Cylinder(
            radius=1.5 / 2,
            height=pitch_x * (part_count),
            align=bde.align.ANCHOR_BOTTOM,
        )
        .rotate(bd.Axis.Y, angle=90)
        .translate((-pitch_x / 2, 0, 7.75 - _shift_z))
    )

    # Add a connecting rod to the bottom.
    for i in range(part_count):
        p += bd.Cylinder(
            radius=1.5 / 2,
            height=5,
            align=bde.align.ANCHOR_TOP,
        ).translate((i * pitch_x, 0, rod_part.bounding_box().min.Z - _shift_z))

    return p


if __name__ == "__main__":
    start_time = datetime.now(UTC)
    py_file_name = Path(__file__).name
    logger.info(f"Running {py_file_name}")

    parts = {
        "printable_cam_rods": show(printable_cam_rods()),
        "octagon_cam_rod": (make_final_octagon_cam_rod()),
        "octagon_cam_housing": (make_octagon_cam_housing_no_rods()),
        "octagon_cam_housing_assembly_preview": (
            make_octagon_cam_housing_assembly_preview()
        ),
        "preview_all": (preview_all()),
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
