"""Housing which can work with any column rod design.

## Spec for Column Rod Interface:
    * An interesting part of the rod is inserted into a housing, and the
        interface bearing rod ends are added.
    * The column rod must be in the Z-axis.
    * The rod's positioning does not matter. It will be centered by its bounding box.
    * The rod will be rotated such that max-Z is at the back (max-Y).
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
from bd_warehouse.gear import SpurGear
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


def make_generic_rod_example(
    max_od: float, min_od: float, length: float
) -> tuple[bd.Part, GenericRodProperties]:
    """Make a generic rod part for the assembly."""
    p = bd.Part(None)

    segment_length = length / 4

    # Prepare to remove a hole in the side.
    remove_cylinder = bd.Pos(Z=length / 2) * bd.Cylinder(
        radius=0.6 / 2,
        height=max_od * 2,
        rotation=(90, 0, 0),
    )

    p += bd.Cone(
        bottom_radius=min_od / 2,
        top_radius=max_od / 2,
        height=segment_length,
        align=bde.align.ANCHOR_BOTTOM,
    )
    p += (
        bd.Pos(Z=segment_length)
        * bd.Cylinder(
            radius=max_od / 2,
            height=segment_length,
            align=bde.align.ANCHOR_BOTTOM,
        )
        - remove_cylinder
    )
    p += bd.Pos(Z=2 * segment_length) * bd.Cone(
        bottom_radius=max_od / 2,
        top_radius=min_od / 2,
        height=segment_length,
        align=bde.align.ANCHOR_BOTTOM,
    )
    p += bd.Pos(Z=3 * segment_length) * bd.Cone(
        bottom_radius=min_od / 2,
        top_radius=max_od / 2,
        height=segment_length,
        align=bde.align.ANCHOR_BOTTOM,
    )

    # Remove a random hole in the side in its center.
    # Doesn't work well with the render, but that's fine.

    prop = GenericRodProperties(
        min_od=min_od,
        max_od=max_od,
        od_top_end=max_od,
        od_bottom_end=min_od,
        length=length,
    )
    return (p, prop)


@dataclass(kw_only=True)
class HousingSpec:
    """Specification for braille cell housing."""

    rod_part: bd.Part
    rod_props: GenericRodProperties

    # Important settings.
    rod_center_z_from_bottom: float = 3.05  # 4.7mm motor OD/2 + 0.7mm motor leg length

    # Amount of slop in the radial direction. Radial clearance is half of this.
    rod_slop_diameter: float = 0.32

    # Amount of slop in axial direction. Clearance is half of this.
    rod_slop_axial: float = 0.6

    # Distance along the rod which is at the rod_interface_min_od.
    rod_interface_min_od_length: float = 1.2
    rod_interface_min_od: float = 1.4
    # Rod extension from the loaded rod to the outer wall.
    rod_extension_od: float = 2.2

    # ##### Resume less-important settings (common mostly common across designs).

    dot_pitch_x: float = 2.5
    dot_pitch_y: float = 2.5
    cell_pitch_x: float = 6
    cell_pitch_y: float = 10

    # Diameter of dot holes on the top.
    dot_hole_id: float = 1.3

    rod_pitch_x: float = 2.5  # Very closely related to the cell pitch.
    dist_rod_max_od_to_top_face: float = 0.5

    # JLC: Wall thickness>1.2mm, thinnest part≥0.8mm, hole size≥1.5mm.
    top_face_thickness: float = 3.0  # Rather thick to hold dots vertical.
    left_right_wall_thickness: float = 1.2
    front_back_wall_thickness: float = 2.5

    # Distance from outer dots to mounting holes. PCB property.
    x_dist_dots_to_mounting_holes: float = 5.0
    mounting_hole_spacing_y: float = 3
    mounting_hole_diameter: float = 2  # Thread-forming screws from bottom.
    margin_around_mounting_holes: float = 5.0  # Sets box size. Radius.

    cell_count_x: int = 4

    rod_length_past_outside_wall_to_gear_or_shaft: float = 0.5
    gear_module_number: float = 0.2
    gear_length: float = 0  # Disabled.
    gear_tooth_count: int = 12
    gear_pressure_angle: float = 14.5  # Controls tooth length.

    motor_shaft_hole_id_outer: float = 0.7
    motor_shaft_hole_id_inner: float = 0.45
    motor_shaft_hole_depth: float = 2  # 1.4mm nominally.
    motor_shaft_grip_length: float = 3.5  # Fits magnet within.
    motor_shaft_grip_od: float = 3.2

    zeroing_magnet_od: float = 2.0
    zeroing_magnet_height: float = 0.7  # Sink it a bit more.

    def __post_init__(self) -> None:
        """Post initialization checks."""
        magnet_coord_y = (
            self.total_y / 2
            + self.rod_slop_axial / 2
            + self.rod_length_past_outside_wall_to_gear_or_shaft
            + self.motor_shaft_grip_length / 2
        )

        motor_shaft_gripper_max_y = (
            self.total_y / 2
            + self.rod_slop_axial / 2
            + self.rod_length_past_outside_wall_to_gear_or_shaft
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


def make_complete_rod(
    spec: HousingSpec,
    draw_gear_mode: Literal["top", "bottom", "both"] | None = "top",
) -> bd.Part:
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

    # Add on the rod extensions.
    # TODO(KilowattSynthesis): Could use Cone to grow it out to the inner holder plate.

    # Make a list of the rod segments (diameter, z-val).
    rod_segments_list: list[tuple[float, float, Literal["length", "z"]]] = [
        # Extension from rod to rod_interface_min_od part, minus the slop.
        (
            spec.rod_extension_od,
            spec.total_y / 2
            - spec.front_back_wall_thickness / 2
            - spec.rod_interface_min_od_length / 2
            - spec.rod_slop_axial / 2,
            "z",
        ),
        # Rod_interface_min_od part.
        (
            spec.rod_interface_min_od,
            spec.rod_interface_min_od_length + spec.rod_slop_axial,
            "length",
        ),
        # Extension from rod to outer wall, plus the slop.
        (
            spec.rod_extension_od,
            spec.total_y / 2 + spec.rod_slop_axial / 2,
            "z",
        ),
    ]
    for final_rot_value in (0, 180):
        # Draw everything as though it's for the top (+Z).
        cur_bottom_z = spec.rod_props.length / 2
        for rod_segment in rod_segments_list:
            if rod_segment[2] == "z":
                segment_length = rod_segment[1] - cur_bottom_z
                assert segment_length > 0
            elif rod_segment[2] == "length":
                segment_length = rod_segment[1]

            p += (
                bd.Pos(Z=cur_bottom_z)
                * bd.Cylinder(
                    radius=rod_segment[0] / 2,
                    height=segment_length,
                    align=bde.align.ANCHOR_BOTTOM,
                )
            ).rotate(axis=bd.Axis.Y, angle=final_rot_value)

            cur_bottom_z += segment_length

    # Add on the driving gears.
    rot_vals = {
        "top": (0,),
        "bottom": (180,),
        "both": (0, 180),
        None: (),
    }[draw_gear_mode]
    for final_rot_value in rot_vals:
        # Past outside_wall, toward the gear.
        p += (
            bd.Pos(Z=cur_bottom_z)
            * bd.Cylinder(
                radius=spec.rod_extension_od / 2,
                height=spec.rod_length_past_outside_wall_to_gear_or_shaft,
                align=bde.align.ANCHOR_BOTTOM,
            )
        ).rotate(axis=bd.Axis.Y, angle=final_rot_value)

        # Add the gear.
        if spec.gear_length > 0:
            p += (
                bd.Pos(
                    Z=cur_bottom_z + spec.rod_length_past_outside_wall_to_gear_or_shaft
                )
                * SpurGear(
                    module=spec.gear_module_number,
                    tooth_count=spec.gear_tooth_count,
                    thickness=spec.gear_length,
                    pressure_angle=spec.gear_pressure_angle,
                    root_fillet=0.001,  # Rounding at base of each tooth.
                    align=bde.align.ANCHOR_BOTTOM,
                )
            ).rotate(axis=bd.Axis.Y, angle=final_rot_value)

        # Add the motor shaft gripper and hole.
        p += (
            bd.Pos(Z=cur_bottom_z + spec.rod_length_past_outside_wall_to_gear_or_shaft)
            * bd.Cylinder(
                radius=spec.motor_shaft_grip_od / 2,
                height=spec.motor_shaft_grip_length,
                align=bde.align.ANCHOR_BOTTOM,
            )
        ).rotate(axis=bd.Axis.Y, angle=final_rot_value)
        p -= (
            bd.Pos(
                Z=cur_bottom_z
                + spec.rod_length_past_outside_wall_to_gear_or_shaft
                + (spec.motor_shaft_grip_length - spec.motor_shaft_hole_depth)
            )
            * bd.Cone(
                top_radius=spec.motor_shaft_hole_id_outer / 2,
                bottom_radius=spec.motor_shaft_hole_id_inner / 2,
                height=spec.motor_shaft_hole_depth,
                align=bde.align.ANCHOR_BOTTOM,
            )
        ).rotate(axis=bd.Axis.Y, angle=final_rot_value)
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
                    (
                        cur_bottom_z
                        + spec.rod_length_past_outside_wall_to_gear_or_shaft
                        + spec.motor_shaft_grip_length / 2
                    ),
                )
            )
            .rotate(axis=bd.Axis.Y, angle=final_rot_value)
        )

    return p


def make_housing(
    spec: HousingSpec,
    *,
    enable_add_rods: bool = False,
    # enable_print_in_place: bool = False,
) -> bd.Part:
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
        spec.total_y - 2 * spec.front_back_wall_thickness,
        spec.total_z - spec.top_face_thickness,
        align=bde.align.ANCHOR_BOTTOM,
    )

    # Remove the mounting holes.
    for x_val, y_val in product(
        bde.evenly_space_with_center(
            count=2,
            spacing=spec.mounting_hole_spacing_x,
        ),
        bde.evenly_space_with_center(
            count=3,
            spacing=spec.mounting_hole_spacing_y,
        ),
    ):
        p -= bd.Pos(x_val, y_val) * bd.Cylinder(
            radius=spec.mounting_hole_diameter / 2,
            height=spec.total_z,
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
            height=spec.total_z,
            align=bde.align.ANCHOR_BOTTOM,
        )

    final_rod_part_top_gear = make_complete_rod(spec, draw_gear_mode="top")
    final_rod_part_bottom_gear = make_complete_rod(spec, draw_gear_mode="bottom")

    # For each `rod_x`.
    # Remove the rod holes.
    for cell_x, rod_offset_x, final_rot_val in product(
        spec.get_cell_center_x_values(),
        bde.evenly_space_with_center(count=2, spacing=spec.rod_pitch_x),
        (0, 180),
    ):
        rod_x = cell_x + rod_offset_x

        # Remove rod spot from inside of wall to rod_interface_min_od.
        length_of_each_rod_extension_space = (
            spec.front_back_wall_thickness - spec.rod_interface_min_od_length
        ) / 2
        p -= (
            bd.Pos(
                X=rod_x,
                Y=spec.total_y / 2 - spec.front_back_wall_thickness,
                Z=spec.rod_center_z_from_bottom,
            )
            * bd.Cylinder(
                radius=spec.rod_extension_od / 2 + spec.rod_slop_diameter / 2,
                height=length_of_each_rod_extension_space,
                align=bde.align.ANCHOR_BOTTOM,
            ).rotate(axis=bd.Axis.X, angle=-90)
        ).rotate(axis=bd.Axis.Z, angle=final_rot_val)

        # Remove `rod_interface_min_od` part.
        # Lazy: Remove it all the way through.
        p -= (
            bd.Pos(X=rod_x, Z=spec.rod_center_z_from_bottom)
            * bd.Cylinder(
                radius=spec.rod_interface_min_od / 2 + spec.rod_slop_diameter / 2,
                height=spec.total_y,
                rotation=(90, 0, 0),
            )
        ).rotate(axis=bd.Axis.Z, angle=final_rot_val)

        # Remove rod spot from rod_interface_min_od to outside wall.
        p -= (
            bd.Pos(
                X=rod_x,
                Y=spec.total_y / 2 - length_of_each_rod_extension_space,
                Z=spec.rod_center_z_from_bottom,
            )
            * bd.Cylinder(
                radius=spec.rod_extension_od / 2 + spec.rod_slop_diameter / 2,
                height=length_of_each_rod_extension_space,
                align=bde.align.ANCHOR_BOTTOM,
            ).rotate(axis=bd.Axis.X, angle=-90)
        ).rotate(axis=bd.Axis.Z, angle=final_rot_val)

    # Add the rods.
    if enable_add_rods:
        for rod_num, (cell_x, rod_offset_x) in enumerate(
            product(
                spec.get_cell_center_x_values(),
                bde.evenly_space_with_center(count=2, spacing=spec.rod_pitch_x),
            )
        ):
            rod_x = cell_x + rod_offset_x
            rod_part = (
                final_rod_part_top_gear
                if rod_num % 2 == 0
                else final_rod_part_bottom_gear
            )
            p += bd.Pos(X=rod_x, Z=spec.rod_center_z_from_bottom) * rod_part.rotate(
                axis=bd.Axis.X,
                # -90 puts the top-side of the rod at the back.
                angle=-90,
            )

    return p


def make_octagon_cam_housing_in_place() -> bd.Part:
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

    housing_part = make_housing(
        housing_spec,
        enable_add_rods=True,
    )

    return housing_part


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


if __name__ == "__main__":
    start_time = datetime.now(UTC)
    py_file_name = Path(__file__).name
    logger.info(f"Running {py_file_name}")

    generic_rod_1_part, generic_rod_1_props = make_generic_rod_example(
        max_od=2.4,
        min_od=1.5,
        length=6,
    )
    logger.info(f"Generic Rod 1 Properties: {generic_rod_1_props}")
    housing_spec_1 = HousingSpec(
        rod_part=generic_rod_1_part,
        rod_props=generic_rod_1_props,
    )

    print_pcb_column_x_coords(housing_spec_1)

    parts = {
        "generic_rod": show(generic_rod_1_part),
        "housing": show(make_housing(housing_spec_1)),
        "complete_rod": show(make_complete_rod(housing_spec_1)),
        "housing_with_rods": show(
            make_housing(
                housing_spec_1,
                enable_add_rods=True,
            )
        ),
        "octagon_cam_housing_print_in_place": show(make_octagon_cam_housing_in_place()),
    }

    housing_height = (
        parts["housing"].bounding_box().max.Z - parts["housing"].bounding_box().min.Z
    )
    clip_at_z = -housing_height / 2 + housing_spec_1.rod_center_z_from_bottom
    logger.info(f"Clip at Z={clip_at_z:.3f}")

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
