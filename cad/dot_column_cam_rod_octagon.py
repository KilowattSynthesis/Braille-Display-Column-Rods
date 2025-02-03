"""Cam rod for each column, rotated to a multiple of 45 degrees (8 positions).

Would require that the bed be lifted.

This design creates an octagonal prism cam rod, and each dot is a slice of that prism.
"""

import copy
import json
import math
import random
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
    """Specification for braille cell housing."""

    # dot_pitch_x: float = 2.5  # Unused.
    dot_pitch_y: float = 2.5
    cell_pitch_x: float = 6
    cell_pitch_y: float = 10

    # Settings for the render.
    render_cam_pitch_x: float = 2.9
    render_cell_count_x: int = 3

    # None means "use polygon_side_length".
    divot_dot_down_depth: float = 1.0
    divot_dot_up_depth: float = 0.1
    divot_dot_up_width_top: float | None = 0.8
    divot_dot_up_width_bottom: float = 0.6
    divot_dot_down_width_top: float | None = None
    divot_dot_down_width_bottom: float = 0.1
    divot_length_axial: float = 1.5

    # Must be less than dot_pitch_x/render_cam_pitch_x.
    # Major diameter means that the entire rod's diameter is less than this.
    cam_rod_diameter_major: float = 2.4
    cam_rod_length: float = 7

    cam_rod_polygon_side_count: int = 8

    def __post_init__(self) -> None:
        """Post initialization checks."""
        data = {
            "polygon_major_diameter": self.cam_rod_diameter_major,
            "polygon_side_length": self.polygon_side_length,
            "polygon_minor_diameter": self.polygon_minor_diameter,
        }
        logger.info(json.dumps(data, indent=2))

    def deep_copy(self) -> "MainSpec":
        """Copy the current spec."""
        return copy.deepcopy(self)

    @property
    def polygon_minor_diameter(self) -> float:
        """Get the minor diameter of the polygon."""
        return self.cam_rod_diameter_major * math.cos(
            math.pi / self.cam_rod_polygon_side_count
        )

    @property
    def polygon_side_length(self) -> float:
        """Get the side length of the polygon."""
        return self.cam_rod_diameter_major * math.sin(
            math.pi / self.cam_rod_polygon_side_count
        )


def make_cam_rod(spec: MainSpec) -> bd.Part:
    """Make a single cam rod, pointing in Z axis."""
    p = bd.Part(None)

    p += (
        bd.extrude(
            bd.RegularPolygon(
                radius=spec.cam_rod_diameter_major / 2,
                side_count=spec.cam_rod_polygon_side_count,
            ),
            amount=spec.cam_rod_length,
        )
        .translate((0, 0, -spec.cam_rod_length / 2))
        # Rotate to align the flat faces with the cardinal directions.
        .rotate(axis=bd.Axis.Z, angle=360 / spec.cam_rod_polygon_side_count / 2)
    )

    # Remove the magnets (one on each side).
    for (rot_idx, rot_val), (z_idx, z_pos) in product(
        enumerate(range(0, 360, 360 // spec.cam_rod_polygon_side_count)),
        enumerate(bde.evenly_space_with_center(count=3, spacing=spec.dot_pitch_y)),
    ):
        # Only dig out not-extended pins.
        assert 0 <= rot_idx < spec.cam_rod_polygon_side_count
        assert 0 <= z_idx < 3  # noqa: PLR2004
        is_pin_up: bool = (rot_idx & (1 << z_idx)) > 0

        if is_pin_up:
            dot_width_top = spec.divot_dot_up_width_top
            dot_width_bottom = spec.divot_dot_up_width_bottom
            dot_depth = spec.divot_dot_up_depth
        else:
            dot_width_top = spec.divot_dot_down_width_top
            dot_width_bottom = spec.divot_dot_down_width_bottom
            dot_depth = spec.divot_dot_down_depth

        if dot_width_top is None:
            dot_width_top = spec.polygon_side_length
        if dot_width_bottom is None:
            dot_width_bottom = spec.polygon_side_length

        p -= (
            (
                bd.extrude(
                    # Draw out a polygon for the divot facing +X.
                    bd.make_face(
                        bd.Plane.XY
                        * bd.Polyline(  # type: ignore reportArgumentType
                            # 2 points at the "top" (far right, toward edge).
                            (
                                spec.polygon_minor_diameter / 2,
                                +dot_width_top / 2,
                            ),
                            (
                                spec.polygon_minor_diameter / 2,
                                -dot_width_top / 2,
                            ),
                            # 2 points at the "bottom" (toward center).
                            (
                                spec.polygon_minor_diameter / 2 - dot_depth,
                                -dot_width_bottom / 2,
                            ),
                            (
                                spec.polygon_minor_diameter / 2 - dot_depth,
                                +dot_width_bottom / 2,
                            ),
                            close=True,
                        )
                    ),
                    amount=spec.divot_length_axial,
                )
            )
            .translate(
                (
                    0,
                    0,
                    z_pos - spec.divot_length_axial / 2,
                )
            )
            .rotate(axis=bd.Axis.Z, angle=rot_val)
        )

    # Trial: Remove a screw in the middle.
    # p -= bd.Cylinder(
    #     radius=1.4 / 2,
    #     height=spec.cam_rod_length,
    # )

    return p


def make_assembly_cam_rod(spec: MainSpec) -> bd.Part:
    """Make an assembly of cam rod with magnet."""
    p = bd.Part(None)

    for cell_num, (cell_x, dot_offset_x) in enumerate(
        product(
            bde.evenly_space_with_center(count=3, spacing=spec.cell_pitch_x),
            bde.evenly_space_with_center(count=2, spacing=spec.render_cam_pitch_x),
        )
    ):
        dot_x = cell_x + dot_offset_x

        # Randomly rotate the cam rods, except for 0 and 1.
        random_rot_value = random.randint(0, 359) if cell_num not in (0, 1) else 0

        p += (
            make_cam_rod(spec)
            .rotate(axis=bd.Axis.Z, angle=random_rot_value)
            .translate((0, dot_x, 0))
        )

    return p


if __name__ == "__main__":
    start_time = datetime.now(UTC)
    py_file_name = Path(__file__).name
    logger.info(f"Running {py_file_name}")

    parts = {
        "cam_rod": show(make_cam_rod(MainSpec())),
        "several_rods": show(make_assembly_cam_rod(MainSpec())),
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
