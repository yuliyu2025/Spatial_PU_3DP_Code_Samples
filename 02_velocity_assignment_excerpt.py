"""Selected original source excerpts for academic application review.
Taken from _06_velocity.py. These are class methods, NOT standalone runnable programs.
They depend on the project PrintPath / PrintPoint data model and COMPAS.
"""

import math
import _01_config as config

class VelocityControl:

    def assign_velocity_by_movement(
        self,
        velocity_ground,
        velocity_air,
        velocity_up_min=5.5,
        velocity_up_max=11.0,
        velocity_down_min=5.0,
        velocity_down_max=10.0,
        angle_min=30.0,
        angle_max=90.0,
        vertical_tolerance=5.0,
        ground_tolerance=10.0
    ):
        """Assign velocity by movement direction and inclination."""

        first_layer_z = config.LAYER_HEIGHT
        printpoints = self.printpoints

        for current, next_pt in zip(
            printpoints,
            printpoints[1:]
        ):

            if current.point_type != "print":
                continue

            segment = next_pt.point - current.point

            if segment.length == 0:
                continue

            dz = segment.z

            # Up / down
            if abs(dz) > vertical_tolerance:

                angle = math.degrees(
                    math.asin(
                        abs(dz) / segment.length
                    )
                )

                angle = max(
                    angle_min,
                    min(angle_max, angle)
                )

                t = (
                    (angle - angle_min)
                    / (angle_max - angle_min)
                )

                if dz > 0:
                    velocity = (
                        velocity_up_min
                        + t * (
                            velocity_up_max
                            - velocity_up_min
                        )
                    )

                else:
                    velocity = (
                        velocity_down_min
                        + t * (
                            velocity_down_max
                            - velocity_down_min
                        )
                    )

            # Horizontal
            else:

                segment_z = (
                    current.point.z
                    + next_pt.point.z
                ) / 2.0

                velocity = (
                    velocity_ground
                    if segment_z
                    <= first_layer_z + ground_tolerance
                    else velocity_air
                )

            current.velocity = float(velocity)

    def adjust_velocity_by_cantilever(
        self,
        overhang_limit=8.0,
        cantilever_velocity=5.0,
        z_tolerance=10.0
    ):
        """Reduce velocity when support below is insufficient."""

        printpoints = self.printpoints
        layer_height = config.LAYER_HEIGHT

        for i, current in enumerate(printpoints):

            if (
                current.point_type != "print"
                or current.point.z <= layer_height
            ):
                continue

            distance = self._support_distance_xy(
                printpoints,
                i,
                layer_height,
                z_tolerance
            )

            if (
                distance is None
                or distance > overhang_limit
            ):
                current.velocity = min(
                    current.velocity,
                    cantilever_velocity
                )

    @staticmethod
    def _support_distance_xy(
        printpoints,
        index,
        layer_height,
        z_tolerance
    ):
        """Return nearest XY support distance one layer below."""

        current = printpoints[index]
        target_z = current.point.z - layer_height

        distances = []

        for previous in printpoints[:index]:

            if previous.point_type != "print":
                continue

            if (
                abs(previous.point.z - target_z)
                > z_tolerance
            ):
                continue

            dx = current.point.x - previous.point.x
            dy = current.point.y - previous.point.y

            distances.append(
                math.sqrt(
                    dx * dx
                    + dy * dy
                )
            )

        return min(distances) if distances else None
