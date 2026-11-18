"""Selected original source excerpts for academic application review.
Taken from _05_frame_orientation.py. These are class methods, NOT standalone runnable programs.
They depend on the project PrintPath / PrintPoint data model and COMPAS.
"""

import math
from compas.geometry import Frame, Point, Quaternion, Vector
from _02_print_point import PrintPoint

class FrameOrientation:

    def align_frames_z_to_segments(
        self,
        ground_z=0.0,
        ground_tolerance=5.0,
        tolerance=1e-1
    ):
        """
        Ground horizontal segments -> World Down.
        Other segments -> opposite path direction.
        """

        world_down = Vector(0, 0, -1)

        for layer in self.layers:

            pts = layer.printpoints

            if len(pts) < 2:
                continue

            for i, pt in enumerate(pts):

                segment = self._get_segment(
                    pts,
                    i,
                    tolerance
                )

                if segment is None:
                    continue

                other = (
                    pts[i + 1]
                    if i == 0
                    else pts[i - 1]
                )

                on_ground = (
                    abs(pt.point.z - ground_z)
                    <= ground_tolerance
                    and
                    abs(other.point.z - ground_z)
                    <= ground_tolerance
                )

                zaxis = (
                    world_down
                    if on_ground and segment.z >= -tolerance
                    else -segment
                )

                pt.frame = self._frame_from_z(
                    pt.point,
                    zaxis,
                    tolerance
                )

    def orient_frames_down(
        self,
        min_angle=45.0,
        upward_lift=30.0,
        tolerance=1e-3
    ):
        """Keep all tool Z axes downward and at least min_angle from XY."""

        world_down = Vector(0, 0, -1)

        target_angle = math.radians(min_angle)
        lift_angle = math.radians(upward_lift)

        for layer in self.layers:

            pts = layer.printpoints

            for i, pt in enumerate(pts):

                zaxis = pt.frame.zaxis.copy()

                if zaxis.length <= tolerance:
                    continue

                zaxis.unitize()

                horizontal = Vector(
                    zaxis.x,
                    zaxis.y,
                    0
                )

                # -------------------------------------------------------------
                # 1. Upward -> rotate downward
                # -------------------------------------------------------------

                if zaxis.z > tolerance:

                    if horizontal.length <= tolerance:

                        segment = self._get_segment(
                            pts,
                            i,
                            tolerance
                        )

                        if segment is not None:
                            horizontal = Vector(
                                segment.x,
                                segment.y,
                                0
                            )

                        if horizontal.length <= tolerance:
                            horizontal = Vector(1, 0, 0)

                    horizontal.unitize()

                    current_angle = math.atan2(
                        zaxis.z,
                        math.sqrt(
                            zaxis.x ** 2
                            + zaxis.y ** 2
                        )
                    )

                    new_angle = (
                        current_angle
                        - lift_angle
                    )

                    if new_angle >= 0:
                        new_angle = -target_angle

                    zaxis = (
                        horizontal * math.cos(new_angle)
                        + Vector(0, 0, 1) * math.sin(new_angle)
                    )

                    zaxis.unitize()

                # -------------------------------------------------------------
                # 2. Downward but too horizontal -> minimum angle
                # -------------------------------------------------------------

                if zaxis.z < -tolerance:

                    angle = self._angle_to_xy(
                        zaxis
                    )

                    if angle < min_angle:

                        horizontal = Vector(
                            zaxis.x,
                            zaxis.y,
                            0
                        )

                        if horizontal.length <= tolerance:
                            zaxis = world_down.copy()

                        else:
                            horizontal.unitize()

                            zaxis = (
                                horizontal * math.cos(target_angle)
                                + world_down * math.sin(target_angle)
                            )

                            zaxis.unitize()

                # -------------------------------------------------------------
                # 3. Final safety
                # -------------------------------------------------------------

                if zaxis.z >= -tolerance:

                    horizontal = Vector(
                        zaxis.x,
                        zaxis.y,
                        0
                    )

                    if horizontal.length <= tolerance:
                        zaxis = world_down.copy()

                    else:
                        horizontal.unitize()

                        zaxis = (
                            horizontal * math.cos(target_angle)
                            + world_down * math.sin(target_angle)
                        )

                        zaxis.unitize()

                pt.frame = self._frame_from_z(
                    pt.point,
                    zaxis,
                    tolerance
                )

    def interpolate_frames(
        self,
        max_angle=10.0,
        max_distance=50.0
    ):
        """Insert points when frame rotation or distance is too large."""

        for layer in self.layers:

            original = list(layer.printpoints)

            if len(original) < 2:
                continue

            result = []

            for a, b in zip(
                original,
                original[1:]
            ):

                result.append(a)

                angle_steps = int(
                    math.ceil(
                        self._frame_angle(
                            a.frame,
                            b.frame
                        ) / max_angle
                    )
                )

                distance_steps = int(
                    math.ceil(
                        a.point.distance_to_point(
                            b.point
                        ) / max_distance
                    )
                )

                steps = max(
                    1,
                    angle_steps,
                    distance_steps
                )

                for j in range(1, steps):

                    t = float(j) / steps

                    frame = self._tween_frame(
                        a.frame,
                        b.frame,
                        t
                    )

                    source = (
                        b
                        if b.point_type == "departure"
                        else a
                    )

                    pt = PrintPoint(
                        key=0,
                        point=frame.point,
                        velocity=source.velocity,
                        blend=source.blend,
                        wait_time=0.0,
                        layer_idx=layer.layer_idx,
                        point_type=source.point_type
                    )

                    pt.frame = frame

                    result.append(pt)

            result.append(original[-1])

            layer.printpoints = result
            layer._update_printpoints()
            layer._update_path()

    @staticmethod
    def _get_segment(
        printpoints,
        index,
        tolerance=1e-6
    ):

        if len(printpoints) < 2:
            return None

        if index == 0:
            segment = (
                printpoints[1].point
                - printpoints[0].point
            )

        else:
            segment = (
                printpoints[index].point
                - printpoints[index - 1].point
            )

        if segment.length <= tolerance:
            return None

        segment.unitize()

        return segment

    @staticmethod
    def _frame_from_z(
        point,
        zaxis,
        tolerance=1e-6
    ):

        reference_x = Vector(0, -1, 0)
        reference_y = Vector(-1, 0, 0)

        zaxis = zaxis.copy()

        if zaxis.length <= tolerance:
            return Frame(
                point,
                reference_x,
                reference_y
            )

        zaxis.unitize()

        xaxis = (
            reference_x
            - zaxis * reference_x.dot(zaxis)
        )

        if xaxis.length <= tolerance:
            xaxis = (
                reference_y
                - zaxis * reference_y.dot(zaxis)
            )

        if xaxis.length <= tolerance:
            return Frame(
                point,
                reference_x,
                reference_y
            )

        xaxis.unitize()

        yaxis = zaxis.cross(xaxis)
        yaxis.unitize()

        return Frame(
            point,
            xaxis,
            yaxis
        )

    @staticmethod
    def _angle_to_xy(vector):

        vector = vector.copy()
        vector.unitize()

        return math.degrees(
            math.asin(
                min(
                    1.0,
                    abs(vector.z)
                )
            )
        )

    def _frame_angle(
        self,
        frame_a,
        frame_b
    ):

        return max(
            self._vector_angle(
                frame_a.zaxis,
                frame_b.zaxis
            ),
            self._vector_angle(
                frame_a.xaxis,
                frame_b.xaxis
            )
        )

    @staticmethod
    def _vector_angle(
        vector_a,
        vector_b
    ):

        a = vector_a.copy()
        b = vector_b.copy()

        a.unitize()
        b.unitize()

        dot = max(
            -1.0,
            min(
                1.0,
                a.dot(b)
            )
        )

        return math.degrees(
            math.acos(dot)
        )

    @staticmethod
    def _tween_frame(
        frame_a,
        frame_b,
        t
    ):

        t = max(
            0.0,
            min(1.0, t)
        )

        point = Point(
            frame_a.point.x
            + (frame_b.point.x - frame_a.point.x) * t,

            frame_a.point.y
            + (frame_b.point.y - frame_a.point.y) * t,

            frame_a.point.z
            + (frame_b.point.z - frame_a.point.z) * t
        )

        qa = Quaternion.from_frame(frame_a)
        qb = Quaternion.from_frame(frame_b)

        frame = Frame.from_quaternion(
            Quaternion.slerp(
                qa,
                qb,
                t
            )
        )

        frame.point = point

        return frame
