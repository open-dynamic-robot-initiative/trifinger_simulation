import pybullet


class Marker:
    """
    In case any point(for eg. the goal position) in space is to be
    visualized using a marker.
    """

    def __init__(
        self,
        number_of_goals,
        goal_size=0.015,
        initial_position=[0.18, 0.18, 0.08],
    ):
        """
        Import a marker for visualization

        Args:
            number_of_goals (int): the desired number of goals to display
            goal_size (float): how big should this goal be
            initial_position (list of floats): where in xyz space should the
                goal first be displayed
            """
        color_cycle = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]]

        goal_shape_ids = [None] * number_of_goals
        self.goal_ids = [None] * number_of_goals
        self.goal_orientations = [None] * number_of_goals

        # Can use both a block, or a sphere: uncomment accordingly
        for i in range(number_of_goals):
            color = color_cycle[i % len(color_cycle)]
            goal_shape_ids[i] = pybullet.createVisualShape(
                # shapeType=pybullet.GEOM_BOX,
                # halfExtents=[goal_size] * number_of_goals,
                shapeType=pybullet.GEOM_SPHERE,
                radius=goal_size,
                rgbaColor=color,
            )
            self.goal_ids[i] = pybullet.createMultiBody(
                baseVisualShapeIndex=goal_shape_ids[i],
                basePosition=initial_position,
                baseOrientation=[0, 0, 0, 1],
            )
            (
                _,
                self.goal_orientations[i],
            ) = pybullet.getBasePositionAndOrientation(self.goal_ids[i])

    def set_state(self, positions):
        """
        Set new positions for the goal markers with the orientation being the
        same as when they were imported.

        Args:
            positions (list of lists):  List of lists with
                x,y,z positions of all goals.
        """
        for goal_id, orientation, position in zip(
            self.goal_ids, self.goal_orientations, positions
        ):
            pybullet.resetBasePositionAndOrientation(
                goal_id, position, orientation
            )


class CubeMarker:
    """Visualize a cube."""

    def __init__(
        self,
        width,
        position,
        orientation,
        color=(0, 1, 0, 0.5)
    ):
        """
        Create a cube marker for visualization

        Args:
            width (float): Length of one side of the cube.
            position: Position (x, y, z)
            orientation: Orientation as quaternion (x, y, z, w)
            color: Color of the cube as a tuple (r, b, g, q)
            """
        self.shape_id = pybullet.createVisualShape(
            shapeType=pybullet.GEOM_BOX,
            halfExtents=[width / 2] * 3,
            rgbaColor=color,
        )
        self.body_id = pybullet.createMultiBody(
            baseVisualShapeIndex=self.shape_id,
            basePosition=position,
            baseOrientation=orientation,
        )

    def set_state(self, position, orientation):
        """Set pose of the marker.

        Args:
            position: Position (x, y, z)
            orientation: Orientation as quaternion (x, y, z, w)
        """
        pybullet.resetBasePositionAndOrientation(
            self.body_id, position, orientation
        )
