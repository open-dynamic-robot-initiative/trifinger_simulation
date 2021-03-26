import numpy as np
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
        **kwargs,
    ):
        """
        Import a marker for visualization

        Args:
            number_of_goals (int): the desired number of goals to display
            goal_size (float): how big should this goal be
            initial_position (list of floats): where in xyz space should the
                goal first be displayed
        """
        self._kwargs = kwargs
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
                **self._kwargs,
            )
            self.goal_ids[i] = pybullet.createMultiBody(
                baseVisualShapeIndex=goal_shape_ids[i],
                basePosition=initial_position,
                baseOrientation=[0, 0, 0, 1],
                **self._kwargs,
            )
            (
                _,
                self.goal_orientations[i],
            ) = pybullet.getBasePositionAndOrientation(
                self.goal_ids[i],
                **self._kwargs,
            )

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
                goal_id,
                position,
                orientation,
                **self._kwargs,
            )


class ObjectMarker:
    def __init__(
        self,
        shape_type,
        position,
        orientation,
        color=(0, 1, 0, 0.5),
        pybullet_client_id=0,
        **kwargs,
    ):
        """
        Create a cube marker for visualization

        Args:
            shape_type: Shape type of the object (e.g. pybullet.GEOM_BOX).
            position: Position (x, y, z)
            orientation: Orientation as quaternion (x, y, z, w)
            kwargs: Keyword arguments that are passed to
                pybullet.createVisualShape.  Use this to specify
                shape-type-specify parameters like the object size.
        """
        self._pybullet_client_id = pybullet_client_id

        self.shape_id = pybullet.createVisualShape(
            shapeType=shape_type,
            rgbaColor=color,
            physicsClientId=self._pybullet_client_id,
            **kwargs,
        )
        self.body_id = pybullet.createMultiBody(
            baseVisualShapeIndex=self.shape_id,
            basePosition=position,
            baseOrientation=orientation,
            physicsClientId=self._pybullet_client_id,
        )

    def __del__(self):
        """Removes the cuboid from the environment."""
        # At this point it may be that pybullet was already shut down. To avoid
        # an error, only remove the object if the simulation is still running.
        if pybullet.isConnected(self._pybullet_client_id):
            pybullet.removeBody(self.body_id, self._pybullet_client_id)

    def set_state(self, position, orientation):
        """Set pose of the marker.

        Args:
            position: Position (x, y, z)
            orientation: Orientation as quaternion (x, y, z, w)
        """
        pybullet.resetBasePositionAndOrientation(
            self.body_id,
            position,
            orientation,
            physicsClientId=self._pybullet_client_id,
        )


class CuboidMarker(ObjectMarker):
    """Visualize a Cuboid."""

    def __init__(
        self,
        size,
        position,
        orientation,
        color=(0, 1, 0, 0.5),
        pybullet_client_id=0,
    ):
        """
        Create a cube marker for visualization

        Args:
            size (list): Lengths of the cuboid sides.
            position: Position (x, y, z)
            orientation: Orientation as quaternion (x, y, z, w)
            color: Color of the cube as a tuple (r, b, g, a)
        """
        size = np.asarray(size)
        super().__init__(
            pybullet.GEOM_BOX,
            position,
            orientation,
            color,
            pybullet_client_id,
            halfExtents=size / 2,
        )


class CubeMarker(CuboidMarker):
    """Visualize a cube."""

    def __init__(
        self,
        width,
        position,
        orientation,
        color=(0, 1, 0, 0.5),
        pybullet_client_id=0,
    ):
        """
        Create a cube marker for visualization

        Args:
            width (float): Length of one side of the cube.
            position: Position (x, y, z)
            orientation: Orientation as quaternion (x, y, z, w)
            color: Color of the cube as a tuple (r, b, g, a)
        """
        super().__init__(
            [width] * 3,
            position,
            orientation,
            color,
            pybullet_client_id,
        )
