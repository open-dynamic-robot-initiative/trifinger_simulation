import pybullet


def import_mesh(
    mesh_file_path,
    position,
    orientation=[0, 0, 0, 1],
    is_concave=False,
    color_rgba=None,
):
    """
    Create a collision object based on a mesh file.

    Args:
        mesh_file_path:  Path to the mesh file.
        position:  Position (x, y, z) of the object.
        orientation:  Quaternion defining the orientation of the object.
        is_concave:  If set to true, the object is loaded as concav shape.
            Only use this for static objects.
        color_rgba:  Optional colour of the object given as a list of RGBA
            values in the interval [0, 1].  If not specified, pyBullet
            assigns a random colour.

    Returns:
        The created object.
    """
    if is_concave:
        flags = pybullet.GEOM_FORCE_CONCAVE_TRIMESH
    else:
        flags = 0

    object_id = pybullet.createCollisionShape(
        shapeType=pybullet.GEOM_MESH, fileName=mesh_file_path, flags=flags
    )

    obj = pybullet.createMultiBody(
        baseCollisionShapeIndex=object_id,
        baseVisualShapeIndex=-1,
        basePosition=position,
        baseOrientation=orientation,
    )

    # set colour
    if color_rgba is not None:
        pybullet.changeVisualShape(obj, -1, rgbaColor=color_rgba)

    return obj


class Block:
    """
    To interact with a block object
    """

    def __init__(
        self,
        position=[0.15, 0.0, 0.0425],
        orientation=[0, 0, 0, 1],
        half_size=0.0325,
        mass=0.08,
    ):
        """
        Import the block

        Args:
            position (list): where in xyz space should the block
                be imported
            orientation (list): initial orientation quaternion of the block
            half_size (float): how large should this block be
            mass (float): how heavy should this block be
        """
        self.block_id = pybullet.createCollisionShape(
            shapeType=pybullet.GEOM_BOX, halfExtents=[half_size] * 3
        )
        self.block = pybullet.createMultiBody(
            baseCollisionShapeIndex=self.block_id,
            basePosition=position,
            baseOrientation=orientation,
            baseMass=mass,
        )

        # set dynamics of the block
        lateral_friction = 1
        spinning_friction = 0.001
        restitution = 0
        pybullet.changeDynamics(
            bodyUniqueId=self.block,
            linkIndex=-1,
            lateralFriction=lateral_friction,
            spinningFriction=spinning_friction,
            restitution=restitution,
        )

    def set_state(self, position, orientation):
        """
        Resets the block state to the provided position and
        orientation

        Args:
            position: the position to which the block is to be
                set
            orientation: desired to be set
        """
        pybullet.resetBasePositionAndOrientation(
            self.block, position, orientation
        )

    def get_state(self):
        """
        Returns:
            Current position and orientation of the block.
        """
        position, orientation = pybullet.getBasePositionAndOrientation(
            self.block
        )
        return list(position), list(orientation)

    def __del__(self):
        """
        Removes the block from the environment
        """
        # At this point it may be that pybullet was already shut down. To avoid
        # an error, only remove the object if the simulation is still running.
        if pybullet.isConnected():
            pybullet.removeBody(self.block)
