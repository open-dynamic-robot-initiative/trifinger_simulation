import pybullet


def import_mesh(
    mesh_file_path,
    position,
    orientation=[0, 0, 0, 1],
    is_concave=False,
    color_rgba=None,
    pybullet_client_id=None,
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
        shapeType=pybullet.GEOM_MESH,
        fileName=mesh_file_path,
        flags=flags,
        physicsClientId=pybullet_client_id,
    )

    obj = pybullet.createMultiBody(
        baseCollisionShapeIndex=object_id,
        baseVisualShapeIndex=-1,
        basePosition=position,
        baseOrientation=orientation,
        physicsClientId=pybullet_client_id,
    )

    # set colour
    if color_rgba is not None:
        pybullet.changeVisualShape(
            obj,
            -1,
            rgbaColor=color_rgba,
            physicsClientId=pybullet_client_id,
        )

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
        **kwargs,
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

        self._kwargs = kwargs

        self.block_id = pybullet.createCollisionShape(
            shapeType=pybullet.GEOM_BOX,
            halfExtents=[half_size] * 3,
            **self._kwargs,
        )
        self.block = pybullet.createMultiBody(
            baseCollisionShapeIndex=self.block_id,
            basePosition=position,
            baseOrientation=orientation,
            baseMass=mass,
            **self._kwargs,
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
            **self._kwargs,
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
            self.block,
            position,
            orientation,
            **self._kwargs,
        )

    def get_state(self):
        """
        Returns:
            Current position and orientation of the block.
        """
        position, orientation = pybullet.getBasePositionAndOrientation(
            self.block,
            **self._kwargs,
        )
        return list(position), list(orientation)

    def __del__(self):
        """
        Removes the block from the environment
        """
        # At this point it may be that pybullet was already shut down. To avoid
        # an error, only remove the object if the simulation is still running.
        if pybullet.isConnected(**self._kwargs):
            pybullet.removeBody(self.block, **self._kwargs)


class Object:
    def __init__(
        self,
        type="block",
        position=[0.05, 0.0, 0.0325],
        orientation=[0, 0, 0, 1],
        half_size=0.0325,
        radius=0.0325,
        height=0.0325,
        color_rgba=[255, 255, 255, 1],
        mass=0.08,
        lateral_friction=1,
        spinning_friction=0.001,
        restitution=0,
        mesh_file_path=None,
        is_concave=False,
        **kwargs,
    ):
        """Imports the object.

        Args:
            type (string): The type of object to load into the env.
                Supported types include "block", "sphere", "capsule",
                "cylinder", or some "custom" object. In order to use a custom
                object type, you also need to pass the location of the mesh
                file for this object.
            position (list): where in xyz space should the object
                be imported
            orientation (list): initial orientation quaternion of
                the object
            half_size (float): how large should this object be, this is a valid
                parameter for a "block"
            radius (float): valid for a "sphere", "capsule", or "cylinder"
                object
            height (float): valid for a "capsule" or a "cylinder" object
            color_rgba:  Optional colour of the object given as a list of RGBA
                values in the interval [0, 1].  If not specified, pyBullet
                assigns a random colour.
            mass (float): how heavy should this object be
            lateral_friction (double): the friction coefficient of the object
            spinning_friction (double): the spinning friction of the object
                around its contact normal
            restitution (double): the coefficient of restittuion for the object
            mesh_file_path:  Path to the mesh file for a "custom" object type
            is_concave (bool): If set to true, the "custom" object is loaded
                as concave shape. Only use this for static objects.
        Note:
            This assumes that you are already connected to an exisiting physics
            server in order to be compatible with pre-existing API of the
            "Block" class.
        """
        self._kwargs = kwargs
        if type == "block":
            self.object_id = pybullet.createCollisionShape(
                shapeType=pybullet.GEOM_BOX,
                halfExtents=[half_size] * 3,
                **self._kwargs,
            )
        elif type == "sphere":
            self.object_id = pybullet.createCollisionShape(
                shapeType=pybullet.GEOM_SPHERE,
                radius=radius,
                **self._kwargs,
            )
        elif type == "capsule":
            self.object_id = pybullet.createCollisionShape(
                shapeType=pybullet.GEOM_CAPSULE,
                radius=radius,
                height=height,
                **self._kwargs,
            )
        elif type == "cylinder":
            self.object_id = pybullet.createCollisionShape(
                shapeType=pybullet.GEOM_CYLINDER,
                radius=radius,
                height=height,
                **self._kwargs,
            )
        elif type == "custom":
            if mesh_file_path is None:
                raise ValueError("Please pass a mesh file to import the custom"
                                "desired object from. :)")
            if is_concave:
                flags = pybullet.GEOM_FORCE_CONCAVE_TRIMESH
            else:
                flags = 0
            self.object_id = pybullet.createCollisionShape(
                shapeType=pybullet.GEOM_MESH,
                fileName=mesh_file_path,
                flags=flags,
                **self._kwargs,
            )
        self.object = pybullet.createMultiBody(
            baseCollisionShapeIndex=self.object_id,
            baseVisualShapeIndex=-1,
            basePosition=position,
            baseOrientation=orientation,
            baseMass=mass,
            **self._kwargs,
        )

        # sets visual properties of the object
        if color_rgba is not None:
            self.change_texture(
                color_rgba=color_rgba
            )

        # sets dynamics of the object
        self.change_dynamics(
            lateral_friction=lateral_friction,
            spinning_friction=spinning_friction,
            restitution=restitution,
        )

    def change_dynamics(
            self,
            lateral_friction=1,
            spinning_friction=0.001,
            restitution=0):
        """Allows to change the dynamics properties of the object.

        Args:

            mass (float): how heavy should this object be
            lateral_friction (double): the friction coefficient of the object
            spinning_friction (double): the spinning friction of the object
                around its contact normal
            restitution (double): the coefficient of restittuion for the object
        """
        pybullet.changeDynamics(
            bodyUniqueId=self.object,
            linkIndex=-1,
            lateralFriction=lateral_friction,
            spinningFriction=spinning_friction,
            restitution=restitution,
            **self._kwargs,
        )

    def change_texture(self, color_rgba=None, texture_file_path=None):
        """Changes the texture and color of the object.

        Allows to load some custom image or texture onto the object, for eg.
        this could be a png file. It also allows to change the object colour.
        Note that, in case both desired color and desired texture are passed,
        then both will get loaded onto the object, and the texture will have
        preference over the color since it will get loaded on top of it.

        Args:

            color_rgba: A list of RGBA values.
            texture_file_path: Path to a texture that is to be loaded onto
                the object

        """
        if texture_file_path is None:
            if color_rgba is None:
                raise ValueError("This method was called without any values"
                                "for the desired visual attribute")
            else:
                pybullet.changeVisualShape(
                    self.object,
                    -1,
                    rgbaColor=color_rgba,
                    **self._kwargs,
                )
        else:
            texture_id = pybullet.loadTexture(texture_file_path)
            pybullet.changeVisualShape(
                    self.object,
                    -1,
                    textureUniqueId=texture_id,
                    **self._kwargs,
                )

    def set_state(self, position, orientation=None):
        """Resets the object state to the provided position and
        orientation

        Args:
            position: the position to which the block is to be
                set
            orientation: desired to be set
        """
        if orientation is None:
            orientation = [1, 1, 1, 1]
        pybullet.resetBasePositionAndOrientation(
            self.object, position, orientation,
            **self._kwargs,
        )

    def get_state(self):
        """Gets the current position and orientation of the object.
        Returns:
            Current position and orientation of the object.
        """
        position, orientation = pybullet.getBasePositionAndOrientation(
            self.object, **self._kwargs,
        )
        return list(position), list(orientation)

    def __del__(self):
        """Removes the object from the environment
        """
        # At this point it may be that pybullet was already shut down. To avoid
        # an error, only remove the object if the simulation is still running.
        if pybullet.isConnected():
            pybullet.removeBody(
                self.object,
                **self._kwargs,
            )
