**********
Quickstart
**********

The heart of this simulation is the
:class:`~trifinger_simulation.sim_finger.SimFinger` class.  It takes care of
initialising PyBullet and loading the robot into the environment.

.. code-block:: Python

    from trifinger_simulation import SimFinger

    sim = SimFinger(
        finger_type="fingerpro",
        enable_visualization=True,
    )

The above example populates the simulation with a "FingerPro" robot, see
:doc:`finger_types` for the different robots that are supported.

By default, the fingers are pointing straight down initially, which usually
means that they are intersecting with the ground.  It is thus important to
first set them to a feasible position before starting to run the simulation:

.. code-block:: Python

    # set feasible initial position for FingerPro
    sim.reset_finger_positions_and_velocities([0, -0.7, -1.5])


Now you can start to run the simulation by sending actions to the robot.  Below
is a very basic example that uses position commands to hold the robot at the
initial position and prints the actual observed position:

.. code-block:: Python

    import time

    while True:
        # to have it run in real-time
        time.sleep(sim.time_step_s)

        # create a position action
        action = sim.Action(position=[0, -0.7, -1.5])

        # send it to the robot (this executes one simulation time step)
        t = sim.append_desired_action(action)

        # get the observation of the current time step
        observation = sim.get_observation(t)

        print(observation.position)

When using position commands, a PD controller is executed internally to compute
the desired torques.  You can also directly use torque commands:

.. code-block:: Python

    action = sim.Action(torque=[0., -0.2, -0.2])


Regarding the meaning of the *time index* ``t`` in the above example, please
see :doc:`../simreal/timesteplogic`.


Adding Objects to the Simulation
================================

We have a few objects defined in the
:mod:`~trifinger_simulation.collision_objects` module, which can easily be
added to the simulation.  For example to add a coloured cube add the following
code **after** initialising ``SimFinger``:

.. code-block:: Python

    from trifinger_simulation import collision_objects

    cube = collision_objects.ColoredCubeV2(position=(0, 0, 0.4))


Modifying the Simulation Environment
====================================

If you want to modify any property of the simulation (add objects, change
parameters, etc.) beyond the interface that is provided by the classes of this
package, you can simply access pybullet directly.  For example to change the
default camera pose in the PyBullet GUI:

.. code-block:: Python

    import pybullet

    pybullet.resetDebugVisualizerCamera(
        cameraDistance=2.0,
        cameraYaw=0.0,
        cameraPitch=-30.0,
        cameraTargetPosition=(0, 0, 0.2),
        physicsClientId=sim._pybullet_client_id,
    )

Note, however, that all these modifications should be done **after**
initialising ``SimFinger``.  Ideally also pass the "physics client id" to all
PyBullet calls like shown above.


More Examples
=============

For more examples, see the scripts in the `demos/` folder of the
trifinger_simulation package.  Good starting points are

- ``demo_plain_torque_control.py``:  A very minimalistic demo, similar to the
  example above.
- ``demo_control.py``:  A slightly more complicated demo.
- ``demo_inverse_kinematics.py``:  Uses inverse kinematics to control the
  finger tip position.
- ``demo_cameras.py``:  Shows how to use the cameras in simulation (note that
  image rendering is pretty slow, though).
