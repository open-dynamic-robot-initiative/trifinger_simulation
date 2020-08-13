.. _`Switching between the Real Robot and the Simulated Robot in this Simulation`:

*****************************************************************************
Switching between the Real Robot and the Simulated Robot in this Simulation
*****************************************************************************

We have designed the entire `trifinger_simulation` package with the
aim of making it possible to seamlessly transition to our software for
interfacing with these robots in the real world, to the extent of making
this transition pretty much plug and play: depending on whether you want
to create an instance of a simulated robot or a
real robot, which you would like to use in a gym environment for learning, or for
optimal control, or just to play around in by sending some crazy random commands.

To give a very simple example of this, consider sending some control commands to
the TriFinger,

.. code-block:: python

    from trifinger_simulation import sim_finger, real_finger

    if use_real == True:
        # to know what this robot is, look at the example below
        robot = sim_finger.RealFinger(finger_type)
    else:
        robot = real_finger.SimFinger(finger_type)

    def step(action):
        for _ in range(steps_per_control):
            t = robot.append_desired_action(action)
            observation = robot.get_observation()

As you can see from the example above, you can control the TriFinger in the
same way irrespective of whether its a simulated or real. You can see an example
of this usage in our `TriFingerReach environment <https://github.com/open-dynamic-robot-initiative/trifinger_simulation/blob/master/python/trifinger_simulation/gym_wrapper/envs/finger_reach.py>`_.

The ``RealFinger`` class exposes analogous methods to interact with the real robot as the ones
for ``SimFinger``. Head over to the :ref:`RealFinger`_ for details.

.. note:: 

    If at this point, you are interested in exploring our software interface for the
    real robot, you can head over to the frontend of this interface to check out
    the exposed methods in the `RobotFrontend`_,
    and check out some demos for controlling a real robot in `here <https://github.com/open-dynamic-robot-initiative/robot_fingers/tree/master/demos>`_.

In addition, you can also access the complete API of interacting with the real robot (the `RobotFrontend`_ mentioned
in the note above) with this simulation. Head over to the next section, :ref:`robot_interfaces with Simulation` to know how to do this.


.. _`RobotFrontend`: https://github.com/open-dynamic-robot-initiative/robot_interfaces/blob/master/include/robot_interfaces/robot_frontend.hpp