********************
Real Robot Interface
********************

The simulation API is designed with the aim of making it easy to transition to the
interface of the real robots.
Below, we describe two different ways of switching between simulated and real robot:

1. Switching between :class:`~trifinger_simulation.sim_finger.SimFinger` and
   :class:`~trifinger_simulation.real_finger.RealFinger`
2. Using the real-robot interface and replacing the hardware back end with the
   simulation.

Note also, that the interface of :class:`~trifinger_simulation.sim_finger.SimFinger` is
designed to be as similar as possible to the real-robot front end, so even switching
between the two should be relatively easy.


RealRobot Wrapper
=================

The :class:`~trifinger_simulation.real_finger.RealFinger` class provides a wrapper
around the real-robot interface which can be used as a drop-in replacement for
:class:`~trifinger_simulation.sim_finger.SimFinger`.

Simple example, sending control commands to the TriFinger:

.. code-block:: python

    from trifinger_simulation import sim_finger, real_finger

    if use_real == True:
        robot = real_finger.RealFinger(finger_type)
    else:
        robot = sim_finger.SimFinger(finger_type)

    def step(action):
        for _ in range(steps_per_control):
            t = robot.append_desired_action(action)
            observation = robot.get_observation()

You can see an example of this usage in our :doc:`TriFingerReach environment
<src_trifinger_reach-py>`.


.. note::

    In order to use the ``RealFinger`` class, you would need additional packages from
    the TriFinger software bundle.  See :ref:`colcon`.



Real Robot Front End with Simulation Back End
=============================================

The :doc:`robot_fingers <robot_fingers:index>` package provides an
:doc:`robot_interfaces <robot_interfaces:index>`-based interface to the hardware of the
real TriFinger robots.

It is possible to use the front end of the real-robot interface but with the simulation
plugged in as back end.  This allows using the exact same code for simulation and real
robot.  For more information on this, see :ref:`robot_fingers:simulation_backend` in the
``robot_fingers`` documentation.
