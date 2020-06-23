***************
Getting Started
***************

.. contents::


Installation
============

See `installation instructions <installation.md>`_.


A Simple Demo
=============

`demo_plain_torque_control.py <../demos/demo_plain_torque_control.py>`_ shows a
minimal example of how to control the robot in simulation.

.. literalinclude:: ../demos/demo_plain_torque_control.py
   :lines: 2-

The ``SimFinger`` class is a wrapper around pyBullet that sets up the
environment and provides an API to control the robot equivalent to the
``RobotFrontend`` from the ``robot_interfaces`` package.  In fact, it should be
possible to simply replace the ``SimFinger`` instance with with an
``RobotFrontend`` instance to execute the same code on the real robot (in
practice, there are a few things to consider, see `Simulation vs Real Robot`_).

Note that the next simulation step is computed in the
``append_desired_action()`` method.  So the state of the simulation only
changes when calling this method.


Simulation vs Real Robot
========================

We aim to have the same API for controlling the robot in the ``SimFinger`` class
as in ``robot_interfaces::RobotFrontend`` to allow transition from simulation to
real robot with only minimal modifications to the code.  There still a few
important differences one should keep in mind though.  These differences are
described in the following.

Note that it is also possible to use the simulation through the
``robot_interfaces`` structure by using a backend that uses the pyBullet driver.
This way, the full functionality of ``robot_interfaces`` is available (i.e. all
the issues described below are not there).  The drawback is that this way it is
not possible to directly access the simulation, e.g. to visualize goal
positions.  For more details see `robot_interfaces with Simulation`_.


Simulation is stepped in ``append_desired_action()``
----------------------------------------------------

Everytime the ``append_desirec_action()`` method is called, the next time step
in the simulation is computed.  This means that the state of the simulation
does not change as long as this method is not called.
This is different to the real robot, which will physically continue to move
and will repeat the last action if no new action is provided in time.


Real Time Behaviour
-------------------

As described above, the simulation is stepped in ``get_observation()``.  Each
time this function is called, the next time step is computed, based on the
``time_step`` duration passed to ``SimFinger`` during initialization.  It does
not matter how much time passes between these calls as the simulation is paused
in this time.  So your algorithm can take all the time it needs to compute the
next action.

On the real robot though, the world does not simply stop while your algorithm is
running.  The robot is controlled at a rate of 1 kHz so it expects a new action
every 1 ms.  If your algorithm is not providing the next action fast enough, the
robot will repeat the previous action until a new action is provided.


No Time Series in SimFinger
---------------------------

The ``robot_interfaces`` package makes use of time series for observations,
actions, etc.  This means all data of the last few time steps is available.  One
could, for example do the following to determine how the state of the robot
changed:

.. code-block::

    previous_observation = frontend.get_observation(t - 1)
    current_observation = frontend.get_observation(t)

The ``SimFinger`` class does not implement a time series, so it only provides
the observation of the current time step ``t``.  Passing any other value for
``t`` will result in an error.


The Simulation API is not complete
----------------------------------

While our goal is to provide the full API of ``RobotFrontend`` in ``SimFinger``,
this is not yet the case, so some methods of ``RobotFrontend`` are not yet
available in ``SimFinger``.


Currently ``SimFinger`` supports the following methods:

- ``append_desired_action()``
- ``get_observation()``



robot_interfaces with Simulation
================================

It is possible to use ``robot_interfaces`` with pyBullet by using the pyBullet
driver in the backend.  By doing this, you only need to replace the backend when
switching between simulation and real robot while all the rest of your code can
remain the same.


To create a TriFinger backend using simulation:

.. code-block::

    import pybullet_fingers.drivers

    backend = pybullet_fingers.drivers.create_trifinger_backend(
        robot_data, real_time_mode=True, visualize=True
    )

If ``real_time_mode`` is ``True``, the backend will expect a new action every
millisecond and repeat the previous action if it is not provided in time (like
on the real robot).  If set to ``False``, it will run as fast as possible and
wait for new actions.

Set ``visualize=True`` to run the pyBullet GUI for visualization.


For a complete example, see `demo_robot_interface.py
<../demos/demo_robot_interface.py>`_.
