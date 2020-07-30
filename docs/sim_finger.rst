***********************
Finger Robot Simulation
***********************

.. contents::


The :class:`~trifinger_simulation.SimFinger` class provides a simulation
environment for the "Finger Robots", i.e. FingerOne, TriFingerOne,
TriFingerEdu, etc.  See
:meth:`~trifinger_simulation.SimFinger.get_valid_finger_types` for a list of all
supported robots.

The interface for controlling the robot in the simulation is equivalent to the
one of ``robot_interfaces::RobotFrontend`` that is used for the real robots.
Note, however, that there are some differences in behaviour between simulation
and real robot, see :ref:`sec-simulation-vs-real-robot`.


.. _simfinger-usage-example:

Usage Example
=============

``demo_plain_torque_control.py`` shows a minimal example of how to control the
robot in simulation.

.. literalinclude:: ../demos/demo_plain_torque_control.py
   :lines: 2-

The ``SimFinger`` class is a wrapper around pyBullet that sets up the
environment and provides an API to control the robot equivalent to the
``RobotFrontend`` from the ``robot_interfaces`` package.  In fact, it should be
possible to simply replace the ``SimFinger`` instance with with an
``RobotFrontend`` instance to execute the same code on the real robot (in
practice, there are a few things to consider, see
:ref:`sec-simulation-vs-real-robot`).

Note that the next simulation step is computed in the
``append_desired_action()`` method.  So the state of the simulation only
changes when calling this method.



API Documentation
=================


.. autoclass:: trifinger_simulation.SimFinger

   .. automethod:: __init__

   .. automethod:: get_valid_finger_types

   .. automethod:: Action

   .. automethod:: append_desired_action

   .. automethod:: get_current_timeindex

   .. automethod:: get_timestamp_ms

   .. automethod:: get_desired_action

   .. automethod:: get_applied_action

   .. automethod:: get_observation

------------------------------------------------------------------------------

.. autoclass:: trifinger_simulation.Action

------------------------------------------------------------------------------

.. autoclass:: trifinger_simulation.Observation

