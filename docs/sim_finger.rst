*******************
The SimFinger Class
*******************

The :class:`~trifinger_simulation.SimFinger` class provides a simulation
environment for the different finger robots: these include the TriFingerEdu, our
reproducible open-source robot platform; the TriFingerPro, and the TriFingerOne.

.. note:: for a complete list of supported robots,
call :method:`~trifinger_simulation.finger_types_data.get_valid_finger_types()`.

The interface for controlling the robot in the simulation is equivalent to the
one of ``robot_interfaces::RobotFrontend`` that is used for the real robots.
Note, however, that there are some differences in behaviour between simulation
and real robot, see :ref:`sec-simulation-vs-real-robot`.

Class :class:`~trifinger_simulation.TriFingerPlatform` is a wrapper around the
:class:`~trifinger_simulation.SimFinger` class.
.

API Documentation 
===================


.. autoclass:: trifinger_simulation.SimFinger

   :members:

------------------------------------------------------------------------------

.. autoclass:: trifinger_simulation.Action

------------------------------------------------------------------------------

.. autoclass:: trifinger_simulation.Observation

.. .. _simfinger-usage-example:
.. 
.. Usage Example
.. =============
.. 
.. ``demo_plain_torque_control.py`` shows a minimal example of how to control the
.. robot in simulation.
.. 
.. .. literalinclude:: ../examples/demo_plain_torque_control.py
..    :lines: 2-
.. 
.. The ``SimFinger`` class is a wrapper around pyBullet that sets up the
.. environment and provides an API to control the robot equivalent to the
.. ``RobotFrontend`` from the ``robot_interfaces`` package.  In fact, it should be
.. possible to simply replace the ``SimFinger`` instance with with an
.. ``RobotFrontend`` instance to execute the same code on the real robot (in
.. practice, there are a few things to consider, see
.. :ref:`sec-simulation-vs-real-robot`).
.. 
.. Note that the next simulation step is computed in the
.. ``append_desired_action()`` method.  So the state of the simulation only
.. changes when calling this method.


