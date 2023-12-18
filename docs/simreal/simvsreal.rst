.. _sec-simulation-vs-real-robot:

************************
Simulation vs Real Robot
************************

The :class:`~trifinger_simulation.sim_finger.SimFinger` class provides an API to control
the TriFinger robots in simulation. Similarly, the
:cpp:class:`robot_interfaces::RobotFrontend` class provides an API to interact with
these robots in real life.

It is easy to switch between these two (see :doc:`simwithreal`). However, there are some
subtle differences one should keep in mind, which arise due to the accompanying switch
between non-real-time behaviour (in simulation), and real-time behaviour (with real
robots).


Simulation is Stepped in ``append_desired_action()``
====================================================

The simulation is explicitly stepped in the ``append_desired_action()``
method: this is because the simulation doesn't exhibit real-time
behaviour. So, every time the ``append_desired_action()`` is called,
the simulation is stepped, and the next time step in the simulation is computed.
This means that the state of the simulation does not change as long as this
method is not called. This is different than on the real robot, which will physically
continue to move and will repeat the last action if no new action is provided in time.

.. _`No waiting for future time steps`:

No Waiting for Future Time Steps
================================

On the real robot, it is possible to pass a time index that lies in the future
and most methods of the :cpp:class:`robot_interfaces::RobotFrontend` will simply block
and wait until this time step is reached.  The simulation, however, is not running
asynchronously but is actively stepped every time ``append_desired_action()`` is called.
Therefore the behaviour of waiting for a time step is not supported.  Instead, passing a
time index that lies in the future will result in an error.


No Time Series in SimFinger
===========================

The :doc:`robot_interfaces <robot_interfaces:index>` package makes use of time series
for observations, actions, etc.  This means all data of the last few time steps is
available.  One could, for example do the following to determine how the state of the
robot changed:

.. code-block:: python

    previous_observation = frontend.get_observation(t - 1)
    current_observation = frontend.get_observation(t)

The ``SimFinger``:class:`~trifinger_simulation.sim_finger.SimFinger` class does not
implement a time series, so it only provides the observation of the current time step
``t``.  Passing any other value for ``t`` will result in an error.


API-differences with robot_interfaces::RobotFrontend
====================================================

Our goal is to provide the same API in
:class:`~trifinger_simulation.sim_finger.SimFinger` as in ``RobotFrontend`` to make
transition between simulation and real robot easy.  There are a few differences, though.

Currently :class:`~trifinger_simulation.sim_finger.SimFinger` supports the following
methods:

- ``append_desired_action()``
- ``get_observation()``
- ``get_desired_action()``
- ``get_applied_action()``
- ``get_timestamp_ms()``
- ``get_current_timeindex()``

The following methods are not supported:

- ``get_status()``:  There are no meaningful values for the status message in
  simulation, so this method is omitted to avoid confusion.
- ``wait_until_timeindex()``:  In general the process of waiting for a specific
  time step is not supported, see :ref:`No waiting for future time steps`.
