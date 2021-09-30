*****************************************
Time Relation of Actions and Observations
*****************************************

On the real robot, we are using *time series* for all robot data like actions
and observations.  The user does not send actions directly to the robot but
appends them to the "desired actions" time series which serves as a queue. 
At each time step, identified by a *time index t*, the action at position *t* is
taken from the "desired actions" time series and processed.  At the same time an
observation is acquired from the robot and added to the "observation" time
series.  This means that the effect of the desired action ``a_t`` is not yet
visible in the observation ``y_t`` as is illustrated below.  (``a'_t``
corresponds to the *applied action*, see :ref:`Desired vs Applied Action`)

.. image:: ../images/applied_action_dependency.png
   :width: 80%
   :align: center


In the `robot_interfaces`_ software, the method
``append_desired_action()`` is used to
append actions to the time series.  It returns the time index *t* at which the
appended action will be executed.  Methods like
``get_observation`` expect a time index as
input and will return the data corresponding to this time step.  If the given
time index refers to a point in the future, these methods will block and wait
until that point is reached.

As the simulation is not real-time critical, the behaviour is a bit different
here:

- :meth:`~trifinger_simulation.SimFinger.append_desired_action` will directly
  apply the action and step the simulation.
- There is no actual time series.  The API in the simulation
  follows the same principle to make the transition to the real robot easier.
  However, it is implemented with a buffer size of 1, so the getter methods only
  provide data for the current time step.
- It is possible to access information from *t + 1*.  In a typical gym
  environment, it is expected that the observation returned by ``step(action)``
  belongs to the moment *after* the given action is executed (this corresponds
  to the time index *t + 1*).  To make it easier to get started, we therefore
  allow to access the observations of this time index in the simulation.  Note,
  however, that this is **not possible** on the real robot!

For more information on the API of the real robot, see our publication `TriFinger: An Open-Source
Robot for Learning Dexterity <https://arxiv.org/abs/2008.03596>`_.

.. _`robot_interfaces`: https://github.com/open-dynamic-robot-initiative/robot_interfaces
