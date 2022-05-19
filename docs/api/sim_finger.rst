*******************
The SimFinger Class
*******************

The :class:`~trifinger_simulation.SimFinger` class provides a simulation
environment for the different finger robots: these include the TriFingerEdu, our
reproducible open-source robot platform; the TriFingerPro, and the TriFingerOne.
See :func:`trifinger_simulation.finger_types_data.get_valid_finger_types()` for
a complete list of supported robots.

The interface of :class:`~trifinger_simulation.SimFinger` mimics that of the
real robot frontend (see :doc:`../simreal/simwithreal`),
which makes it easy to transfer code between simulation and real robot.

When using the TriFinger platform, see also
:class:`~trifinger_simulation.TriFingerPlatform`, which is a wrapper around
:class:`~trifinger_simulation.SimFinger`.

.. _`Desired vs Applied Action`:

Desired vs Applied Action
=========================

The action given by the user is called the *desired* action.  This action may be
altered before it is actually applied on the robot, e.g. by some safety checks
limiting torque and velocity.  This altered action is called the *applied*
action.  You may use
:meth:`~trifinger_simulation.SimFinger.get_applied_action` to see what action
actually got applied on the robot.


API Documentation 
===================


.. autoclass:: trifinger_simulation.SimFinger
   :members:

------------------------------------------------------------------------------

.. autoclass:: trifinger_simulation.Action
   :members:
   :member-order: bysource

------------------------------------------------------------------------------

.. autoclass:: trifinger_simulation.Observation
   :members:
   :member-order: bysource
