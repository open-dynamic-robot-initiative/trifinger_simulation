****************************
TriFingerPlatform Simulation
****************************

.. contents::


The :class:`~pybullet_fingers.TriFingerPlatform` class
provides access to the simulation of the TriFinger platform using the same
interface as ``robot_interfaces::TriFingerPlatformFrontend`` for the real robot.

It is basically a wrapper around :class:`~pybullet_fingers.SimFinger` which
takes care of initializing the environment, exposes the relevant methods to
control the robot and adds the ones for getting the object and camera
observations.


Usage Example
=============

.. literalinclude:: ../demos/demo_trifinger_platform.py


API Documentation
=================


.. autoclass:: pybullet_fingers.TriFingerPlatform
   :special-members:

   .. automethod:: __init__

   .. method:: Action(torque=None, position=None)

      See :meth:`pybullet_fingers.SimFinger.Action`.

   .. method:: append_desired_action(action)

      See :meth:`pybullet_fingers.SimFinger.append_desired_action`.

   .. method:: get_current_timeindex

      See :meth:`pybullet_fingers.SimFinger.get_current_timeindex`.

   .. method:: get_timestamp_ms(t)

      See :meth:`pybullet_fingers.SimFinger.get_timestamp_ms`.

   .. method:: get_desired_action(t)

      See :meth:`pybullet_fingers.SimFinger.get_desired_action`.

   .. method:: get_applied_action(t)

      See :meth:`pybullet_fingers.SimFinger.get_applied_action`.

   .. method:: get_robot_observation(t)

      Get observation of the robot state (joint angles, torques, etc.).
      See :meth:`pybullet_fingers.SimFinger.get_observation`.

   .. automethod:: get_object_pose

   .. automethod:: get_camera_observation

   .. automethod:: store_action_log

------------------------------------------------------------------------------

.. autoclass:: pybullet_fingers.ObjectPose
   :members:

------------------------------------------------------------------------------

.. autoclass:: pybullet_fingers.CameraObservation
   :members:

------------------------------------------------------------------------------

.. autoclass:: pybullet_fingers.TriCameraObservation
   :members:
