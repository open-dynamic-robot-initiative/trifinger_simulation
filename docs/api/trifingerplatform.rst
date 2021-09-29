***************************
The TriFingerPlatform Class
***************************


The :class:`~trifinger_simulation.TriFingerPlatform` class provides access to the
simulation of the TriFinger platform using the same interface as for the real
robot.

For a detailed explanation of the software interface on the real robot, and the
logic behind it, please check the `paper
<https://arxiv.org/abs/2008.03596>`_.


API Documentation
=================


.. autoclass:: trifinger_simulation.TriFingerPlatform
   :special-members:

   .. automethod:: __init__

   .. method:: Action(torque=None, position=None)

      See :meth:`trifinger_simulation.SimFinger.Action`.

   .. method:: append_desired_action(action)

      See :meth:`trifinger_simulation.SimFinger.append_desired_action`.

   .. method:: get_current_timeindex

      See :meth:`trifinger_simulation.SimFinger.get_current_timeindex`.

   .. method:: get_timestamp_ms(t)

      See :meth:`trifinger_simulation.SimFinger.get_timestamp_ms`.

   .. method:: get_desired_action(t)

      See :meth:`trifinger_simulation.SimFinger.get_desired_action`.

   .. method:: get_applied_action(t)

      See :meth:`trifinger_simulation.SimFinger.get_applied_action`.

   .. method:: get_robot_observation(t)

      Get observation of the robot state (joint angles, torques, etc.).
      See :meth:`trifinger_simulation.SimFinger.get_observation`.

   .. automethod:: get_camera_observation

   .. automethod:: store_action_log

------------------------------------------------------------------------------

.. autoclass:: trifinger_simulation.ObjectPose
   :members:

------------------------------------------------------------------------------

.. autoclass:: trifinger_simulation.CameraObservation
   :members:

------------------------------------------------------------------------------

.. autoclass:: trifinger_simulation.TriCameraObservation
   :members:

------------------------------------------------------------------------------

.. autoclass:: trifinger_simulation.TriCameraObjectObservation
   :members:
