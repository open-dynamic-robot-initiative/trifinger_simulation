**********************************************
Simulation vs Real Robot in the Real Robot API
**********************************************

It is also possible to use this simulation through our software for interfacing with the
real robot. So, you could
access the real TriFinger with the simulation as seen in :doc:`simwithreal`.
You can also access the simulated TriFinger through the real interface, as seen here below.

The Real Robot API
======================

The `robot_interfaces`_ sotware package provides an interface to the hardware
of the real TriFinger robots.  See the documentation there fore more details.


.. _`robot_interfaces with Simulation`:

Using the Simulation through the `robot_interfaces`_ API
===========================================================

It is possible to create a backend of this simulation and have access to all the methods
provided by `robot_interfaces`_ in addition to all the methods provided by ``SimFinger``.
This is done by using the pyBullet
driver in the backend.  By doing this, you only need to replace the backend of the robot
when switching between simulation and real, while all the rest of your code can
remain **exactly** the same.


To create a TriFinger backend using simulation:

.. code-block:: python

    import trifinger_simulation.drivers

    backend = trifinger_simulation.drivers.create_trifinger_backend(
        robot_data, real_time_mode=True, visualize=True
    )

If ``real_time_mode`` is ``True``, the backend will expect a new action every
millisecond and repeat the previous action if it is not provided in time (like
on the real robot).  If set to ``False``, it will run as fast as possible and
wait for new actions.

Set ``visualize=True`` to run the pyBullet GUI for visualization.


For a complete example, see `demo_robot_interface.py`_

.. _`demo_robot_interface.py`: https://github.com/open-dynamic-robot-initiative/trifinger_simulation/blob/master/demos/catkin/demo_robot_interface.py
.. _`robot_interfaces`: https://github.com/open-dynamic-robot-initiative/robot_interfaces
