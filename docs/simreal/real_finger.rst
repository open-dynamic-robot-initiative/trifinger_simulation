.. _`RealFinger`:

**********************
The RealFinger
**********************

The :class:`~trifinger_simulation.RealFinger` class provides a very straightforward way of calling the
frontend methods of the real TriFinger robot. 

Inside ``RealFinger``, the robot actually gets created like this:

.. code-block:: python

    import robot_interfaces
    import robot_fingers

    robot_config_path = "path_to_the_config_yml_of_the_desired_robot_type"

    robot_data = robot_interfaces.trifinger.SingleProcessData()

    robot_backend = robot_fingers.create_trifinger_backend(
                robot_data, robot_config_path)
    robot = robot_interfaces.trifinger.Frontend(robot_data)