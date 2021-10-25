.. _finger_types:

************
Finger Types
************

The simulation supports different types of (Tri-)Finger robots.  The type has
to be passed to the :class:`~trifinger_simulation.sim_finger.SimFinger` class
upon initialisation.  The different types that are currently supported by the
simulation are described below.  Note that the moving direction of some joints
differ between the types.

The "TriFinger" is basically just an arrangement of three individual fingers,
so it is also possible to use a single finger.  Thus for each type there is
also a single finger version implemented, simply drop the "tri" from the name.


TriFingerPro
============

.. list-table::

   * - .. figure:: ../images/trifingerpro.jpg

          ``finger_type="trifingerpro"``

     - .. figure:: ../images/fingerpro.jpg

          ``finger_type="fingerpro"``


This is the robot that is used in the `Real Robot Challenge`_.  The visuals of
the robot model are close to what the real robot looks like, so it can be used
for rendering of reasonable realistic images (within the limitations of
PyBullet's rendering options).


TriFingerEdu
============

.. list-table::

   * - .. figure:: ../images/trifingeredu.jpg

          ``finger_type="trifingeredu"``

     - .. figure:: ../images/fingeredu.jpg

          ``finger_type="fingeredu"``


The "Edu" version is a more lightweight, cheaper and easier to build version of
the robot.  The hardware documentation is available through the `Open Dynamic
Robot Initiative`_

The robot model is not optimised for realistic rendering, the different fingers
are rendered in different colours to make it easier to distinguish them.


TriFingerOne
============

.. list-table::

   * - .. figure:: ../images/trifingerone.jpg

          ``finger_type="trifingerone"``

     - .. figure:: ../images/fingerone.jpg

          ``finger_type="fingerone"``


(Tri)FingerOne is an early prototype (so to say "version 1") of the robot.

The robot model is not optimised for realistic rendering, the different fingers
are rendered in different colours to make it easier to distinguish them.



.. _Real Robot Challenge: https://real-robot-challenge.com
.. _Open Dynamic Robot Initiative: https://github.com/open-dynamic-robot-initiative/open_robot_actuator_hardware
