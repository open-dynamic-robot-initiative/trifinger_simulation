*************************************************
Welcome to the TriFinger Robot Simulation docs!
*************************************************

.. image:: images/all_finger_types.jpg
   :alt: Screenshots of different (Tri)Finger robots in simulation

The trifinger_simulation_ package contains a PyBullet simulation environment
for the different TriFinger robots as well as helper functions for some tasks
(for sampling goals, computing rewards, etc.).

To learn more about the TriFinger robots, check out the `TriFinger project
website`_ and the corresponding paper_.


.. toctree::
   :maxdepth: 1
   :caption: Getting Started

   getting_started/installation
   getting_started/first_steps
   getting_started/finger_types

.. toctree::
   :maxdepth: 1
   :caption: Ease of Use with a Real Robot

   simreal/simwithreal
   simreal/realwithsim
   simreal/simvsreal
   simreal/timesteplogic

.. toctree::
   :maxdepth: 1
   :caption: API Documentation

   api/sim_finger
   api/trifingerplatform
   api/collision_objects
   api/visual_objects
   api/camera
   api/pinocchio_utils
   api/finger_types_data


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`


Cite Us
=======

If you are using this package in you academic work,
please cite this repository and also the corresponding paper:

.. code-block:: bibtex

   @misc{trifinger-simulation,
      author = {Joshi, Shruti and Widmaier, Felix and Agrawal, Vaibhav and Wüthrich, Manuel},
      year = {2020},
      publisher = {GitHub},
      journal = {GitHub repository},
      howpublished = {\url{https://github.com/open-dynamic-robot-initiative/trifinger_simulation}},
   }

.. code-block:: bibtex

    @conference{wuethrich2020trifinger,
      title = {TriFinger: An Open-Source Robot for Learning Dexterity},
      author = {W{\"u}thrich, M. and Widmaier, F. and Grimminger, F. and Akpo, J. and Joshi, S. and Agrawal, V. and Hammoud, B. and Khadiv, M. and Bogdanovic, M. and Berenz, V. and Viereck, J. and Naveau, M. and Righetti, L. and Sch{\"o}lkopf, B. and Bauer, S.},
      booktitle = {Proceedings of the 4th Conference on Robot Learning (CoRL)},
      month = nov,
      year = {2020},
      doi = {},
      url = {https://corlconf.github.io/corl2020/paper_421/},
      month_numeric = {11}
    }

.. _trifinger_simulation: https://github.com/open-dynamic-robot-initiative/trifinger_simulation
.. _`TriFinger project website`: https://sites.google.com/view/trifinger
.. _paper: https://arxiv.org/abs/2008.03596


----

Last rebuild of this documentation: |today|
