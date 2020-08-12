*************************************************
Welcome to the TriFinger Robot Simulation docs!
*************************************************

.. image:: images/multi_object.JPG

This is the accompanying documentation of the `trifinger_simulation` package_.

To know more about the TriFinger robots, check out our official `project website`_, and the preprint_
of this work.

.. note::

   The Real Robot Challenge is currently in progress! Please refer to the official site of the
   `Real Robot Challenge`_ for more details!
   Also, note that this is *not* the challenge simulator. The challenge simulator can be found on
   `rrc_simulation`_.

Dive in!

.. toctree::
   :maxdepth: 2
   :caption: Getting Started

   Installation <installation>

.. add basic usage example here

.. toctree::
   :maxdepth: 2
   :caption: Ease of Use with a Real Robot
   
   Simulation vs Real Robot <simvsreal>
   Interface with a Real Robot <driverinterface>

.. toctree::
   :maxdepth: 1
   :caption: API Documentation

   The SimFinger Class <sim_finger>
   The TriFingerPlatform Class <trifingerplatform>


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

.. youtube:: FDWyIdC1EEQ

Cite Us!
==============

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

   @misc{wthrich2020trifinger,
      title={TriFinger: An Open-Source Robot for Learning Dexterity},
      author={Manuel Wüthrich and Felix Widmaier and Felix Grimminger and Joel Akpo and Shruti Joshi and Vaibhav Agrawal and Bilal Hammoud and Majid Khadiv and Miroslav Bogdanovic and Vincent Berenz and Julian Viereck and Maximilien Naveau and Ludovic Righetti and Bernhard Schölkopf and Stefan Bauer},
      year={2020},
      eprint={2008.03596},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
   }

.. _package: https://github.com/open-dynamic-robot-initiative/trifinger_simulation
.. _`project website`: https://sites.google.com/view/trifinger
.. _preprint: https://arxiv.org/abs/2008.03596
.. _`Real Robot Challenge`: https://people.tuebingen.mpg.de/felixwidmaier/realrobotchallenge/
.. _`rrc_simulation`: https://github.com/rr-learning/rrc_simulation
