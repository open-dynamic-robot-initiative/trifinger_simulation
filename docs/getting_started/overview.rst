************
Overview
************

The ``trifinger_simulation`` package provides an interface to the TriFinger robots
in simulation using the `pybullet physics engine <https://pypi.org/project/pybullet/>`_.

.. note::
    The TriFinger robots are a little family of kinematically similar robots, including the
    TriFingerOne, the TriFingerEdu, and the TriFingerPro. These robots signify the different iterations
    towards making a low-cost, robust, reproducible, and open-source robotic platform, which is both durable
    against the heavy wear-and-tear of exploration-heavy reinforcement learning algorithms, and also allows
    for precise control. Among these three, the TriFingerEdu is completely open-source and can be built from
    scratch. For details, please refer `here <https://github.com/open-dynamic-robot-initiative/open_robot_actuator_hardware/blob/master/mechanics/tri_finger_edu_v1/README.md>`_.
    You can also find other details on this platform in our `preprint <https://arxiv.org/abs/2008.03596>`_.

    .. list-table:: 

    * - .. figure:: ../images/edu.png
           :width: 620

           Fig 1. TriFingerEdu

    * - .. figure:: ../images/one.png
           :width: 620

           Fig 2. TriFingerOne

    * - .. figure:: ../images/pro.png
           :width: 620

           Fig 3. TriFingerPro

A. Its main components include:

    1. The ``SimFinger`` class: which provides the complete simulation environment for the TriFinger, including all
    methods to control it, interact with it, to set its properties, as well as to setup the world around it.
    For more details on this class, check out :doc:`../api/sim_finger` (API doc).

    2. The ``TriFingerPlatform`` class: which is a wrapper around ``SimFinger`` to provide a similar API as that of the
    real TriFinger platform. For more details on this, please refer to :doc:`../api/trifingerplatform` (API doc),
    and the docs in :ref:`sim-real`.

    3. The ``RealFinger`` class: which is a wrapper around the real robot API to make the real TriFinger robots accessible
    in any gym environment in the same way as the simulated TriFinger robots. For details on this, please refer to :doc:`../simreal/simwithreal`.

    4. A gym-wrapper with two basic environments: for reaching ("TriFingerReach-v0"), and for pushing ("TriFingerPush-v0"),
    and an environment that you can use to perform tasks of varying
    difficulty levels involving manipulation of a cubical object ("TriFingerCubeDifficulty{}-v1"), where the difficulty values could be one of {1, 2, 3, 4}. This environment is from
    the `Real Robot Challenge <https://real-robot-challenge.com/>`_ . For more details on this, please refer `here <https://people.tuebingen.mpg.de/felixwidmaier/realrobotchallenge/simulation_phase/tasks.html>`_.


B. You can also start by looking at some demos illustrating some basic use cases `in here <https://github.com/open-dynamic-robot-initiative/trifinger_simulation/tree/master/demos>`_.
