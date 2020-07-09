Use-cases and Examples
======================

.. _typical_use_cases:

Typical Use-cases
^^^^^^^^^^^^^^^^^

Typical scenarios in which *eProsima Integration-Service* is relevant are:

* Communication of different *DDS*-based systems that use incompatible configurations.
* Communication of a *DDS* system with systems with incompatible protocols.
* Integrating *DDS* systems into arbitrarily complex systems using different protocols.
* Communication between *DDS* systems located in different geographical regions through the Internet.

In this user manual we discuss representative use-cases demonstrating these
*eProsima Integration-Service*'s functionalities.
For each use-case, a related example is presented and the user is guided step-by-step through the
installation protocol and environment preparation necessary to have the examples set up and working.

Namely, we will go through the following:

.. list-table::
   :header-rows: 1
   :align: left

   * - Use-cases
     - Examples
   * - :ref:`dds_bridge`
     - :ref:`ros2_comm`
   * - :ref:`unsupp_protocol`
     - :ref:`ros1_comm`
   * - :ref:`integrate_large_system`
     - :ref:`orion`
   * - :ref:`wan_comm`
     - :ref:`wan_tcp_tunneling`

.. _important_remarks:

Important remarks
^^^^^^^^^^^^^^^^^

A compulsory prerequisite for running the examples of the following sections is
to have *eProsima Integration-Service* correctly installed as explained
in the introductory section :ref:`Getting Started <getting_started>`.
Please make sure to follow all the steps described in the document before proceeding.

Whenever you run the :code:`colcon build` command in the examples provided, if any package is missing dependencies
**causing the compilation to fail**, you can add the flag :code:`--packages-up-to soss-dds-test` as follows:

.. code-block:: bash

    colcon build --packages-up-to soss-dds-test

Also notice that, for being able to execute *eProsima Integration-Service* with the :code:`soss` command at the end of
each example, the shell must be fully overlaid with the sourcing of all colcon-built packages required by the specific
use-case:

- The *eProsima Integration-Service* installation, along with that of any possible **System-Handle** that
  might be required by the specific example (e. g., **SOSS-FIWARE** and **SOSS-ROS1**).
- The *ROS2* or *ROS1* installation, when needed.

As an alternative, you can install permanently the overlays relevant to your use-case system-wide,
by adding the opportune :code:`source` commands to your :code:`.bashrc` file.
