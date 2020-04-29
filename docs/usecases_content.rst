Use-cases and Examples
======================

User Manual organization
^^^^^^^^^^^^^^^^^^^^^^^^

In this part of the user manual we discuss the most representative use-cases demonstrating
*eProsima Integration-Service*'s functionalities.
For each use-case, a related example is presented and the user is guided step-by-step through the
installation protocol and environment preparation necessary to have the examples set up and working.

Namely, we will go through the following:

+----------------------------------------------------------------------------------------------------+----------------------------------------------------------------------+
| Use-cases                                                                                          | Examples                                                             |
+====================================================================================================+======================================================================+
| :ref:`DDS bridge <dds bridge>`                                                                     | :ref:`Example: ROS2 communication <example: ros2 communication>`     |
+----------------------------------------------------------------------------------------------------+----------------------------------------------------------------------+
| :ref:`Integrate a large system <integrate a large system>`                                         | :ref:`Example: Orion Context-Broker <example: orion context-broker>` |
+----------------------------------------------------------------------------------------------------+----------------------------------------------------------------------+
| :ref:`Add compatibility to an unsupported protocol <add compatibility to an unsupported protocol>` | :ref:`Example: ROS1 communication <example: ros1 communication>`     |
+----------------------------------------------------------------------------------------------------+----------------------------------------------------------------------+
| :ref:`WAN communication <wan communication>`                                                       | :ref:`Example: WAN TCP tunneling <example: wan tcp tunneling>`       |
+----------------------------------------------------------------------------------------------------+----------------------------------------------------------------------+


Important remarks
^^^^^^^^^^^^^^^^^

A compulsory prerequisite for running the examples of the following sections is
to have *eProsima Integration-Service* correctly installed as explained in detail
in the introductory section :ref:`Getting Started <getting started>`.
Please make sure to follow all the steps described in the document before proceeding.

Also notice that, for being able to execute *eProsima Integration-Service* with the :code:`soss` command at the end of
each example,
the shell must be fully overlaid with the sourcing of any colcon-built package required by the specific
use-case:

- The *eProsima Integration-Service* installation, along with the installation of any possible **System-Handle** that
  might be required by the specific example (e. g., **SOSS-FIWARE** and **SOSS-ROS1**).
- The *ROS2* or *ROS1* installation, when needed.

As an alternative, you can install permanently the overlays relevant to your use-case system-wide,
by adding the opportune :code:`source` commands to your :code:`.bashrc` file.
