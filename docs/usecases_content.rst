
Use-cases and Examples
======================

In this part of the user-manual, the most representative use-cases demonstrating :code:`integration-service`
functionalities are shown.
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
| :ref:`Add compatibility to an unsupported protocol <add compatibility to an unsupported protocol>` | :ref:`Example: SOME/IP <example: some/ip>`                           |
+----------------------------------------------------------------------------------------------------+----------------------------------------------------------------------+
| :ref:`WAN communication <wan communication>`                                                       | :ref:`Example: WAN TCP tunneling <example: wan tcp tunneling>`       |
+----------------------------------------------------------------------------------------------------+----------------------------------------------------------------------+


Important reminders
^^^^^^^^^^^^^^^^^^^

A compulsory prerequisite for running the examples presented in the following sections is
to have :code:`integration-service` correctly installed as explained in detail
in the introductory section :ref:`Getting Started <getting started>`.
Please make sure to follow all the steps described in the document before proceeding.

Also notice that for being able to execute :code:`integration-service` with the :code:`soss` command at the end of
each example,
the shell must be fully overlaid with the sourcing of any colcon-built package required by the specific
use-case:

 - The :code:`soss` installation, if this has been made by following the installation manual (see the *Getting Started*
   section of the :code:`soss` documentation).
 - The :code:`integration-service` installation, as explained in the :ref:`Getting Started <getting started>` section.
 - The specific **System-Handle** installation required by the example
   (e. g., :code:`SOSS-FIWARE`, :code:`SOSS-SOMPE/IP` ..)

As an alternative, you can install permanently the overlays relevant to your use-case system-wide,
by adding the :code:`source` command of the the :code:`install/setup.bash` file of your local
installations to the :code:`.bashrc` file as:

.. code-block:: bash

    source ~/PATH_TO_FOLDER/install/setup.bash

Where :code:`PATH_TO_FOLDER` is the path to the folder where the packages have been downloaded and built.
