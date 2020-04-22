Add compatibility to an unsupported protocol
============================================

Another typical scenario encountered when communicating different systems is that they use different protocols,
for example, :code:`DDS` and :code:`ROS 1`.

.. image:: DDS_NO_ROS1.png

In such a case, in the absence of the :code:`integration service` tool
the user would need to create a custom :code:`DDS` to :code:`ROS 1` bridge that will not be reusable to
communicate neither of the two with other protocols.

By using :code:`integration-service` instead, this communication can be achieved with minimum user's effort.
In this specific case, a :code:`ROS 1` **System-Handle** already exists, so the communication with :code:`DDS` is
essentially direct.
However, the communication is straightforward enough even if a dedicated **System-Handle** doesn't exist yet, as
the user can create his own **System-Handle**, thus becoming able to communicate with :code:`DDS` and
any other protocol already supported by :code:`integration-service`.
For more information regarding how to generate a **System-Handle** from scratch, please consult the *System-Handle
creation* [TODO: add link] section of the :code:`soss` documentation.

.. image:: DDS_IS_ROS1.png


In the example below we show how :code:`integration-service` bridges a :code:`DDS` application
with a :code:`ROS 1` application, by communicating a *HelloWorld* :code:`DDS` application with
the `WritingPublisherSubscriber <http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29>`__
tutorial from :code:`ROS 1`.


Example: ROS1 communication
^^^^^^^^^^^^^^^^^^^^^^^^^^^

To prepare the system and setup the environment correctly, please follow the introductory steps delined in
:ref:`Getting Started <getting started>` and read carefully the :ref:`Important reminders <important reminders>`
section.

Also, to execute this example you need to have installed:

- :code:`ROS 1` melodic.
- The :code:`SOSS-ROS1` **System-Handle**, that you can download from the dedicated
  `SOSS-ROS1 repository <https://github.com/eProsima/soss-ros1>`__.
- The `WritingPublisherSubscriber <http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29>`__
  tutorial compiled and working.

**Note**: If you built the :code:`integration-service` and/or :code:`SOSS-ROS1` packages with colcon, please make
sure to have done all the required sourcing of the colcon overlays or, in alternative, to have added the opportune
source commands to the .bashrc file, as explained in the :ref:`Getting Started <getting started>` section.

Open three terminals:

- In the first terminal, launch the :code:`ROS 1 talker` application:

.. code-block:: bash

    talker

- In the second terminal, execute the :code:`HelloWorldExample` as *subscriber*:

.. code-block:: bash

    HelloWorldExample subscriber

- In the third terminal, execute :code:`integration-service` with the :code:`ros1_dds.yaml` configuration file:

.. code-block:: bash

    soss examples/dds/ros1_dds.yaml

Once :code:`soss` is launched, you should see that the :code:`talker` and the :code:`HelloWorldExample` *subscriber*
will start communicating.

If you want to test it the other way around, launch :code:`ROS 1 listener`, :code:`HelloWorldExample` as *publisher*,
and :code:`integration-service` with the file :code:`dds_ros1.yaml` instead.

.. _comment_ros1_1: create the example so the user can test it and verify the ROS1 commands/environment.
    There exists an already created example which comes with a typical ROS1 installation?
