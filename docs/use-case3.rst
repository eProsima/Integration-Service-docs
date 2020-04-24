Add compatibility to an unsupported protocol
============================================

Another typical scenario encountered when communicating different systems is that they use different protocols,
for example, :code:`DDS` and :code:`ROS1`.

.. image:: DDS_NO_ROS1.png

In such a case, in the absence of the :code:`integration service` tool
the user would need to create a custom :code:`DDS` to :code:`ROS1` bridge that won't be reusable to
communicate either of the two with other protocols.

By using :code:`integration-service` instead, this communication can be achieved with minimum user's effort.
In this specific case, a :code:`ROS1` **System-Handle** already exists, so the communication with :code:`DDS` is
essentially direct.
However, the communication is straightforward enough even if a dedicated **System-Handle** doesn't exist yet, as
the user can create his own **System-Handle**, thus becoming able to communicate with :code:`DDS` and
any other protocol already supported by :code:`integration-service`.
For more information regarding how to generate a **System-Handle** from scratch, please consult the *System-Handle
creation* [TODO: add link] section of the :code:`soss` documentation.

.. image:: DDS_IS_ROS1.png


In the example below we show how :code:`integration-service` bridges a :code:`DDS` application
with a :code:`ROS1` application, by communicating a *HelloWorld* :code:`DDS` application with
the `WritingPublisherSubscriber <http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29>`__
tutorial from :code:`ROS1`.


Example: ROS1 communication
^^^^^^^^^^^^^^^^^^^^^^^^^^^

To prepare the deployment and setup the environment correctly, please follow the introductory steps delined in
:ref:`Getting Started <getting started>` and read carefully the :ref:`Important remarks <important remarks>`
section.

To make this example work, you will require to install the :code:`SOSS-ROS1` **System-Handle**, that you can
download from the dedicated
`SOSS-ROS1 repository <https://github.com/eProsima/soss-ros1/tree/feature/xtypes-support>`__. Clone it into the
workspace where you have :code:`integration service` installed:

.. code-block:: bash

    cd is-workspace
    git clone ssh://git@github.com/eProsima/SOSS-FIWARE src/soss-ros1 -b feature/xtypes-support

And then build the packages by running:

.. code-block:: bash

    colcon build

Finally, source the new colcon overlay:

.. code-block:: bash

    source install/setup.bash

Also, to execute this example you need to have installed:

- :code:`ROS1` melodic.
- The `WritingPublisherSubscriber <http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29>`__
  tutorial compiled and working.

Open three terminals:

- In the first terminal, launch the :code:`ROS1 talker` application:

.. code-block:: bash

    talker

- In the second terminal, execute the :code:`HelloWorldExample` as *subscriber*:

.. code-block:: bash

    HelloWorldExample subscriber

- In the third terminal, execute :code:`integration-service` with the :code:`ros1_dds.yaml` configuration file:

.. code-block:: bash

    soss examples/dds/ros1_dds.yaml

Once :code:`integration-service` is launched, you should see that the :code:`talker` and the :code:`HelloWorldExample`
*subscriber* will start communicating.

If you want to test it the other way around, launch :code:`ROS1 listener`, :code:`HelloWorldExample` as *publisher*,
and :code:`integration-service` with the file :code:`dds_ros1.yaml` instead.

**Note**: Each time you execute :code:`integration-service` with the :code:`soss` command in a new shell,
please make sure to have done the sourcing of the colcon overlay with the command

.. code-block:: bash

    source install/setup.bash

Also, remember to source the :code:`ROS1` insallation in the first and third shells with the command

.. code-block:: bash

    source /opt/ros/melodic/setup.bash

As an alternative, you can add the opportune source commands to the :code:`.bashrc` file.

.. _comment_ros1_1: create the example so the user can test it and verify the ROS1 commands/environment.
    There exists an already created example which comes with a typical ROS1 installation?
