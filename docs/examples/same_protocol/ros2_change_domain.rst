.. _ros2_change_of_domain:

ROS 2 Domain ID change
======================

A very typical scenario within the *ROS 2* ecosystem is that of two applications running
under different *ROS 2* domain IDs, so that they are isolated from each other;
however, it might be required to bridge some of the published topics by the first application,
so that a subscriber on the second application, running on a different domain ID can consume this information.
This is where the *Integration Service* plays a fundamental role, by allowing to bridge two *ROS 2* dataspaces easily.

The steps described below allow a *ROS 2* publisher application, running under a certain domain ID,
to communicate with a *ROS 2* subscriber (echo) application, which is running under a different domain ID.

.. image:: images/ros2_domains.png

Requirements
^^^^^^^^^^^^

To prepare the deployment and setup the environment, you need to have *Integration Service* correctly
installed in your system.
To do so, please follow the steps delineated in the :ref:`installation` section.

Also, to get this example working, the following requirements must be met:

* Having **ROS 2** (*Foxy* or superior) installed, with the :code:`talker-listener` example working.

* Having the **Static ROS 2 System Handle** installed. You can download it from the `dedicated repository <https://github.com/eProsima/ROS2-SH>`_
  into the :code:`is-workspace` where you have *Integration Service* installed:

  .. code-block:: bash

    cd ~/is-workspace
    git clone https://github.com/eProsima/ROS2-SH.git src/ROS2-SH

After you have everything correctly installed in your :code:`is-workspace`, build the packages by running:

.. code-block:: bash

    colcon build


Deployment
^^^^^^^^^^

Below we explain how to deploy an example of this use case. To do so, open three terminals:

* In the first terminal, source the *ROS 2* installation and launch the *ROS 2* :code:`pub` application,
  under domain ID **5**:

  .. code-block:: bash

    ROS_DOMAIN_ID=5 ros2 topic pub -r 1 /string_topic std_msgs/String "{data: \"Hello, ROS 2 domain 10\"}"

* In the second terminal, source the *ROS 2* installation and launch the *ROS 2* :code:`echo` application,
  under domain ID **10**:

  .. code-block:: bash

    ROS_DOMAIN_ID=10 ros2 topic echo /string_topic std_msgs/msg/String

Up to this point, no communication should be seen between the publisher and the subscriber, due to the domain ID change.
This is where *Integration Service* comes into play to make the communication possible.

* In the third terminal, go to the :code:`is-workspace` folder, source the local installations,
  and execute *Integration Service* with the :code:`integration-service` command followed by the
  `ros2__domain_id_change.yaml <https://github.com/eProsima/Integration-Service/blob/main/examples/basic/ros2__domain_id_change.yaml>`_
  configuration file located in the :code:`src/Integration-Service/basic` folder:

  .. code-block:: bash

      cd ~/is-workspace
      source install/setup.bash
      integration-service src/Integration-Service/examples/basic/ros2__domain_id_change.yaml

Once the last command is executed, the two applications will start communicating.
