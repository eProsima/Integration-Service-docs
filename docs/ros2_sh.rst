.. _ros2_sh:

ROS 2 System Handle
===================

The *ROS 2 System Handle* can be used for two main purposes:

* Connection between a *ROS 2* application and an application running over a different middleware implementation.
  This is the classic use-case for *Integration Service*.

* Connecting two *ROS 2* applications running under different Domain IDs.

Dependencies
^^^^^^^^^^^^

The only dependency of this *System Handle* is to have a ROS 2 installation (`Foxy <https://docs.ros.org/en/foxy/Installation.html>`_ or superior) in your system.

Configuration
^^^^^^^^^^^^^

Regarding the *ROS 2 System Handle*, there are several specific parameters which can be configured
for the *ROS 2* middleware. All of these parameters are optional, and are suboptions of the main
five sections:

* :code:`systems`: The system :code:`type` must be :code:`ros2`. In addition to the :code:`type` and :code:`types-from` fields,
  the *ROS 2 System Handle* accepts the following specific configuration fields:

  .. code-block:: yaml
  
      systems:
        ros2:
          type: ros2
          namespace: "/"
          node_name: "my_ros2_node"
          domain: 4

  * :code:`namespace`: The *namespace* of the ROS 2 node created by the *ROS 2 System Handle*.

  * :code:`node_name`: The *ROS 2 System Handle* node name.

  * :code:`domain`: Provides with an easy way to change the *Domain ID* of the ROS 2 entities created
    by the *ROS 2 System Handle*.

Examples
^^^^^^^^

There are three examples that you can find in this documentation in which the *ROS 2 System Handle* is employed in the communication:

* :ref:`ros1-ros2_bridge`
* :ref:`dds-ros2_bridge`
* :ref:`ros2_change_of_domain`

Compilation flags
^^^^^^^^^^^^^^^^^


Besides the :ref:`global_compilation_flags` available for the
whole *Integration Service* product suite, there are some specific flags which apply only to the
*ROS 2 System Handle*; they are listed below:

* :code:`BUILD_ROS2_TESTS`: Allows to specifically compile the *ROS 2 System Handle* unitary and
  integration tests. It is useful to avoid compiling each *System Handle*'section test suite present
  in the :code:`colcon` workspace, which is what would happen if using the :code:`BUILD_TESTS` flag, with the objective of minimizing building time. To use it, after making sure that the *ROS 2 System Handle*
  is present in your :code:`colcon` workspace, execute the following command:
  
  .. code-block:: bash
  
      ~/is_ws$ colcon build --cmake-args -DBUILD_ROS2_TESTS=ON

* :code:`MIX_ROS_PACKAGES`: It accepts as an argument a list of `ROS 2 packages <https://index.ros.org/packages/>`_,
  such as :code:`std_msgs`, :code:`geometry_msgs`, :code:`sensor_msgs`, :code:`nav_msgs`... for which the required transformation
  library to convert the specific *ROS 2* type definitions into *xTypes*, and the other way around, will be built.
  This library is also known within the *Integration Service* context as :code:`Middleware Interface Extension`
  or :code:`mix` library.

  By default, only the :code:`std_msgs_mix` library is compiled, unless the :code:`BUILD_TESTS`
  or :code:`BUILD_ROS2_TESTS` is used, case in which some additional ROS 2 packages :code:`mix` files
  required for testing will be built.

  If the user wants to compile some additional packages to use them with *Integration Service*,
  the following command must be launched to compile it, adding as many packages to the list as desired:

  .. code-block:: bash
  
      ~/is_ws$ colcon build --cmake-args -DMIX_ROS_PACKAGES="std_msgs geometry_msgs sensor_msgs nav_msgs"

.. TODO: complete when it is uploaded to read the docs

.. API Reference
.. ^^^^^^^^^^^^^
