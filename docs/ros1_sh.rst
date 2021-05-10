.. _ros1_sh:

ROS 1 System Handle
===================

The main purpose of the *ROS 1 System Handle* is that of establishing a connection between a *ROS 1* application and an application running over a different middleware implementation. This is the classic use-case for Integration Service.

Dependencies
^^^^^^^^^^^^

The only dependency of this *System Handle* is to have a `ROS 1 installation <http://wiki.ros.org/ROS/Installation>`_ (`Melodic <http://wiki.ros.org/melodic/Installation>`_ or `Noetic <http://wiki.ros.org/noetic/Installation>`_) in your system.

Configuration
^^^^^^^^^^^^^

Regarding the *ROS 1 System Handle*, there are several specific parameters which can be configured
for the *ROS 1* middleware. All of these parameters are optional, and are suboptions of the main
five sections:

* :code:`systems`: The system :code:`type` must be :code:`ros1`. In addition to the :code:`type` and :code:`types-from` fields,
  the *ROS 1 System Handle* accepts the following specific configuration fields:

  .. code-block:: yaml
  
      systems:
        ros1:
          type: ros1
          node_name: "my_ros1_node"
  
  * :code:`node_name`: The *ROS 1 System Handle* node name.

* :code:`topics`: The topic :code:`route` must contain :code:`ros1` within its :code:`from` or :code:`to` fields. Additionally,
  the *ROS 1 System Handle* accepts the following topic specific configuration parameters, within the
  :code:`ros1` specific middleware configuration tag:

  .. code-block:: yaml
  
      routes:
        ros2_to_ros1: { from: ros2, to: ros1 }
        ros1_to_dds: { from: ros1, to: dds }

      topics:
        hello_ros1:
          type: std_msgs/String
          route: ros2_to_ros1
          ros1: { queue_size: 10, latch: false }
        hello_dds:
          type: std_msgs/String
          route: ros1_to_dds
          ros1: { queue_size: 5 }

  * :code:`queue_size`: The maximum message queue size for the *ROS 1* publisher or subscription.
  * :code:`latch`: Enable or disable latching. When a connection is latched,
    the last message published is saved and sent to any future subscribers that connect.
    This configuration parameter only makes sense for *ROS 1* publishers, so it is only useful for
    routes where the *ROS 1 System Handle* acts as a publisher, that is, for routes where :code:`ros1` is
    included in the :code:`to` list.

Examples
^^^^^^^^

There is one example that you can find in this documentation in which the *ROS 1 System Handle* is employed in the communication:

* :ref:`ros1-ros2_bridge`


Compilation flags
^^^^^^^^^^^^^^^^^

Besides the :ref:`global_compilation_flags` available for the
whole *Integration Service* product suite, there are some specific flags which apply only to the
*ROS 1 System Handle*; they are listed below:

* :code:`BUILD_ROS1_TESTS`: Allows to specifically compile the *ROS 1 System Handle* unitary and
  integration tests. It is useful to avoid compiling each *System Handle*'section test suite present
  in the :code:`colcon` workspace, which is what would happen if using the :code:`BUILD_TESTS` flag, with the objective of minimizing building time. To use it, after making sure that the *ROS 1 System Handle*
  is present in your :code:`colcon` workspace, execute the following command:
  
  .. code-block:: bash

      ~/is_ws$ colcon build --cmake-args -DBUILD_ROS1_TESTS=ON

* :code:`MIX_ROS_PACKAGES`: It accepts as an argument a list of `ROS 1 packages <https://index.ros.org/packages/>`,
  such as :code:`std_msgs`, :code:`geometry_msgs`, :code:`sensor_msgs`, :code:`nav_msgs`... for which the required transformation
  library to convert the specific *ROS 1* type definitions into *xTypes*, and the other way around, will be built.
  This library is also known within the *Integration Service* context as :code:`Middleware Interface Extension`
  or :code:`mix` library.

  By default, only the :code:`std_msgs_mix` library is compiled, unless the :code:`BUILD_TESTS`
  or :code:`BUILD_ROS1_TESTS` is used, case in which some additional ROS 1 packages :code:`mix` files
  required for testing will be built.

  If an user wants to compile some additional packages to use them with *Integration Service*,
  the following command must be launched to compile it, adding as much packages to the list as desired:

  .. code-block:: bash

      ~/is_ws$ colcon build --cmake-args -DMIX_ROS_PACKAGES="std_msgs geometry_msgs sensor_msgs nav_msgs"


.. TODO: complete when it is uploaded to read the docs

.. API Reference
.. ^^^^^^^^^^^^^
