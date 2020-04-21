Examples
========

A compulsory prerequisite for running the examples below is to have :code:`integration-service` correctly installed
as explained in the introductory section :ref:`Getting Started <getting started>`.
Please make sure to follow all the necessary steps before proceeding.

Also notice that, for being able to execute :code:`integration-service` with the :code:`soss` command,
the shell must be fully overlaid with the sourcing of any colcon-built package required by the specific
use-case:

 - The :code:`soss` installation, if this has been made by following the installation manual (see the *Getting Started*
   section of the :code:`soss` documentation).
 - The :code:`integration-service` installation, as explained in the :ref:`Getting Started <getting started>` section.
 - The specific **System-Handle** installation required by the example
   (e. g., :code:`SOSS-FIWARE`, :code:`SOSS-SOMPE/IP` ..)

ROS2
^^^^

This example is intended to illustrate the :ref:`DDS Bridge` use-case.

In this example, :code:`integration-service` is going to put into communication a :code:`ROS2` *talker-listener*
example with a :code:`Fast-RTPS` *HelloWorld* example.

To get it working, the following requirements must be met:

- Having :code:`ROS2` (Crystal or superior) installed, with the *talker-listener* example working.
- Having :code:`Fast-RTPS` installed (at least v1.9.2), with the *HelloWorld* example working.

**Note**: If you built the :code:`integration-service` and/or :code:`ROS2` packages with colcon, please make sure
to have done all the required sourcing of the colcon overlays or, in alternative, to have added the opportune
source commands to the .bashrc file, as explained in the :ref:`Getting Started <getting started>` section.

Open three terminals:

- In the first terminal, execute a :code:`ROS2` *talker*

.. code-block:: bash

    ros2 run demo_nodes_cpp talker

- In the second terminal, execute the :code:`Fast-RTPS` HelloWorld *subscriber*

.. code-block:: bash

    ./HelloWorldExample subscriber

At this point, the two applications cannot communicate due to the incompatibility of
their **topic** and **type** in their :code:`DDS` configuration. This is where :code:`integration-service` comes
into play to make this communication possible.

- In the third terminal, execute :code:`integration-service` using the :code:`dds_ros2_string.yaml` configuration file
  located in the :code:`soss-dds/examples/udp/` folder.

.. code-block:: bash

    soss soss-dds/examples/udp/dds_ros2_string.yaml

Once the last command is executed, the two :code:`DDS` applications will start communicating.

To test the same communication the other way around,
launch the :code:`ROS2` *listener*, the  HelloWorld *publisher* and the same :code:`soss`
command.

.. _comment_1: Currently, soss-ros2-test is failing to compile, so `std_msgs/String` isn't being generated.
.. _comment_2: Maybe some changes must be done to allow the conversion between the struct types.

Orion Context-Broker
^^^^^^^^^^^^^^^^^^^^

This example integrates a :code:`DDS` application into a :code:`Orion Context-Broker` system.

To make this example work, you will require:

- An accesible :code:`contextBroker` service.
- An installation of the :code:`SOSS-FIWARE` **System-Handle**, that you can download from the dedicated
  `SOSS-FIWARE repository <https://github.com/eProsima/SOSS-FIWARE/tree/feature/xtypes-support>`__).
- An installation of :code:`Fast-RTPS` (at least v1.9.2) with the *HelloWorld* example working. Indeed, in order to feed
  the :code:`contextBroker`, the example will use a :code:`Fast-RTPS` HelloWorld *publisher*.

The file :code:`soss-dds/examples/udp/dds_fiware.yaml` must be edited to match the IP address and port used by the
:code:`contextBroker` configuration in the testing environment.

**Note**: If you built the :code:`integration-service` and/or :code:`SOSS-FIWARE` packages with colcon, please make sure
to have done all the required sourcing of the colcon overlays or, in alternative, to have added the opportune
source commands to the .bashrc file, as explained in the :ref:`Getting Started <getting started>` section.

Open three terminals (replace <url> with the location of the :code:`contextBroker`, 
following the format :code:`http://<ip>:<port>`):

- In the first terminal, execute the HelloWorld *publisher*:

.. code-block:: bash

    ./HelloWorldExample publisher

- In the second terminal, create/check the value of the :code:`data` field in the :code:`contextBroker`:

  - When testing for the first time, the structure for this test must be created in the :code:`contextBroker`:

  .. code-block:: bash

      curl --include \
          --request POST \
          --header "Content-Type: application/json" \
          --data-binary "{ \"type\": \"String\", \"id\": \"String\", \"data\": { \"value\": \"\" } }" \
          '<url>/v2/entities?options='

  - Check the value of the attribute if it already exists:

  .. code-block:: bash

      curl <url>/v2/entities/String/attrs/data/value?type=String

  - If the result isn't empty, set the value to empty:

  .. code-block:: bash

      curl <url>/v2/entities/String/attrs/data/value -X PUT -s -S --header 'Content-Type: text/plain' --data-binary \"\"

- Execute :code:`integration-service` in the third terminal with the YAML example file:

.. code-block:: bash

    soss soss-dds/examples/udp/dds_fiware.yaml

- Check again the value of the data in the `contextBroker`:

.. code-block:: bash

    curl <url>/v2/entities/String/attrs/data/value?type=String

Now, the value must contain information (normally, "HelloWorld").

If you want to test the communication the other way around, launch Helloworld as *subscriber* and force an update
in the :code:`contextBroker` data while :code:`integration-service` is executing with the same YAML file.

.. _comment_3: Maybe some changes must be done to allow the conversion between the struct types.

SOME/IP
^^^^^^^

This example shows how to communicate a *radar/fusion* :code:`DDS` application with :code:`SOME/IP` using 
:code:`integration-service`.

To execute this example you need to have installed:

- :code:`vsomeip`.
- The :code:`SOSS-SOME/IP` **System-Handle**, that you can download from the dedicated SOSS-SOMEIP repository
  [TODO: add link, when public].
  Specifically, you will need the :code:`simple_radar_fusion_fastdds` and :code:`simple_radar_fusion_vsomeip` examples
  compiled. These examples are located in the :code:`examples/radar_fusion_dds` folder.

**Note**: If you built the :code:`integration-service` and/or :code:`SOSS-SOME/IP` packages with colcon, please make
sure to have done all the required sourcing of the colcon overlays or, in alternative, to have added the opportune
source commands to the .bashrc file, as explained in the :ref:`Getting Started <getting started>` section.

Open three terminals:

- In the first terminal, launch the :code:`radar` application:

.. code-block:: bash

    radar

- In the second terminal, execute the :code:`RadarExample` as *subscriber*:

.. code-block:: bash

    RadarExample subscriber

- In the third terminal, execute :code:`integration-service` with the :code:`someip_dds.yaml` configuration file:

.. code-block:: bash

    soss examples/radar_fusion_dds/someip_dds.yaml

Once :code:`soss` is launched, you should see that the :code:`radar` and the :code:`RadarExample` *subscriber*
will start communicating.

If you want to test it the other way around, launch :code:`fusion`, :code:`RadarExample` as *publisher*,
and :code:`integration-service` with the file :code:`dds_someip.yaml` instead.

Take into account that due to limitations in the :code:`SOME/IP` protocol,
:code:`integration-service` doesn't work when executed with :code:`radar` and 
with the file :code:`dds_someip.yaml`, because both try to offer the same service.

WAN TCP Tunneling
^^^^^^^^^^^^^^^^^

This example illustrates how to configure :code:`integration-service` to achieve WAN communication.

To test this example properly, you need two separated subnets that are not connected but both with internet access,
or a testing environment simulating this scenario (for example, two routers, with one of them acting as
ISP of the second).

Notice that route tables and NAT must be configured so as to ensure proper port redirection before starting the test.

.. figure:: WAN_example.png

    The IP addresses shown only serve the purpose of illustrating the example, but the important information is the
    **real** public IP of the *server* machine. Also, its router must enable NAT to forward the listening port to
    the *server*.

Once the environment is prepared and tested (for example, using a port-scanner), modify the file :code:`wan_config.xml`
to match the IP address and port of with the WAN IP address and forwarded port of your environment.


This test will launch a :code:`ROS2` talker in the *server* machine, and a :code:`ROS2` listener in the *client*
machine. In both machines, an :code:`integration-service` instance will communicate with the other using WAN-TCP
communication capabilities of `Fast-RTPS`.

So, the requirement in both machines is to have :code:`ROS2` (Crystal or superior) installed
with a *talker-listener* example working.

**Note**: If you built the :code:`integration-service` and/or :code:`ROS2` packages with colcon, please make
sure to have done all the required sourcing of the colcon overlays or, in alternative, to have added the opportune
source commands to the .bashrc file, as explained in the :ref:`Getting Started <getting started>` section.

Open 2 terminals in each machine:

In the *server* side:

- Launch :code:`ROS2` talker example:

.. code-block:: bash

    ros2 run demo_nodes_cpp talker

- Launch :code:`soss` with the *server* YAML:

.. code-block:: bash

    soss example/wan/server.yaml

In the *client* side:

- Launch :code:`ROS2` listener example:

.. code-block:: bash

    ros2 run demo_nodes_cpp listener

- Launch :code:`soss` with the *client* YAML:

.. code-block:: bash

    soss example/wan/client.yaml

Once the two :code:`integration-service` instances match, the talker-listener example will start to communicate.
If the test doesn't work, review carefully your NAT configuration.

.. _comment_4: wan_config.xml
.. _comment_5: create server.yaml and client.yaml both loading wan_config.xml, but different profiles
