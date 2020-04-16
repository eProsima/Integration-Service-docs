Examples
========

ROS2
^^^^

This examples ties to illustrate the :ref:`DDS Bridge` use-case.

In this example, `integration-service` is going to put into communication a `ROS 2` talker-listener example
with `Fast-RTPS` HelloWorld example.

First of all, the following requirements must be met:

- ROS 2 (Crystal or superior) with talker-listener example working.
- Fast-RTPS (at least v1.9.2) with HelloWorld example.
- `Integration-service` installed with `SOSS-ROS2`.

Open three terminals:

- In the first terminal, execute a `ROS 2` talker

.. code-block:: bash

    ros2 run demo_nodes_cpp talker

- In the second terminal, execute the `Fast-RTPS` HelloWorld subscriber

.. code-block:: bash

    ./HelloWorldExample subscriber

Both application don't communicate due to the incompatibility of their `DDS` configuration, **topic** and **type**.

- In the third terminal, execute `integration-service` using the `dds_ros2_string.yaml` config file located in the
  folder `soss-dds/examples/udp/`.

.. code-block:: bash

    soss soss-dds/examples/udp/dds_ros2_string.yaml

Once `integration-service` is executed, both `DDS` application will start communicating.

If you want to test the other way, launch `ROS 2` listener, HelloWorld publisher and the same `integration-service`
command.

.. _comment_1: Currently, soss-ros2-test is failing to compile, so `std_msgs/String` isn't being generated.
.. _comment_2: Maybe some changes must be done to allow the conversion between the struct types.

Orion Context-Broker
^^^^^^^^^^^^^^^^^^^^

This example integrates a `dds` application into a `Orion Context-Broker` system.

To work this example, it is required an accesible `contextBroker` service, and install `SOSS-FIWARE` along with
`integration-service`. To feed the `contextBroker` the example will use a `Fast-RTPS` HelloWorld publisher,
so this example is needed too.

The file `soss-dds/examples/udp/dds_fiware.yaml` must be edited to match the `contextBroker` configuration
in the testing environment.

Open three terminals (replace <url> with the location of the `contextBroker`):

- In the first terminal, execute the HelloWorld publisher:

.. code-block:: bash

    ./HelloWorldExample publisher

- In the second terminal, create/check the value of the `data` value in the `contextBroker`:

  - The first time testing, the structure for this test must be created in the `contextBroker`:

  .. code-block:: bash

      curl --include \
          --request POST \
          --header "Content-Type: application/json" \
          --data-binary "{ \"type\": \"String\", \"id\": \"String\", \"data\": { \"value\": \"\" } }" \
          '<url>/v2/entities?options='

  - Check the value of the attribute if already exists:

  .. code-block:: bash

      curl <url>/v2/entities/String/attrs/data/value?type=String

  - If the result isn't empty, set the value to empty:

  .. code-block:: bash

      curl <url>/v2/entities/String/attrs/data/value -X PUT -s -S --header 'Content-Type: text/plain' --data-binary \"\"

- Execute `integration-service` in the third terminal with the example YAML:

.. code-block:: bash

    soss soss-dds/examples/udp/dds_fiware.yaml

- Check again the value of the data in the `contextBroker`:

.. code-block:: bash

    curl <url>/v2/entities/String/attrs/data/value?type=String

Now, the value must contain information (normally, "HelloWorld").

If you want to test the other way, launch Helloworld as subscriber and force an update in the `contextBroker` data while
`integration-service` is executing with the same YAML file.

.. _comment_3: Maybe some changes must be done to allow the conversion between the struct types.

Some/IP
^^^^^^^

This example shows how to communicate a radar/fusion `dds` application with `someip` using `soss`.

To execute this example `SOSS-SOMEIP` and `vsomeip` must be installed.
The `SOSS-SOMEIP` examples `simple_radar_fusion_fastdds` and `simple_radar_fusion_vsomeip` must be compiled.
They are located in the `examples/radar_fusion_dds` folder of `SOSS-SOMEIP`.

Open three terminals:

- In the first terminal, launch the `radar` application:

.. code-block:: bash

    radar

- In the second terminal, execute `RadarExample` as subscriber:

.. code-block:: bash

    RadarExample subscriber

- In the third terminal, execute `integration-service` with the `someip_dds.yaml` configuration file:

.. code-block:: bash

    soss examples/radar_fusion_dds/someip_dds.yaml

`radar` and `RadarExample` subscriber should start communicating.

If you want to test the other way, launc `fusion`, `RadarExample` as publisher, and `integration-service` with the file
`dds_someip.yaml` instead.
Take into account, that due to limitations in the **Some/IP** protocol, `radar` and `integration-service` with the
file `dds_someip.yaml` cannot work together because both try to offer the same service.

WAN TCP Tunneling
^^^^^^^^^^^^^^^^^

This example illustrates how to configure `integration-service` to achieve WAN communication.

To test this example properly, you need two separated subnets not connected but with internet access or a testing
environment simulating this scenario (for example, two routers, with one of them acting as ISP of the second).

Route tables and NAT must be configured to ensure proper port redirection before starting the test.

.. figure:: WAN_example.png

    The IP addresses shown are only to illustrate the example, the important information is the **real** public IP of
    the *server* machine, and that its router must enable NAT to forward the listening port to the *server*.

Once the environment is prepared and tested (for example, using a port-scanner), modify the file `wan_config.xml` to
match the IP address and port of with the WAN IP address and forwarded port of your environment.


This test will launch a `ROS 2` talker in the *server* machine, and a `ROS 2` listener in the *client* machine.
In both machines, an `integration-service` instance will communicate with the other using WAN-TCP communication
capabilities of `Fast-RTPS`.

So, the requirements in both machines are:

- ROS 2 (Crystal or superior) with talker-listener example working.
- `Integration-service` installed with `SOSS-ROS2`.

Open 2 terminals in each machine:

In the *server* side:

- Launch `ROS 2` talker example:

.. code-block:: bash

    ros2 run demo_nodes_cpp talker

- Launch `integration-service` with the server yaml:

.. code-block:: bash

    soss example/wan/server.yaml

In the *client* side:

- Launch `ROS 2` listener example:

.. code-block:: bash

    ros2 run demo_nodes_cpp listener

- Launch `integration-service` with the client yaml:

.. code-block:: bash

    soss example/wan/client.yaml

Once both `integration-service` instances match, the talker-listener example will start to communicate.
If the test doesn't work, review carefully your NAT configuration.

.. _comment_4: wan_config.xml
.. _comment_5: create server.yaml and client.yaml both loading wan_config.xml, but different profiles
