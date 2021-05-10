.. _existing_shs:

Existing System Handles
=======================

This section provides an insight over the *System Handles* existing and tested at the time of writing, and it is kept constantly up to date in order to include any new *System Handle* or new features of those already implemented 
(such as e.g. the extension of the types supported or the addition/removal of an external dependency).

For each, we detail the fields of the YAML configuration file which are compulsory to fill when the associated middleware is involved in an *Integration Service*-mediated communication. 
We also provide a list of the external dependencies, a step-by-step guide of how to compile tests and examples, and any other *System Handle*-specific configurational detail.
It is organized as follows:

* :ref:`dds_sh`
* :ref:`ros2_sh`
* :ref:`ros1_sh`
* :ref:`websocket_sh`
* :ref:`fiware_sh`


.. _dds_sh:

Fast DDS System Handle
^^^^^^^^^^^^^^^^^^^^^^

The *Fast DDS System Handle* can be used for three main purposes:

* Connection between a *DDS* application and an application running over a different middleware implementation.
  This is the classic use-case for *Integration Service*.

* Connecting two *DDS* applications running under different Domain IDs.

* Creating a *TCP tunnel*, by running an *Integration Service* instance on each of the
  machines you want to establish a communication between.

Dependencies
------------

The only dependency of this *System Handle* is to have a `Fast DDS installation <https://fast-dds.docs.eprosima.com/en/latest/installation/binaries/binaries_linux.html>`_ (v2.0.0 or superior) in your system.

.. note::

    The *Fast DDS System Handle* requires an installation of *Fast DDS* to work. The *System Handle* first looks into the system for a previous installation of *Fast DDS* v2.0.0 or superior. If it doesn't find one, 
    it downloads and installs its own version.


Configuration
-------------

Regarding the *Fast DDS System Handle*, there are several specific parameters which can be configured
for the *DDS* middleware. All of these parameters are optional, and are suboptions of the main
five sections:

* :code:`systems`: The system :code:`type` must be :code:`fastdds`. In addition to the :code:`type` and :code:`types-from` fields,
  the *Fast DDS System Handle*.

  .. code-block:: yaml
  
      systems:
          dds:
          type: fastdds
          participant:
              domain_id: 3
              file_path: <path_to_xml_profiles_file>.xml
              profile_name: fastdds-sh-participant-profile


   This section accepts the following specific configuration fields for the *Fast DDS System Handle*:

  * :code:`participant`: Allows to add a specific configuration for the [Fast DDS DomainParticipant](https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/domain/domainParticipant/domainParticipant.html):

    * :code:`domain_id`: Provides an easy way to change the *Domain ID* of the DDS entities created
      by the *Fast DDS System Handle*.

    * :code:`file_path`: Path to an XML file, containing a configuration profile for the System Handle
      participant. More information about Fast DDS XML profiles and how to fully customize the
      properties of DDS entities through them is available [here](https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/xml_configuration.html).

    * :code:`profile_name`: Within the provided XML file, the name of the XML profile associated to the
      *Integration Service Fast DDS System Handle* participant.

Examples
--------

There are three examples that you can find in this documentation in which the *Fast DDS System Handle* is employed in the communication:

* :ref:`Publisher/subscriber intercommunication between Fast DDS and ROS 2 <dds-ros2_bridge>`
* :ref:`Bridging the communication between two DDS data spaces under different Domain IDs <dds_change_of_domain>`
* :ref:`Using Fast DDS TCP WAN tunneling to communicate two applications running on different networks <wan_tcp_tunneling>`

Compilation flags
-----------------

Besides the :ref:`global_compilation_flags` available for the
whole *Integration Service* product suite, there are some specific flags which apply only to the
*Fast DDS System Handle*. They are listed below:

* :code:`BUILD_FASTDDS_TESTS`: Allows to specifically compile the *Fast DDS System Handle* unitary and
  integration tests. It is useful to avoid compiling each *System Handle*'section test suite present
  in the :code:`colcon` workspace, which is what would happen if using the :code:`BUILD_TESTS` flag, with the objective of minimizing building time. To use it, after making sure that the *Fast DDS System Handle*
  is present in your :code:`colcon` workspace, execute the following command:
  
  .. code-block:: bash

      ~/is_ws$ colcon build --cmake-args -DBUILD_FASTDDS_TESTS=ON

.. TODO: complete when it is uploaded to read the docs

API Reference
-------------


.. _ros2_sh:

ROS 2 System Handle
^^^^^^^^^^^^^^^^^^^

The *ROS 2 System Handle* can be used for two main purposes:

* Connection between a *ROS 2* application and an application running over a different middleware implementation.
  This is the classic use-case for *Integration Service*.

* Connecting two *ROS 2* applications running under different Domain IDs.

Dependencies
------------

The only dependency of this *System Handle* is to have a ROS 2 installation (`Foxy <https://docs.ros.org/en/foxy/Installation.html>`_ or superior) in your system.

Configuration
-------------

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
--------

There are three examples that you can find in this documentation in which the *ROS 2 System Handle* is employed in the communication:

* :ref:`Publisher/subscriber intercommunication between ROS 2 and ROS 1 <ros1-ros2_bridge>`
* :ref:`Publisher/subscriber intercommunication between Fast DDS and ROS 2 <dds-ros2_bridge>`
* :ref:`Bridging the communication between two ROS 2 data spaces under different Domain IDs <ros2_change_of_domain>`

Compilation flags
-----------------


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

API Reference
-------------


.. _ros1_sh:

ROS 1 System Handle
^^^^^^^^^^^^^^^^^^^

The main purpose of the *ROS 1 System Handle* is that of establishing a connection between a *ROS 1* application and an application running over a different middleware implementation. This is the classic use-case for Integration Service.

Dependencies
------------

The only dependency of this *System Handle* is to have a `ROS 1 installation <http://wiki.ros.org/ROS/Installation>`_ (`Melodic <http://wiki.ros.org/melodic/Installation>`_ or `Noetic <http://wiki.ros.org/noetic/Installation>`_) in your system.

Configuration
-------------

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
--------

There is one example that you can find in this documentation in which the *ROS 1 System Handle* is employed in the communication:

* :ref:`Publisher/subscriber intercommunication between ROS 2 and ROS 1 <ros1-ros2_bridge>`


Compilation flags
-----------------

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

API Reference
-------------
-------------


.. _websocket_sh:

WebSocket System Handle
^^^^^^^^^^^^^^^^^^^^^^^

This repository contains the source code of *Integration Service System Handle*
for the `WebSocket <https://www.websocket.org/>` middleware protocol, widely used in the robotics field.
The main purpose of the *WebSocket System Handle* is that of establishing a connection between a *WebSocket*
application and an application running over a different middleware implementation.
This is the classic use-case for *Integration Service*.

Dependencies
------------

The dependencies of the *WebSocket System Handle* are:

* `OpenSSL <https://www.openssl.org/>`_
* `WebSocket++ <https://github.com/zaphoyd/websocketpp>`_

Configuration
-------------

Regarding the *WebSocket System Handle*, there are several specific parameters which can be configured
for the WebSocket middleware. All of these parameters are suboptions of the main
five sections:

* :code:`systems`: The system :code:`type` must be :code:`websocket_server` or :code:`websocket_client`. In addition to the
  :code:`type` and :code:`types-from` fields, the *WebSocket System Handle* accepts a wide variety of specific
  configuration fields, depending on the selected operation mode (*Client* or *Server*).

  For the :code:`websocket_server` *System Handle*, there are two possible configuration scenarios:
  the former one uses a TLS endpoint, and the latter uses a TCP endpoint.

  **TLS**
  
  .. code-block::yaml
  
      systems:
        websocket:
          type: websocket_server
          port: 80
          cert: path/to/cert/file.crt
          key: path/to/key/file.key
          authentication:
           policies: [
               { secret: this-is-a-secret, algo: HS256, rules: {example: *regex*} }
           ]

  **TCP**

  .. code-block::yaml
  
      systems:
        websocket:
          type: websocket_server
          port: 80
          security: none
          encoding: json

    * :code:`port`: The specific port where the *server* will listen for incoming connections. This field is
      required.
    * :code:`security`: If this field is not present, a secure TLS endpoint will be created. If the special
      value :code:`none` is written, a TCP *WebSocket server* will be set up.
    * :code:`cert`: The *X.509* certificate that the *server* should use. This field is mandatory if
      :code:`security` is enabled.
    * :code:`key`: A path to the file containing the public key used to verify credentials with the specified
      certificate. If :code:`security` is enabled, this field must exist and must be filled in properly.
    * :code:`authentication`: It is a list of :code:`policies`. Each policy accepts the following keys:
      * :code:`secret`: When using **MAC** *(Message Authentication Code)* method for verification,
        this field allows to set the secret used to authenticate the client requesting a connection to the server.
      * :code:`pubkey`: Path to a file containing a **PEM** encoded public key.

      **NOTE:** Either a `secret` or a `pubkey` is required.

      * :code:`rules`: List of additional claims that should be checked. It should contain a map with keys
        corresponding to the claim identifier, and values corresponding to regex patterns that should match
        the payload's value. In the example above, the rule will check that the payload contains
        an :code:`example` claim and that its value contains the *regex* keyword in any position of the mesage. This field is optional.
      * :code:`algo`: The algorithm that should be used for encrypting the connection token. If the incoming token
        is not encrypted with the same algorithm, it will be discarded. If not specified, the HS256 algorithm will be used.
    * :code:`encoding`: Specifies the protocol, built over JSON, that allows users to exchange useful information
      between the client and the server, by means of specifying which keys are valid for the JSON
      sent/received messages and how they should be formatted for the server to accept and process these
      messages. By default, :code:`json` encoding is provided in the *WebSocket System Handle* and used
      if not specified otherwise. Users can implement their own encoding by implementing the
      `Encoding class <src/Encoding.hpp>`_.

    For the :code:`websocket_client` *System Handle*, there are also two possible configuration scenarios:
    using TLS or TCP.

    **TLS**
    
    .. code-block:: yaml

        systems:
          websocket:
            type: websocket_client
            host: localhost
            port: 80
            cert_authorities: [my_cert_authority.ca.crt]
            authentication:
                token: eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.ey...

    **TCP**
    
    .. code-block:: yaml
    
        systems:
          websocket:
            type: websocket_client
            port: 80
            security: none
            encoding: json
            authentication:
                token: eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.ey...

    * :code:`port`: The specific port where the *client* will attempt to establish a connection to a
      *WebSocket server*. This field is mandatory.
    * :code:`host`: Address where the *WebSocket server* is hosted. If not specified, it will use
      :code:`localhost` as the default value.
    * :code:`security`: If this field is not present, a secure TLS endpoint will be created. If the special
      value :code:`none` is written, a TCP *WebSocket client* will be set up.
    * :code:`cert_authorities`: List of *certificate authorities* used to validate the client against the
      server. This field is optional and only applicable if :code:`security` is not disabled.
    * :code:`authentication`: allows to specify the public :code:`token` used to perform the secure authentication process
      with the server. This field is mandatory.
    * :code:`encoding`: Specifies the protocol, built over JSON, that allows users to exchange useful information
      between the client and the server, by means of specifying which keys are valid for the JSON
      sent/received messages and how they should be formatted for the server to accept and process these
      messages. By default, :code:`json` encoding is provided in the *WebSocket System Handle* and used
      if not specified otherwise. Users can implement their own encoding by implementing the
      `Encoding class <src/Encoding.hpp>`_.


JSON encoding protocol
----------------------

In order to communicate with the *WebSocket System Handle* using the JSON encoding, the messages should follow a specific pattern. This pattern will be different depending on the paradigm used for the connection (*pub/sub* or *client/server*) and the communication purpose.

Several fields can be used in those messages, but not all of them are mandatory. All of them will be described in this section, as well as in which cases they are optional:

* :code:`op`: The *Operation Code* is mandatory in every communication as it specifies the purpose of the message.
  This field can assume nine different values,   which are the ones detailed below.
  * :code:`advertise`: It notifies that there is a new publisher that is going to publish messages on a specific topic.
  The fields that can be set for this operation are: :code:`topic`, :code:`type` and optionally the :code:`id`.

    .. code-block::JSON
        {"op": "advertise", "topic": "helloworld", "type": "HelloWorld", "id": "1"}

  * :code:`unadvertise`: It states that a publisher is not going to publish any more messages on a specific topic.
    The fields that can be set for this operation are: :code:`topic` and optionally the :code:`id`.

    .. code-block::JSON
        {"op": "unadvertise", "topic": "helloworld", "id": "1"}

  * :code:`publish`: It identifies a message that wants to be published over a specific topic.
    The fields that can be set for this operation are: :code:`topic` and :code:`msg`.

    .. code-block::JSON

        {"op": "publish", "topic": "helloworld", "msg": {"data": "Hello"}}

  * :code:`subscribe`: It notifies that a subscriber wants to receive the messages published under a specific topic.
    The fields that can be set for this operation are: :code:`topic` and optionally the :code:`id` and :code:`type`.

    .. code-block::JSON
        {"op": "subscribe", "topic": "helloworld", "type": "HelloWorld", "id": "1"}

  * :code:`unsubscribe`: It states that a subscriber doesn't want to receive messages from a specific topic anymore.
    The fields that can be set for this operation are: :code:`topic` and optionally the :code:`id`.

    .. code-block::JSON
        {"op": "unsubscribe", "topic": "helloworld", "id": "1"}

  * :code:`call_service`: It identifies a message request that wants to be published on a specific service.
    The fields that can be set for this operation are: :code:`service`, :code:`args` and optionally the :code:`id`.

    .. code-block::JSON
        {"op": "call_service", "service": "hello_serv", "args": {"req": "req"}, "id": "1"}

  * :code:`advertise_service`: It notifies that a new server is going to attend to the requests done on a specific service.
    The fields that can be set for this operation are: :code:`request_type`, :code:`reply_type` and :code:`service`.

    .. code-block::JSON
        {"op": "advertise_service", "service": "hello_serv", "request_type":
         "HelloRequest", "reply_type": "HelloReply"}

  * :code:`unadvertise_service`: It states that a server is not going to attend any more the requests done on a specific service.
    The fields that can be set for this operation are: :code:`type` and :code:`service`.

    .. code-block::JSON
        {"op": "unadvertise_service", "service": "hello_serv", "type": "HelloReply"}
  
  * :code:`service_response`: It identifies a message reply that wants to be published as response to a specific request.
    The fields that can be set for this operation are: :code:`service`, :code:`values` and optionally the :code:`id`.

    .. code-block::JSON
        {"op": "service_response", "service": "hello_serv", "values": {"resp": "resp"}, 
         "id": "1"}
  
* :code:`id`: Code that identifies the message.
* :code:`topic`: Name that identifies a specific topic.
* :code:`type`: Name of the type that wants to be used for publishing messages on a specific topic.
* :code:`request_type`: Name of the type that wants to be used for the service requests.
* :code:`reply_type`: Name of the type that wants to be used for the service responses.
* :code:`msg`: Message that is going to be published under a specific topic.
* :code:`service`: Name that identifies a specific service.
* :code:`args`: Message that is going to be published under a specific service as a request.
* :code:`values`: Message that is going to be published under a specific service as a response.
* :code:`result`: Value that states if the request has been successful.

Examples
--------

There is one example that you can find in this documentation in which the *WebSocket System Handle* is employed in the communication:

* :ref:`Publisher/subscriber intercommunication between WebSocket and ROS 2 <ros2-websocket_comm>`

Compilation flags
-----------------

Besides the :ref:`global_compilation_flags` available for the
whole *Integration Service* product suite, there are some specific flags which apply only to the
*WebSocket System Handle*; they are listed below:

* `BUILD_WEBSOCKET_TESTS`: Allows to specifically compile the *WebSocket System Handle* unitary and
  integration tests. It is useful to avoid compiling each *System Handle*'section test suite present
  in the :code:`colcon` workspace, which is what would happen if using the :code:`BUILD_TESTS` flag, with the objective of minimizing building time. To use it, after making sure that the *WebSocket System Handle*
  is present in your :code:`colcon` workspace, execute the following command:
  
  .. code-block:: bash
      
      ~/is_ws$ colcon build --cmake-args -DBUILD_WEBSOCKET_TESTS=ON


.. TODO: complete when it is uploaded to read the docs

API Reference
-------------


.. _fiware_sh:

Fiware System Handle
^^^^^^^^^^^^^^^^^^^^

This repository contains the source code of the *Integration Service System Handle*
for the `FIWARE <https://www.fiware.org/>` middleware protocol, widely used in the robotics field.

The main purpose of the *FIWARE System Handle* is that of establishing a connection between a *FIWARE's Context Broker*
and an application running over a different middleware implementation.
This is the classic use-case for *Integration Service*.

Dependencies
------------

The dependencies of the *FIWARE System Handle* are:
* `Asio C++ Library <https://think-async.com/Asio/>`_
* `cURLpp library <http://www.curlpp.org/>`_
* `cURL library <https://curl.se/>`_

Configuration
-------------

Regarding the *FIWARE System Handle*, there are several specific parameters which must be configured
for the *FIWARE* middleware. These parameters are mandatory, and are suboptions of the main
five sections:

* :code:`systems`: The system :code:`type` must be :code:`fiware`. In addition to the
  :code:`type` and :code:`types-from` fields, the *FIWARE System Handle* accepts some specific
  configuration fields:

  .. code-block:: yaml

  systems:
    fiware:
      type: fiware
      host: localhost
      port: 1026


    * :code:`port`: The specific port where the *FIWARE's Context Broker* will listen for incoming connections. This field is
      required.
    * :code:`host`: The IP address of the *FIWARE's Context Broker*. This field is required.

Examples
--------

There is one example that you can find in this documentation in which the *ROS 1 System Handle* is employed in the communication:

* :ref:`Publisher/subscriber intercommunication between FIWARE and ROS 2 <fiware-ros2_comm>`

Compilation flags
-----------------

Besides the :ref:`global_compilation_flags` available for the
whole *Integration Service* product suite, there are some specific flags which apply only to the
*FIWARE System Handle*. They are listed below:

* :code:`BUILD_FIWARE_TESTS`: Allows to specifically compile the *FIWARE System Handle* unitary and
  integration tests. It is useful to avoid compiling each *System Handle*'section test suite present
  in the :code:`colcon` workspace, which is what would happen if using the :code:`BUILD_TESTS` flag, with the objective of minimizing building time. To use it, after making sure that the *FIWARE System Handle*
  is present in your :code:`colcon` workspace, execute the following command:
  
  .. code-block:: bash
  
      ~/is_ws$ colcon build --cmake-args -DBUILD_FIWARE_TESTS=ON


.. TODO: complete when it is uploaded to read the docs

API Reference
-------------
