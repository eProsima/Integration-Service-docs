.. _websocket_sh:

WebSocket System Handle
=======================

This repository contains the source code of *Integration Service System Handle*
for the `WebSocket <https://www.websocket.org/>`_ middleware protocol, widely used in the robotics field.
The main purpose of the *WebSocket System Handle* is that of establishing a connection between a *WebSocket*
application and an application running over a different middleware implementation.
This is the classic use-case for *Integration Service*.

Dependencies
^^^^^^^^^^^^

The dependencies of the *WebSocket System Handle* are:

* `OpenSSL <https://www.openssl.org/>`_
* `WebSocket++ <https://github.com/zaphoyd/websocketpp>`_

Configuration
^^^^^^^^^^^^^

Regarding the *WebSocket System Handle*, there are several specific parameters which can be configured
for the WebSocket middleware. All of these parameters are suboptions of the main
five sections:

* :code:`systems`: The system :code:`type` must be :code:`websocket_server` or :code:`websocket_client`.
  In addition to the :code:`type` and :code:`types-from` fields,
  the *WebSocket System Handle* accepts a wide variety of specific
  configuration fields, depending on the selected operation mode (*Client* or *Server*).

  For the :code:`websocket_server` *System Handle*, there are two possible configuration scenarios:
  the former one uses a TLS endpoint, and the latter uses a TCP endpoint.

  **TLS**

    .. code-block:: yaml

      systems:
        websocket:
          type: websocket_server
          port: 80
          cert: path/to/cert/file.crt
          key: path/to/key/file.key
          authentication:
            policies: [
              { secret: this-is-a-secret, algo: HS256, rules: {example: "*regex*"} }
            ]


  **TCP**

    .. code-block:: yaml

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
    * :code:`secret`: When using **MAC** *(Message Authentication Code)* method for verification, this field allows to set the secret used to authenticate the client requesting a connection to the server.
    * :code:`pubkey`: Path to a file containing a **PEM** encoded public key.

    **NOTE:** Either a `secret` or a `pubkey` is required.

    * :code:`rules`: List of additional claims that should be checked. It should contain a map with keys
      corresponding to the claim identifier, and values corresponding to regex patterns that should match
      the payload's value. In the example above, the rule will check that the payload contains
      an :code:`example` claim and that its value contains the *regex* keyword in any position of the message. This field is optional.
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
^^^^^^^^^^^^^^^^^^^^^^

In order to communicate with the *WebSocket System Handle* using the JSON encoding,
the messages should follow a specific pattern.
This pattern will be different depending on the paradigm used for the connection
(*pub/sub* or *client/server*) and the communication purpose.

Several fields can be used in those messages, but not all of them are mandatory.
All of them will be described in this section, as well as in which cases they are optional:

* :code:`op`: The *Operation Code* is mandatory in every communication as it specifies the purpose of the message.
  This field can assume nine different values,   which are the ones detailed below.

  * :code:`advertise`: It notifies that there is a new publisher that is going to publish messages on a specific topic. The fields that can be set for this operation are: :code:`topic`, :code:`type` and optionally the :code:`id`.

    .. code-block:: JSON

        {"op": "advertise", "topic": "helloworld", "type": "HelloWorld", "id": "1"}

  * :code:`unadvertise`: It states that a publisher is not going to publish any more messages on a specific topic.
    The fields that can be set for this operation are: :code:`topic` and optionally the :code:`id`.

    .. code-block:: JSON

        {"op": "unadvertise", "topic": "helloworld", "id": "1"}

  * :code:`publish`: It identifies a message that wants to be published over a specific topic.
    The fields that can be set for this operation are: :code:`topic` and :code:`msg`.

    .. code-block:: JSON

        {"op": "publish", "topic": "helloworld", "msg": {"data": "Hello"}}

  * :code:`subscribe`: It notifies that a subscriber wants to receive the messages published under a specific topic.
    The fields that can be set for this operation are: :code:`topic` and optionally the :code:`id` and :code:`type`.

    .. code-block:: JSON

        {"op": "subscribe", "topic": "helloworld", "type": "HelloWorld", "id": "1"}

  * :code:`unsubscribe`: It states that a subscriber doesn't want to receive messages from a specific topic anymore.
    The fields that can be set for this operation are: :code:`topic` and optionally the :code:`id`.

    .. code-block:: JSON

        {"op": "unsubscribe", "topic": "helloworld", "id": "1"}

  * :code:`call_service`: It identifies a message request that wants to be published on a specific service.
    The fields that can be set for this operation are: :code:`service`, :code:`args` and optionally the :code:`id`.

    .. code-block:: JSON

        {"op": "call_service", "service": "hello_serv", "args": {"req": "req"}, "id": "1"}

  * :code:`advertise_service`: It notifies that a new server is going to attend to the requests done on a specific service.
    The fields that can be set for this operation are: :code:`request_type`, :code:`reply_type` and :code:`service`.

    .. code-block:: JSON

        {"op": "advertise_service", "service": "hello_serv", "request_type": "HelloRequest", "reply_type": "HelloReply"}

  * :code:`unadvertise_service`: It states that a server is not going to attend any more the requests done on a specific service.
    The fields that can be set for this operation are: :code:`type` and :code:`service`.

    .. code-block:: JSON

        {"op": "unadvertise_service", "service": "hello_serv", "type": "HelloReply"}

  * :code:`service_response`: It identifies a message reply that wants to be published as response to a specific request.
    The fields that can be set for this operation are: :code:`service`, :code:`values` and optionally the :code:`id`.

    .. code-block:: JSON

        {"op": "service_response", "service": "hello_serv", "values": {"resp": "resp"}, "id": "1"}

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
^^^^^^^^

There is one example that you can find in this documentation in which the *WebSocket System Handle*
is employed in the communication:

* :ref:`ros2_websocket_bridge_pubsub`

Compilation flags
^^^^^^^^^^^^^^^^^

Besides the :ref:`global_compilation_flags` available for the
whole *Integration Service* product suite, there are some specific flags which apply only to the
*WebSocket System Handle*; they are listed below:

* :code:`BUILD_WEBSOCKET_TESTS`: Allows to specifically compile the *WebSocket System Handle* unitary and
  integration tests. It is useful to avoid compiling each *System Handle*'section test suite present
  in the :code:`colcon` workspace, which is what would happen if using the :code:`BUILD_TESTS` flag,
  with the objective of minimizing building time. To use it, after making sure that the
  *WebSocket System Handle* is present in your :code:`colcon` workspace, execute the following command:

  .. code-block:: bash

      ~/is_ws$ colcon build --cmake-args -DBUILD_WEBSOCKET_TESTS=ON


.. TODO: complete when it is uploaded to read the docs

.. API Reference
.. ^^^^^^^^^^^^^
