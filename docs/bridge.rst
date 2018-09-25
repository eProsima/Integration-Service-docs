Bridge Libraries
================

Integration Services allow us to define bridge libraries to integrate new protocols. These libraries must offer the following function declarations:

* create_bridge:

.. code-block:: cpp

    extern "C" USER_LIB_EXPORT ISBridge* create_bridge(const char* name,
        const std::vector<std::pair<std::string, std::string>> *config);

As you can see, the instantiated bridge must implement :ref:`isbridge`.
Bridges are in charge of communicate subscribers with pubsliher and apply transformation functions as defined in
the :ref:`connector`.

* create_subscriber:

.. code-block:: cpp

    extern "C" USER_LIB_EXPORT ISSubscriber* create_subscriber(ISBridge *bridge, const char* name,
        const std::vector<std::pair<std::string, std::string>> *config);

The subscriber returned must implement :ref:`issubscriber`.
Subscribers must be able to receive data from the origin protocol.


* create_publisher:

.. code-block:: cpp

    extern "C" USER_LIB_EXPORT ISPublisher* create_publisher(ISBridge *bridge, const char* name,
        const std::vector<std::pair<std::string, std::string>> *config);

The publisher returned must implement :ref:`ispublisher`.
Publishers must be able to send data to the destination protocol.



In all functions, a vector of pairs of strings is provided is any property exists for each node in the xml configuration file (see :ref:`Integration Services XML Configuration` for more information).
