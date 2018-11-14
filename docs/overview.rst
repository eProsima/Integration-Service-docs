Integration Service Overview
=============================

You can interact with Integration Service (IS) at two different levels:

* As stand-alone application.
* As library to link against.

When using IS as a library, you must provide the configuration xml file programatically.

Integration Service architecture
---------------------------------

IS provides three interfaces that must be implemented by any bridge that you want to use. This classes are
:ref:`isbridge`, :ref:`ispublisher` and :ref:`issubscriber`. There is a :ref:`rtps-bridge`
implementation as default, that uses Fast-RTPS libraries.

Any :ref:`connector` must have at least one endpoint configured as a Fast-RTPS participant,
as IS is intended to communicate Fast-RTPS with others protocols when using bridges.

When you implement your ISBridge derived class, you must take in account:

- Only :class:`ISPublisher::publish` is mandatory to implement.
- When your subscriber receives data, you must call :class:`on_received_data` function with the data properly converted into :class:`SerializedPayload_t`.
- You can override the default behaviour, but isn't recommended in general. This behaviour follows this diagram:

.. image:: flow.png
    :align: center

When the subscriber calls to its method :class:`on_received_data`, it will call all the *bridges* it belongs,
calling the method :class:`on_received_data` of each bridge.
Then the bridges will apply each respective transformation functions to the data and will call the :class:`publish`
method of each of their publishers.
Note that the flavour of these called methods will be always the same depending of the use of dynamic data or not.
All this behaviour will only occurs with the declared connectors in the XML configuration file.

ISBridge
^^^^^^^^
This component must communicate subscribers with publishers, applying the transformation functions if any.
It's default implementation must be enough for the mayority of cases.

Custom bridges must inherit from it:

.. code-block:: cpp

    class ISBridge
    {
    public:
        virtual void onTerminate();
        virtual void addSubscriber(ISSubscriber *sub);
        virtual void addFunction(const std::string &sub, const std::string &fname, userf_t func);
        virtual void addFunction(const std::string &sub, const std::string &fname, userdynf_t func);
        virtual void addPublisher(const std::string &sub, const std::string &funcName, ISPublisher* pub);
        virtual ISPublisher* removePublisher(ISPublisher* pub);
        virtual void on_received_data(const ISSubscriber *sub, SerializedPayload_t *data);
        virtual void on_received_data(const ISSubscriber *sub, DynamicData *data);
    };

ISBridge.h and ISBridge.cpp implements the default behaviour. There is no need to implement any function from any
subclass, but all of the above could be implemented if needed. Be careful to implement the full functionallity.
It is recommended to copy the standard implementation and modify with your needs.
After that, simply remove unmodified methods.
:class:`addFunction` and :class:`on_received_data` methods have two flavours, with static and with dynamic data.

ISPublisher
^^^^^^^^^^^
This component must be able to publish data to the destination protocol. The default implementation uses a Fast-RTPS
publisher.

.. code-block:: cpp

    class ISPublisher
    {
    public:
        virtual bool publish(eprosima::fastrtps::rtps::SerializedPayload_t* /*data*/) = 0;
        virtual bool publish(eprosima::fastrtps::types::DynamicData* /*data*/) = 0;
        virtual ISBridge* setBridge(ISBridge *);
    };

ISPublisher doesn't have a default implementation, so this default behaviour is provided by the builtin RTPS Bridge.
Any custom bridge that needs to define its publisher, must implement at least both :class:`publish` methods.
If one of them isn't needed, just implement as follows:

.. code-block:: cpp

    bool publish([...]) override { return false; }

This is useful if you're sure that version of the method will be never called.

ISSubscriber
^^^^^^^^^^^^
This component is in charge of receive data from the origin protocol. Its default implementation uses a Fast-RTPS
subscriber.

.. code-block:: cpp

    class ISSubscriber
    {
    public:
        virtual void addBridge(ISBridge* bridge);
        virtual void on_received_data(eprosima::fastrtps::rtps::SerializedPayload_t* payload);
        virtual void on_received_data(eprosima::fastrtps::types::DynamicData* data);
    };

ISSubscriber doesn't have a default implementation, so this default behaviour is provided by the builtin RTPS Bridge.
Any custom bridge that needs to define its subscriber, must implement at least both :class:`on_received_data` methods.
If one of them isn't needed, just implement as follows:

.. code-block:: cpp

    void on_received_data([...]) override { }


RTPS-Bridge
-----------

Implements a full bridge using Fast-RTPS publisher and subscriber. Its bridge implementation is able to communicate
several subscribers with several publishers, stablishing routes, and applying transformation functions in function
of each connector configuration.

The connector :ref:`rtps bridge` uses this kind of bridge.


Connector
---------

A connector is a pair subscriber/publisher with an optional transformation function. Internally represents a route
that the data will follow. If a transformation function was defined, then it will be applied before the data is
sent to the publishers.

.. image:: fullconnector.png
   :align: center
