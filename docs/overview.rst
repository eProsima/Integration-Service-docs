Integration Services Overview
=============================

You can interact with Integration Services (IS) at two different levels:

* As stand-alone application.
* As library to link against.

When using IS as a library, you must provide the configuration xml file programatically.

Integration Services architecture
---------------------------------

IS provides three interfaces that must be implemented by any bridge that you want to use. This classes are
:ref:`isbridge`, :ref:`ispublisher` and :ref:`issubscriber`. There is a :ref:`rtps bridge`
implementation as default, that uses Fast-RTPS libraries.

Any :ref:`connector` must have at least one endpoint configured as a Fast-RTPS participant, as IS is intended to communicate
Fast-RTPS with others protocols when using bridges.


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
*addFunction* and *on_received_data* methods have two flavours, with static and with dynamic data.

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
Any custom bridge that needs to define its publisher, must implement at least both *publish* methods. If one of them
isn't needed, just implement as follows:

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
Any custom bridge that needs to define its subscriber, must implement at least both *on_received_data* methods.
If one of them isn't needed, just implement as follows:

.. code-block:: cpp

    void on_received_data([...]) override { }


RTPS Bridge
-----------

Implements a full bridge using Fast-RTPS publisher and subscriber. Its bridge implementation is able to communicate
several subscribers with several publishers, stablishing routes, and applying transformation functions in function
of each connector configuration.


Connector
---------

A connector is a pair subscriber/publisher with an optional transformation function. Internally represents a route
that the data will follow. If a transformation function was defined, then it will be applyed before the data is
sent to the publishers.