Integration Service Overview
=============================

You can interact with Integration Service (IS) at two different levels:

* As a stand-alone application.
* As a library.

If you use IS as a library, you must provide the configuration XML file programmatically.

Integration Service architecture
---------------------------------

IS provides three interfaces that must be implemented by any bridge that you want to use. This classes are
:ref:`isbridge`, :ref:`iswriter` and :ref:`isreader`. There is an :ref:`rtps-bridge`
implementation as default, that uses Fast-RTPS libraries.

IS is intended to communicate Fast-RTPS with others protocols when using bridges, so any :ref:`connector`
must have at least one endpoint configured as a Fast-RTPS participant.

When you implement your ISBridge derived class, you must take into account:

- Only :class:`ISWriter::write` is mandatory to implement.
- When your reader receives data, you must call :class:`on_received_data` function with the data properly converted into :class:`SerializedPayload_t`.
- You can override the default behaviour but isn't recommended in general. This behaviour follows this diagram:

.. image:: flow.png
    :align: center

When the reader calls to its method :class:`on_received_data`, it will call all the *bridges* it belongs,
calling the method :class:`on_received_data` of each bridge.
Then the bridges will apply each respective :ref:`transformation functions <Transformation Libraries>`
to the data and will call the :class:`write` method of each of their writers.
Note that the flavor of these called methods will be always the same depending on the use of dynamic data or not.
This behaviour will only occur with the declared connectors in the XML configuration file.

Connector
---------

A connector is a pair reader/writer with an optional :ref:`transformation function <Transformation Libraries>`.
Internally represents a route that the data will follow, applied by its bridge.
If a transformation function was defined, then it will be applied before the data is
sent to the writers.

.. image:: fullconnector.png
   :align: center

ISBridge
^^^^^^^^
This component must communicate readers with writers, applying the
:ref:`transformation functions <Transformation Libraries>` if any.
Its default implementation must be enough for the majority of cases.
It can be seen as a **connector manager** as it is responsible to apply the data flow and the logic of each connector.
A bridge can manage several **connectors** and it should reuse readers, transformation functions and writers if it's
possible. In complex configurations, like in this :ref:`example`, several connectors can share the same readers,
transformation functions, and writers.

Custom bridges must inherit from it:

.. code-block:: cpp

    class ISBridge
    {
    public:
        virtual void onTerminate();
        virtual void addReader(ISReader *sub);
        virtual void addFunction(const std::string &sub, const std::string &fname, userf_t func);
        virtual void addFunction(const std::string &sub, const std::string &fname, userdynf_t func);
        virtual void addWriter(const std::string &sub, const std::string &funcName, ISWriter* pub);
        virtual ISWriter* removeWriter(ISWriter* pub);
        virtual void on_received_data(const ISReader *sub, SerializedPayload_t *data);
        virtual void on_received_data(const ISReader *sub, DynamicData *data);
    };

ISBridge.h and ISBridge.cpp implement the default behaviour. There is no need to implement any function from any
subclass, but all of the above could be implemented if needed. Be careful to implement the full functionality.
It is recommended to copy the standard implementation and modify with your needs.
After that, simply remove unmodified methods.
:class:`addFunction` and :class:`on_received_data` methods have two flavors, with static and dynamic data.

RTPS-Bridge
^^^^^^^^^^^

Implements a full bridge using Fast-RTPS publisher and subscriber. Its bridge implementation is able to communicate
several subscribers with several publishers, establishing routes and applying
:ref:`transformation functions <Transformation Libraries>` depending on each connector configuration.

The connector :ref:`rtps bridge` uses this kind of bridge.


ISWriter
^^^^^^^^^^^
This component must be able to write data to the destination protocol. The default implementation uses a Fast-RTPS
publisher.

.. code-block:: cpp

    class ISWriter
    {
    public:
        virtual bool write(eprosima::fastrtps::rtps::SerializedPayload_t* /*data*/) = 0;
        virtual bool write(eprosima::fastrtps::types::DynamicData* /*data*/) = 0;
        virtual ISBridge* setBridge(ISBridge *);
    };

ISWriter doesn't have a default implementation, so this default behaviour is provided by the builtin RTPS Bridge.
Any custom bridge that needs to define its writer, must implement at least both :class:`write` methods.
If one of them isn't needed, just implement as follows:

.. code-block:: cpp

    bool write([...]) override { return false; }

This is useful if you're sure that version of the method will be never called.

ISReader
^^^^^^^^^^^^
This component is in charge of receive data from the input protocol. Its default implementation uses a Fast-RTPS
subscriber.

.. code-block:: cpp

    class ISReader
    {
    public:
        virtual void addBridge(ISBridge* bridge);
        virtual void on_received_data(eprosima::fastrtps::rtps::SerializedPayload_t* payload);
        virtual void on_received_data(eprosima::fastrtps::types::DynamicData* data);
    };

ISReader doesn't have a default implementation, so this default behaviour is provided by the builtin RTPS Bridge.
Any custom bridge that needs to define its reader, must implement at least both :class:`on_received_data` methods.
If one of them isn't needed, just implement as follows:

.. code-block:: cpp

    void on_received_data([...]) override { }
