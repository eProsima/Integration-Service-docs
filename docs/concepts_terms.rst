Concepts and Terms
==================

Before configuring and using *Integration Service* here you can find the concepts and terms you should know Before
reading the rest of the documentation.

* **Integration Service**: *Integration Service* is often named as *IS*.
* **ISWriter**: An ``ISWriter`` (or *Writer*) is a component able to write data to a destination protocol.
* **ISReader**: An ``ISReader`` (or *Reader*) is a component able to read data from a source protocol.
* **Transformation Function**: A *Transformation Function* is a function that converts between data types.
* **Transformation Library**: A *Transformation Library* is a library that contains *Transformation Functions*.
* **Endpoint**: An *Endpoint* is a *Writer* or *Reader*.
* **Connector**: A *Connector* is a component that relates a *Reader* with a *Writer* and optionally a *Transformation Function*.
* **ISBridge**: An ``ISBridge`` is a component that holds a set of connectors and manages them.
* **Bridge Library**: A *Bridge Library* (or *Bridge*) is a library that provides *Writers* and *Readers* to support additional protocols and optionally more complex *Bridges*.

.. TODO, change the URL to point to *Fast RTPS* Concepts and Terms.

You should know `Fast-RTPS's Concepts and Terms <http://docs.eprosima.com/en/latest/introduction.html>`__ too.

You can interact with *Integration Service* at two different levels:

* As a stand-alone application.
* As a library.

If you use *IS* as a library, you must provide the configuration XML file programmatically.

Integration Service architecture
---------------------------------

*IS* provides three interfaces that must be implemented by any bridge that you want to use. This classes are
:ref:`isbridge`, :ref:`iswriter` and :ref:`isreader`. There is an :ref:`rtps-bridge`
implementation as default, that uses *Fast RTPS* libraries.

*IS* is intended to communicate *Fast RTPS* with others protocols when using *bridges*, so any :ref:`connector`
must have at least one endpoint configured as a *Fast RTPS* participant.

When you implement your ``ISBridge`` derived class, you must take into account:

- Only :class:`ISWriter::write` is mandatory to implement.
- When your *reader* receives data, you must call :class:`on_received_data` function with the data properly converted into :class:`SerializedPayload_t`.
- You can override the default behaviour but isn't recommended in general. This behaviour follows this diagram:

.. image:: flow.png
    :align: center

When the reader calls to its method :class:`on_received_data`, it will call all the *bridges* it belongs,
calling the method :class:`on_received_data` of each bridge.
Then the bridges will apply each respective :ref:`transformation functions <Transformation Library>`
to the data and will call the :class:`write` method of each of their writers.
Note that the flavor of these called methods will be always the same depending on the use of dynamic data or not.
This behaviour will only occur with the declared connectors in the XML configuration file.

Connector
---------

A connector is a pair reader/writer with an optional :ref:`transformation function <Transformation Library>`.
Internally represents a route that the data will follow, applied by its bridge.
If a transformation function was defined, then it will be applied before the data is
sent to the writers.

.. image:: fullconnector.png
   :align: center