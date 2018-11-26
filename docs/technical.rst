User Libraries
==============

*Integration Service* defines three types of libraries, *Bridge Library*, *Transformation Library*
and *Types Library*. All of them can be implemented by the user.

All these libraries can be mixed in the same library, including adding several libraries of the same kind
(an example of several transformation libraries
can be found on `FIROS2 <https://github.com/eProsima/FIROS2/tree/master/examples/TIS_NGSIv2>`__).

See :ref:`Configuration format` for more information about how to indicate *Integration Service* which libraries use.

* :ref:`Transformation Library`: Allows you to create custom data transformations.

* :ref:`Bridge Library`: Allows you to implement new *Writers* and *Readers* to allow other protocols.

* :ref:`Types Library`: Allows you to define *TopicDataTypes*.


Transformation Library
----------------------

*Integration Service* allows you to define *transformation functions* that will be applied to each :ref:`connector`.
*Transformation functions* are static functions that receive the input data,
apply some transformation and stores the result in the output data.
The *connector* will be configured with the function to call in each case.
There is a static data prototype in
`resource/templatelib.cpp <https://github.com/eProsima/Integration-Service/blob/master/resource/templatelib.cpp>`__:

These functions must have one of the following interfaces:

Static Data
^^^^^^^^^^^

.. literalinclude:: technical.cpp
    :language: cpp
    :start-after: // Static Data Start
    :end-before: // Static Data End


Dynamic Data
^^^^^^^^^^^^

.. literalinclude:: technical.cpp
    :language: cpp
    :start-after: // Dynamic Data Start
    :end-before: // Dynamic Data End

In both cases, the transformation function will parse the :class:`inputData`,
modify it at will and will store the result into :class:`outputData`.
See the :ref:`examples` for some already working implementations.

Bridge Library
--------------

*Integration Service* allows us to define *bridge libraries* to integrate new protocols.
These libraries must offer the following function declarations:

* **create_bridge**:

.. literalinclude:: technical.cpp
    :language: cpp
    :start-after: // Create Bridge Start
    :end-before: // Create Bridge End

As you can see, the instantiated *bridge* must implement :ref:`isbridge`.
``ISBridges`` are in charge of communicating *readers* with *writers* and apply *transformation functions* as defined in
the :ref:`connector`.

* **create_reader**:

.. literalinclude:: technical.cpp
    :language: cpp
    :start-after: // Create Reader Start
    :end-before: // Create Reader End

The *reader* returned must implement :ref:`isreader`.
``ISReaders`` must be able to receive data from the input protocol.


* **create_writer**:

.. literalinclude:: technical.cpp
    :language: cpp
    :start-after: // Create Writer Start
    :end-before: // Create Writer End

The *writer* returned must implement :ref:`iswriter`.
``ISWriters`` must be able to send data to the destination protocol.

In all functions, a vector of pairs of strings is provided if any property exists for each node in the XML
configuration file (see :ref:`Bridge configuration` for more information).

If some functions want to use the default implementation (*RTPS-Bridge*), they must return :class:`nullptr`.

Integration Service will deallocate these objects from memory when the bridge is stopped.

See :ref:`Integration Service architecture` section for more information about the interfaces that any *Bridge Library*
must implement.

The responsibility of how to instantiate your *bridge*, *writer* and/or *reader* is on your *Bridge Library*,
but remember that "RTPS" *publishers* and *subscribers* will be filled automatically by ``ISManager``
with the configuration from the ``<participant>`` node of the :ref:`Fast-RTPS profiles`.

See the :ref:`Adding new Bridges` section for some already working implementations.

ISBridge
^^^^^^^^

This component must communicate ``ISReaders`` with ``ISWriters``, applying the
:ref:`transformation functions <Transformation Library>` if any.
Its default implementation must be enough for the majority of cases.
It can be seen as a **connector manager** as it is responsible to apply the data flow and the logic of each connector.
A bridge can manage several *connectors* and it should reuse readers, transformation functions and writers if it's
possible. In complex configurations, like in this :ref:`example`, several connectors can share the same readers,
transformation functions, and writers.

Custom ``ISBridges`` must inherit from it:

.. literalinclude:: technical.cpp
    :language: cpp
    :start-after: // Class Bridge Start
    :end-before: // Class Bridge End

:class:`ISBridge.h` and :class:`ISBridge.cpp` implement the default behaviour.
There is no need to implement any function from any
subclass, but all of the above could be implemented if needed. Be careful to implement the full functionality.
It is recommended to copy the standard implementation and modify with your needs.
After that, simply remove the unmodified methods.
:class:`addFunction` and :class:`on_received_data` methods have two flavors, with static and dynamic data.

RTPS-Bridge
^^^^^^^^^^^

*Integration Service* has a default builtin *RTPS-Bridge*, but you can specify any other protocol
implementing your own libraries.

Implements a full ``ISBridge`` using *Fast-RTPS* *publisher* and *subscriber*.
Its ``ISBridge`` implementation is able to communicate
several *subscribers* with several *publishers*, establishing routes and applying
:ref:`transformation functions <Transformation Library>` depending on each *connector* configuration.

The *connector* :ref:`rtps bridge` uses this kind of bridge.


ISWriter
^^^^^^^^

This component must be able to write data to the destination protocol. The default implementation uses a *Fast-RTPS
publisher*.

.. literalinclude:: technical.cpp
    :language: cpp
    :start-after: // Class Writer Start
    :end-before: // Class Writer End

``ISWriter`` doesn't have a default implementation, so this default behaviour is provided by the builtin *RTPS-Bridge*.
Any custom *bridge* that needs to define its *writer*, must implement at least both :class:`write` methods.
If one of them isn't needed, just implement as follows:

.. literalinclude:: technical.cpp
    :language: cpp
    :start-after: // Writer Write Start
    :end-before: // Writer Write End

This is useful if you're sure that version of the method will be never called.

ISReader
^^^^^^^^

This component is in charge of receive data from the input protocol. Its default implementation uses a *Fast-RTPS
subscriber*.

.. literalinclude:: technical.cpp
    :language: cpp
    :start-after: // Class Reader Start
    :end-before: // Class Reader End

``ISReader`` doesn't have a default implementation, so this default behaviour is provided by the builtin *RTPS-Bridge*.
Any custom *bridge* that needs to define its *reader*, must implement at least both :class:`on_received_data` methods.
If one of them isn't needed, just implement as follows:

.. literalinclude:: technical.cpp
    :language: cpp
    :start-after: // Reader Read Start
    :end-before: // Reader Read End


Types Library
-------------

*Integration Service* allows us to define types libraries to create custom data types.
These libraries must offer a function with the following declaration:

.. literalinclude:: technical.cpp
    :language: cpp
    :start-after: // Topic Type 1 Start
    :end-before: // Topic Type 1 End

It will be called with the TopicType ``name`` and must return an instance of it
(subclass of *Fast RTPS's* :class:`TopicDataType`).
If the provided type is unknown, the function must return :class:`nullptr`.

.. literalinclude:: technical.cpp
    :language: cpp
    :start-after: // Topic Type 2 Start
    :end-before: // Topic Type 2 End

The returned type can be built using `Fast-RTPS dynamic types <http://docs.eprosima.com/en/latest/dynamictypes.html>`__,
using an already generated IDL statically or implementing it directly as :class:`TopicDataType` subclass.

.. literalinclude:: technical.cpp
    :language: cpp
    :start-after: // Topic Type 3 Start
    :end-before: // Topic Type 3 End

In section :ref:`Dynamic Data Integration` you can find an already working example.