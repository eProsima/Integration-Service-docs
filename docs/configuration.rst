Configuration
=============

Load configuration
------------------

*Integration Service* must receive an argument with the XML file with the configuration that is going to be loaded.

.. code-block:: bash

    $ integration_service config.xml

With this file, *Integration Service* creates all the components needed to do its job.

Configuration format
--------------------

This XML file can contain the following sections, all inside a root :class:`<is>` label.

.. literalinclude:: configuration.xml
    :language: xml
    :start-after: <!-- Configuration Format Start -->
    :end-before: <!-- Configuration Format End -->
    :dedent: 4

IS Types configuration
----------------------

The *IS Types* section allows to specify what topic data types will be loaded through :ref:`types library` and define
topic data types with `Fast RTPS XML Types <http://docs.eprosima.com/en/latest/dynamictypes.html#xml-dynamic-types>`__.

To create data types that use Keys or to define how to build them, it's necessary to use :ref:`types library` to
instantiate them. In most cases, the type details can be ignored and *IS* will use :class:`GenericPubSubType`
as default, which encapsulates any kind of type without keys defined.

This section uses `Fast RTPS Dynamic Types <http://docs.eprosima.com/en/latest/dynamictypes.html>`__ internally,
but it's available to use them by code using the *Fast RTPS* API in a :ref:`types library` or using
`Fast-RTPS XML Types <http://docs.eprosima.com/en/latest/dynamictypes.html#xml-dynamic-types>`__.

.. literalinclude:: configuration.xml
    :language: xml
    :start-after: <!-- IS Types Start -->
    :end-before: <!-- IS Types End -->
    :dedent: 4

This XML example shows how to define :ref:`types library` for each type like :class:`ShapeType` and
:class:`libshape.so`, or use a default library that will try to load the rest of types
(:class:`libdefault.so` in the example).

If no library is defined by a type declared by a *participant* in :ref:`profiles`, and it wasn't declared through
*Fast-RTPS XML Types*, then *IS* will use :class:`GenericPubSubType` to manage it.

If ``<is_types>`` is omitted, *IS* will use :class:`GenericPubSubType` to manage all topic data types declared in the
:ref:`Fast-RTPS profiles` section.

Fast-RTPS profiles
------------------

The profiles section defines *participants*, *subscribers*, *publishers*, etc, following the format used by
`Fast RTPS XML Types <http://docs.eprosima.com/en/latest/dynamictypes.html#xml-dynamic-types>`__,
with its configuration.

.. literalinclude:: configuration.xml
    :language: xml
    :start-after: <!-- Profiles Start -->
    :end-before: <!-- Profiles End -->
    :dedent: 4

Connectors
----------

The *connectors* are just relationships between *readers* and *writers*, and optionally, a *transformation function*.
Any number of *connectors* can be defined in our XML configuration file,
but at least one is needed to make *IS* perform any work.
They must contain a *reader* and a *writer*. 
Each of them is configured by a *participant* or *bridge* name and the *reader's* or *writer's* name respectively.

In the following example, we define a *connector* whose *subscriber* receives data from Fast-RTPS and its *writer*
writes that data to a text file.
Also, there is defined a function of a :ref:`transformation library` that adds the timestamp before the data is written.

.. literalinclude:: configuration.xml
    :language: xml
    :start-after: <!-- Basic Connector Start -->
    :end-before: <!-- Basic Connector End -->
    :dedent: 4

There are several possible types of *connectors* depending on the kind of its *participants*.
Each *connector* type will refer to the bottom :ref:`example`.

RTPS Connector
^^^^^^^^^^^^^^

In this kind of *connector*, both *participants* are *RTPS* compliant,
like *shapes_projection* and *shapes_stereo* in our :ref:`example` file.

.. image:: RTPS-bridge.png
    :align: center

.. literalinclude:: configuration.xml
    :language: xml
    :start-after: <!-- RTPS Connector Start -->
    :end-before: <!-- RTPS Connector End -->
    :dedent: 4


Connector from RTPS to Other protocol
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This *connector* will communicate an *RTPS* environment with another protocol.
Just like our *shapes_protocol* connector in the :ref:`example`.

The *Bridge Library* must define at least a *writer* to the desired protocol and it is responsible to
communicate with it and follow the ``ISWriter`` interface. By default, the *transformation function* is applied after
:class:`on_received_data` method calls to the instance of ``ISBridge``.
To change this behaviour it's mandatory to override the complete data flow.

.. image:: IS-RTPS-to-Other.png
    :align: center

.. literalinclude:: configuration.xml
    :language: xml
    :start-after: <!-- RTPS To other connector Start -->
    :end-before: <!-- RTPS To other connector End -->
    :dedent: 4

Connector from Other protocol to RTPS
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This is a similar case as the previous one, but in the other way,
as in the *connector* *protocol_shapes* of our example.

The same logic applies in this *connectors* as in the :ref:`Connector from RTPS to Other protocol` case,
but in this case, the RTPS participant is the *writer*. An example of this can be found on
`FIROS2 <https://github.com/eProsima/FIROS2/tree/master/examples/helloworld_ros2>`__.

.. image:: IS-Other-to-RTPS.png
    :align: center

.. literalinclude:: configuration.xml
    :language: xml
    :start-after: <!-- Other connector to RTPS Start -->
    :end-before: <!-- Other connector to RTPS End -->
    :dedent: 4

Bidirectional connector
^^^^^^^^^^^^^^^^^^^^^^^

This case is not a *connector*, but the consequence of setting two *connectors* with the correct parameters.
In our :ref:`example`, the combination of *shapes_projection* and *shapes_stereo* is a bidirectional *connector*,
as well as, *shapes_protocol* and *protocol_shapes*.

A combination of both logics :ref:`Connector from RTPS to Other protocol` and :ref:`Connector from Other protocol to RTPS` applies here.
The example `TIS_NGSIv2 <https://github.com/eProsima/FIROS2/tree/master/examples/TIS_NGSIv2>`__ of *FIROS2* uses a
bridge of this type.

.. image:: IS-RTPS-Other.png
    :align: center

.. literalinclude:: configuration.xml
    :language: xml
    :start-after: <!-- Bidirectional connector Start -->
    :end-before: <!-- Bidirectional connector End -->
    :dedent: 4

Bridge configuration
--------------------

Bridge sections allow to define new *endpoints* to implement new protocols.
Inside the tag ``<bridge>``, a :ref:`bridge library` must be defined.
It contains the methods to create the *bridge* (implementing :ref:`isbridge`),
*writers* (implementing :ref:`iswriter`) and *readers* (implementing :ref:`isreader`).
If any of them uses the default implementation, its method can simply return :class:`nullptr`.

Inside the ``<bridge>``, ``<writer>`` and ``<reader>`` can be defined ``<properties>``.
They are pairs ``<name>`` and ``<value>`` to configure the elements
that can be accessible to the elements in code, without affecting the XML parsing.
The next example shows how to create some *properties* to create attributes for
the *bridge* and some to configure a *writer*.
Each property set will be sent to its component as a vector of pairs of strings, and if no *properties* are provided,
then the :class:`create_` method will be called with :class:`nullptr` or an empty vector as parameter config.

.. literalinclude:: configuration.xml
    :language: xml
    :start-after: <!-- Bridge Start -->
    :end-before: <!-- Bridge End -->
    :dedent: 4

The complete explanation about *bridges* and their API are available :ref:`here <Bridge Library>`.

Writer configuration
^^^^^^^^^^^^^^^^^^^^

The ``<writer>`` section exposes *writer* classes of the *bridge library* to be used by *Integration Service*.
The only mandatory field is the ``<name>`` of the class, and with it, *Integration Service*
is able to create instances of the *writer* when :ref:`connectors` need them.

The complete explanation about *writers* and their API are available :ref:`here <ISWriter>`.


Reader configuration
^^^^^^^^^^^^^^^^^^^^

The ``<reader>`` section exposes *reader* classes of the *bridge library* to be used by *Integration Service*.
The only mandatory field is the ``<name>`` of the class, and with it, *Integration Service*
is able to create instances of the *reader* when :ref:`connectors` need them.

The complete explanation about *readers* and their API are available :ref:`here <ISReader>`.

IS Libraries
------------

There are three different kind of libraries that *Integration Service* manages:

- **Type Library**: Exposes types and the methods to create instances of them. The information about their configuration :ref:`here <IS Types configuration>` and a deeper explanation of them :ref:`here <Types library>`.

- **Transformation Library**: *Transformation libraries* stores functions to manage the input and output data communicating a *reader* and a *writer* inside of a *connector*. These libraries are configured inside the :ref:`Connectors` section. A deeper explanation of this kind of library can be found :ref:`here <Transformation library>`.

- **Bridge Library**: Includes the code to manage endpoints to implement new protocols. Their configuration is explained inside :ref:`Bridge configuration` and the complete description of *bridges* and their API is :ref:`here <ISBridge>`.

Example
-------

In this file, there are defined two RTPS *participants*, and a *bridge*.
All of them have a *subscriber* and a *publisher*.
The relationships between *participants* and *subscribers*/*publishers* defined in the *profiles* section are
stablished by each *connector*. This allows to share *subscribers*/*publishers* configurations between *participants*.
There are four *connectors* defined: *shapes_projection*, *shapes_stereo*, *shapes_protocol* and *protocol_shapes*.

.. figure:: Config.png
    :align: center
    :target: example_config.xml
    :alt: Click on the image to open the code.

    Click on the image to open the code.

.. literalinclude:: configuration.xml
    :language: xml
    :start-after: <!-- IS Libraries Start -->
    :end-before: <!-- IS Libraries End -->
    :dedent: 4

