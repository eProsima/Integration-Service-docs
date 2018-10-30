Integration Services XML Configuration
======================================

Integration Services (IS) uses a XML configuration file to create its connectors.

This XML file can contain the following sections, all inside a root <is> label.

.. code-block:: xml

    <is>
        <types/>
        <topic_types/>
        <profiles/>
        <bridge/>
        <connector/>
    </is>


Types
-----

The types section follows the format of Fast-RTPS XML Types. Dynamic types can be defined in this section to be used
by the participants declared in the :ref:`profiles` section.

For example:

.. code-block:: xml

    <types>
        <type>
            <struct name="HelloWorld">
                <unsignedlong name="index"/>
                <string name="message"/>
            </struct>
        </type>
    </types>

Types section is optional.


Topic Types
-----------

Topic Types section allows you to specify which topic data types will be used by each participant
and :ref:`types libraries`.
If these data types use Keys or you want to define how to instante them, *topic_types* allows you to
relate *data types libraries* with your types.

.. code-block:: xml

    <topic_types>
        <type name="ShapeType">
            <library>libshape.so</library>
            <participants>
                <participant name="2Dshapes"/>
                <participant name="3Dshapes"/>
            </participants>
        </type>
        <types_library>libdefault.so</types_library>
    </topic_types>

Participants subsection, which is optional, is used to link topic data types with each participant in a direct way.
If ommited, the link will be resolve using the publisher/subscriber attributes.

Library subsection, allows to define a types library for the upper type name. This library is detailed in :ref:`types libraries`.

Finally, a default library for other types can be defined through the types_library subsection, similary to the library
subsection.

By default, any used type will be instantiate as :class:`GenericPubSubType`.

Profiles
--------

The profiles define participants, subscribers, publishers, etc, following the format used by **FastRTPS XML configuration files**, with its configuration.

.. code-block:: xml

    <profiles>
        <participant profile_name="domain0">
            <rtps>
                <builtin>
                    <domainId>0</domainId>
                </builtin>
            </rtps>

            <subscriber profile_name="is_subscriber">
                <topic>
                    <name>TextPubSubTopic</name>
                    <dataType>Text</dataType>
                </topic>
                <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
            </subscriber>
        </participant>

        <participant profile_name="domain5">
            <rtps>
                <builtin>
                    <domainId>5</domainId>
                </builtin>
            </rtps>

            <publisher profile_name="is_publisher">
                <topic>
                    <name>TextPubSubTopic</name>
                    <dataType>Text</dataType>
                </topic>
                <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
            </publisher>
        </participant>
    </profiles>

Bridges
-------

Bridge sections allow us to define new endpoints and bridges to implement new protocols.
Inside the bridge, a :ref:`bridge libraries` must be defined. It contains the methods to create the bridge (implementing
:ref:`isbridge`), the publishers (implementing :ref:`ispublisher`) and the subscribers (implementing :ref:`issubscriber`).
If any of them uses the default implementation, its method can simply return :class:`nullptr`.

A properties label with any number of property sections (which are pairs *name* and *value* as shown in the example) can be defined for the bridge.
Properties that apply to participants and subscribers are defined directly inside their sections.
Each property set will be sent to its component as a vector of pairs of strings.

.. code-block:: xml

    <bridge name="file">
        <library>build/libisfile.so</library>

        <publisher name="file_publisher">
            <property>
                <name>filename</name>
                <value>output.txt</value>
            </property>
            <property>
                <name>format</name>
                <value>txt</value>
            </property>
            <property>
                <name>append</name>
                <value>true</value>
            </property>
        </publisher>
    </bridge>


Connectors
----------

The *connectors* are just relationships between subscribers and publishers, and optionally, a transformation function.
Any number of connectors can be defined in our XML configuration file,
but at least one is needed to make IS perform any work.
They must contain a subscriber and a publisher.
Each of them is configured by a participant or bridge name and the subscriber's or publisher's name respectively.

In the follow example, we define a connector whose subscriber receives data from Fast-RTPS, and its publisher
writes that data to a text file.
A :ref:`transformation libraries`'s function that adds the timestamp before the data is wrote is defined too.

.. code-block:: xml

    <connector name="dump_to_file">
        <subscriber participant_name="rtps" subscriber_name="fastrtps_subscriber"/>
        <publisher bridge_name="file" publisher_name="file_publisher"/>
        <transformation file="libfile.so" function="addTimestamp"/>
    </connector>

There are several possible types of connectors depending of the kind of its participants.
Each connector type will refer to :ref:`example`.

RTPS Bridge
^^^^^^^^^^^

In this kind of connector, both participant are RTPS compliant, like *shapes_projection* and *shapes_stereo* in our example file.

.. image:: RTPS-bridge.png
    :align: center

RTPS to Other protocol
^^^^^^^^^^^^^^^^^^^^^^

This connector will communicate a RTPS environment with another protocol. Just like our *shapes_protocol* connector.

Your *Bridge Library* must define at least a publisher to your desired protocol and it is responsible to
communicate with it and follow the ISPublisher interface. By default, the transformation function is applied after
:class:`on_received_data` method calls to the instance of ISBridge.
If you want to change this behaviour you will need to override the complete data flow.

*Bridge_configuration* node can contain configuration information that *Bridge Library* must understand.
ISManager will parse the *property* nodes of each element and will call the respective :class:`create_`
function of the library with a vector of pairs with the data contained.
If no *bridge_configuration* is provided, then your createBridge will be called with :class:`nullptr` or an empty
vector as parameter config.

*Transformation* library could be reused by your bridge library, with the same or another transformation function inside the same transformation library (an example of reusing the transformation library can be found on `FIROS2 <https://github.com/eProsima/FIROS2/tree/master/examples/TIS_NGSIv2>`__.
Of course, you can add built in transformation functions inside your *bridge library*.

.. image:: IS-RTPS-to-Other.png
    :align: center

Other procotol to RTPS
^^^^^^^^^^^^^^^^^^^^^^

This is a similar case as the previous one, but in the other way, as in the connector *protocol_shapes* of our example.

The same logic applies in this connectors as in the :ref:`rtps to other protocol` case,
but in this case the RTPS participant is the publisher. An example of this can be found on
`FIROS2 <https://github.com/eProsima/FIROS2/tree/master/examples/helloworld_ros2>`__.

.. image:: IS-Other-to-RTPS.png
    :align: center

Bidirectional bridge
^^^^^^^^^^^^^^^^^^^^

This case is not a connector, but the consecuence of set two connectors with the correct parameters.
In our example the combination of *shapes_projection* and *shapes_stereo* is a bidirectional bridge,
as well as, *shapes_protocol* and *protocol_shapes*.

A combination of both logics :ref:`rtps to other protocol` and :ref:`Other procotol to RTPS` applies here.
The example `TIS_NGSIv2 <https://github.com/eProsima/FIROS2/tree/master/examples/TIS_NGSIv2>`__ of FIROS2 uses a
bridge of this type.

.. image:: IS-RTPS-Other.png
    :align: center

Example
-------

In this file there are defined two RTPS *participants*, and a *bridge*. All of them have a subscriber and a publisher.
The relationships between *participants* and *subscribers*/*publishers* defined in the *profiles* section are
stablished by each *connector*. This allows to share *subscribers*/*publishers* configurations between *participants*.
There are four connectors defined: *shapes_projection*, *shapes_stereo*, *shapes_protocol* and *protocol_shapes*.

.. figure:: Config.png
    :align: center
    :target: example_config.xml
    :alt: Click on the image to open the code.

    Click on the image to open the code.
