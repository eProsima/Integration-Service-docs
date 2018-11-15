Integration Service XML Configuration
======================================

Integration Service (IS) uses an XML configuration file to create its connectors.

This XML file can contain the following sections, all inside a root :class:`<is>` label.

.. code-block:: xml

    <is>
        <is_types/>
        <profiles/>
        <bridge/>
        <connector/>
    </is>


IS Types
--------

IS Types section allows you to specify what topic data types will be loaded through :ref:`types libraries` and define
topic data types with `Fast-RTPS XML Types <http://docs.eprosima.com/en/latest/dynamictypes.html#xml-dynamic-types>`__.

If a data type uses Keys or you want to define how to build them, you must use :ref:`types libraries` to
instantiate them. In most cases, you can ignore type details and IS will use :class:`GenericPubSubType` as default,
which encapsulates any kind of type without keys defined.

If you want to make use of `Fast-RTPS Dynamic Types <http://docs.eprosima.com/en/latest/dynamictypes.html>`__ you
can use Fast-RTPS API in a :ref:`types libraries` or use
`Fast-RTPS XML Types <http://docs.eprosima.com/en/latest/dynamictypes.html#xml-dynamic-types>`__ as said before.

.. code-block:: xml

    <is_types>
        <!-- Fast-RTPS XML Types -->
        <types>
            <type>
                <struct name="HelloWorld">
                    <unsignedlong name="index"/>
                    <string name="message"/>
                </struct>
            </type>
        </types>

        <!-- IS Types Libraries -->
        <type name="ShapeType">
            <library>libshape.so</library>
        </type>
        <types_library>libdefault.so</types_library>
    </is_types>


As you can see in the example XML code, you can define :ref:`types libraries` for each type like :class:`ShapeType` and
:class:`libshape.so`, or use a default library that will try to load the rest of types
(:class:`libdefault.so` in the example).

If no library is defined by a type declared by a participant in :ref:`profiles`, and it wasn't declared through
*Fast-RTPS XML Types*, then IS will use :class:`GenericPubSubType` to manage it.

If **IS Types** is omitted, IS will use :class:`GenericPubSubType` to manage all topic data types declared in the
:ref:`profiles` section.

Profiles
--------

The profiles section defines participants, subscribers, publishers, etc, following the format used by **FastRTPS XML configuration files**, with its configuration.

.. code-block:: xml

    <profiles>
        <participant profile_name="domain0">
            <rtps>
                <builtin>
                    <domainId>0</domainId>
                </builtin>
            </rtps>
        </participant>

        <participant profile_name="domain5">
            <rtps>
                <builtin>
                    <domainId>5</domainId>
                </builtin>
            </rtps>
        </participant>

        <subscriber profile_name="is_subscriber">
            <topic>
                <name>TextPubSubTopic</name>
                <dataType>Text</dataType>
            </topic>
            <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        </subscriber>

        <publisher profile_name="is_publisher">
            <topic>
                <name>TextPubSubTopic</name>
                <dataType>Text</dataType>
            </topic>
            <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        </publisher>
    </profiles>

Bridges
-------

Bridge sections allow us to define new endpoints and bridges to implement new protocols.
Inside the bridge, a :ref:`bridge libraries` must be defined. It contains the methods to create the bridge (implementing
:ref:`isbridge`), the writers (implementing :ref:`iswriter`) and the readers (implementing :ref:`isreader`).
If any of them uses the default implementation, its method can simply return :class:`nullptr`.

A properties label with any number of property sections (which are pairs *name* and *value* as shown in the example)
can be defined for the bridge.
Properties that apply to writers and readers are defined directly inside their sections.
Each property set will be sent to its component as a vector of pairs of strings.

If no properties are provided, then your :class:`create_` method will be called with :class:`nullptr` or an empty
vector as parameter config.

.. code-block:: xml

    <bridge name="file">
        <library>build/libisfile.so</library>
        <properties>
            <property>
                <name>propertyA</name>
                <value>valueA</value>
            </property>
            <property>
                <name>propertyB</name>
                <value>valueB</value>
            </property>
        </properties>

        <writer name="file_writer">
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
        </writer>
    </bridge>

Connectors
----------

The *connectors* are just relationships between readers and writers, and optionally, a transformation function.
Any number of connectors can be defined in our XML configuration file,
but at least one is needed to make IS perform any work.
They must contain a reader and a writer.
Each of them is configured by a participant or bridge name and the reader's or writer's name respectively.

In the following example, we define a connector whose subscriber receives data from Fast-RTPS and its writer
writes that data to a text file.
Also, there is defined a function of A :ref:`transformation libraries` that adds the timestamp before the data is written.

.. code-block:: xml

    <connector name="dump_to_file">
        <reader participant_profile="rtps" subscriber_profile="fastrtps_subscriber"/>
        <writer bridge_name="file" writer_name="file_writer"/>
        <transformation file="libfile.so" function="addTimestamp"/>
    </connector>

There are several possible types of connectors depending on the kind of its participants.
Each connector type will refer to the bottom :ref:`example`.

RTPS Bridge
^^^^^^^^^^^

In this kind of connector, both participants are RTPS compliant, like *shapes_projection* and *shapes_stereo* in our example file.

.. image:: RTPS-bridge.png
    :align: center

.. code-block:: xml

    <is>
        <profiles>
            <participant profile_name="RTPS-Publisher">
                <!-- RTPS participant attributes -->
            </participant>

            <participant profile_name="RTPS-Subscriber">
                <!-- RTPS participant attributes -->
            </participant>

            <subscriber profile_name="Subscriber">
                <!-- RTPS subscriber attributes -->
            </subscriber>

            <publisher profile_name="Publisher">
                <!-- RTPS publisher attributes -->
            </publisher>
        </profiles>

        <connector name="connector">
            <reader participant_profile="RTPS-Subscriber" subscriber_profile="Subscriber"/>
            <writer participant_profile="RTPS-Publisher" publisher_profile="Publisher"/>
            <transformation file="/path/to/transform/libuserlib.so" function="transform"/>
        </connector>
    </is>

RTPS to Other protocol
^^^^^^^^^^^^^^^^^^^^^^

This connector will communicate an RTPS environment with another protocol. Just like our *shapes_protocol* connector.

Your *Bridge Library* must define at least a writer to your desired protocol and it is responsible to
communicate with it and follow the ISWriter interface. By default, the transformation function is applied after
:class:`on_received_data` method calls to the instance of ISBridge.
If you want to change this behaviour you will need to override the complete data flow.

.. image:: IS-RTPS-to-Other.png
    :align: center

.. code-block:: xml

    <is>
        <profiles>
            <participant profile_name="RTPS">
                <!-- RTPS participant attributes -->
            </participant>

            <subscriber profile_name="Subscriber">
                <!-- RTPS subscriber attributes -->
            </subscriber>
        </profiles>

        <bridge name="Other protocol">
            <library>/path/to/bridge/library/libprotocol.so</library>
            <!-- Other protocol properties -->

            <writer name="Other">
                <!-- Other protocol writer properties -->
            </writer>
        </bridge>

        <connector name="connector">
            <reader participant_profile="RTPS" subscriber_profile="Subscriber"/>
            <writer bridge_name="Other protocol" writer_name="Other"/>
            <transformation file="/path/to/transform/libuserlib.so" function="transform"/>
        </connector>
    </is>

Other procotol to RTPS
^^^^^^^^^^^^^^^^^^^^^^

This is a similar case as the previous one, but in the other way, as in the connector *protocol_shapes* of our example.

The same logic applies in this connectors as in the :ref:`rtps to other protocol` case,
but in this case, the RTPS participant is the writer. An example of this can be found on
`FIROS2 <https://github.com/eProsima/FIROS2/tree/master/examples/helloworld_ros2>`__.

.. image:: IS-Other-to-RTPS.png
    :align: center

.. code-block:: xml

    <is>
        <profiles>
            <participant profile_name="RTPS">
                <!-- RTPS participant attributes -->
            </participant>

            <publisher profile_name="Publisher">
                <!-- RTPS publisher attributes -->
            </publisher>
        </profiles>

        <bridge name="Other protocol">
            <library>/path/to/bridge/library/libprotocol.so</library>
            <!-- Other protocol properties -->

            <reader name="Other">
                <!-- Other protocol reader properties -->
            </reader>
        </bridge>

        <connector name="connector">
            <reader bridge_name="Other protocol" reader_name="Other"/>
            <writer participant_profile="RTPS" publisher_profile="Publisher"/>
            <transformation file="/path/to/transform/libuserlib.so" function="transformFromA"/>
        </connector>
    </is>

Bidirectional bridge
^^^^^^^^^^^^^^^^^^^^

This case is not a connector, but the consequence of set two connectors with the correct parameters.
In our example, the combination of *shapes_projection* and *shapes_stereo* is a bidirectional bridge,
as well as, *shapes_protocol* and *protocol_shapes*.

A combination of both logics :ref:`rtps to other protocol` and :ref:`Other protocol to RTPS` applies here.
The example `TIS_NGSIv2 <https://github.com/eProsima/FIROS2/tree/master/examples/TIS_NGSIv2>`__ of FIROS2 uses a
bridge of this type.

.. image:: IS-RTPS-Other.png
    :align: center

.. code-block:: xml

    <is>
        <profiles>
            <participant profile_name="RTPS">
                <!-- RTPS participant attributes -->
            </participant>

            <subscriber profile_name="Subscriber">
                <!-- RTPS subscriber attributes -->
            </subscriber>

            <publisher profile_name="Publisher">
                <!-- RTPS publisher attributes -->
            </publisher>
        </profiles>

        <bridge name="Other protocol">
            <library>/path/to/bridge/library/libprotocol.so</library>
            <!-- Other protocol properties -->

            <reader name="OtherSub">
                <!-- Other protocol reader properties -->
            </reader>

            <writer name="OtherPub">
                <!-- Other protocol writer properties -->
            </writer>
        </bridge>

        <connector name="connector">
            <reader bridge_name="Other protocol" reader_name="OtherSub"/>
            <writer participant_profile="RTPS" publisher_profile="Publisher"/>
            <transformation file="/path/to/transform/libuserlib.so" function="transformFromA"/>
        </connector>

        <connector name="connector">
            <reader participant_profile="RTPS" subscriber_profile="Subscriber"/>
            <writer bridge_name="Other protocol" writer_name="OtherPub"/>
            <transformation file="/path/to/transform/libuserlib.so" function="transform"/>
        </connector>
    </is>

Example
-------

In this file, there are defined two RTPS *participants*, and a *bridge*. All of them have a subscriber and a publisher.
The relationships between *participants* and *subscribers*/*publishers* defined in the *profiles* section are
stablished by each *connector*. This allows to share *subscribers*/*publishers* configurations between *participants*.
There are four connectors defined: *shapes_projection*, *shapes_stereo*, *shapes_protocol* and *protocol_shapes*.

.. figure:: Config.png
    :align: center
    :target: example_config.xml
    :alt: Click on the image to open the code.

    Click on the image to open the code.

.. code-block:: xml

    <is>
        <is_types>
            <type name="ShapeType">
                <library>libshapelib.so</library> <!-- Library for ShapeType -->
            </type>
            <type name="Unused type"/>
            <!-- Can be used to pack types or types without their own library -->
            <types_library>libotherlib.so</types_library>
        </is_types>

        <profiles>
            <participant profile_name="2Dshapes">
                <!-- RTPS participant attributes -->
            </participant>

            <participant profile_name="3Dshapes">
                <!-- RTPS participant attributes -->
            </participant>

            <subscriber profile_name="2d_subscriber">
                <!-- RTPS subscriber attributes -->
            </subscriber>

            <subscriber profile_name="3d_subscriber">
                <!-- RTPS subscriber attributes -->
            </subscriber>

            <publisher profile_name="2d_publisher">
                <!-- RTPS publisher attributes -->
            </publisher>

            <publisher profile_name="3d_publisher">
                <!-- RTPS publisher attributes -->
            </publisher>
        </profiles>

        <bridge name="protocol">
            <library>/path/to/bridge/library/libprotocol.so</library>
            <properties>
                <property>
                    <name>property1</name>
                    <value>value1</value>
                </property>
            </properties>

            <writer name="protocol_publisher">
                <property>
                    <name>property1</name>
                    <value>value1</value>
                </property>
                <property>
                    <name>property2</name>
                    <value>value2</value>
                </property>
            </writer>

            <reader name="protocol_subscriber">
                <property>
                    <name>property1</name>
                    <value>value1</value>
                </property>
                <property>
                    <name>property2</name>
                    <value>value2</value>
                </property>
            </reader>
        </bridge>

        <connector name="shapes_projection">
            <reader participant_profile="3Dshapes" subscriber_profile="3d_subscriber"/>
            <writer participant_profile="2Dshapes" publisher_profile="2d_publisher"/>
            <transformation file="/path/to/transform/libuserlib.so" function="transform3D_to_2D"/>
        </connector>

        <connector name="shapes_stereo">
            <reader participant_profile="2Dshapes" subscriber_profile="2d_subscriber"/>
            <writer participant_profile="3Dshapes" publisher_profile="3d_publisher"/>
            <transformation file="/path/to/transform/libuserlib.so" function="transform2D_to_3D"/>
        </connector>

        <connector name="shapes_protocol">
            <reader participant_profile="2Dshapes" subscriber_profile="2d_subscriber"/>
            <writer bridge_name="protocol" writer_name="protocol_publisher"/>
            <transformation file="/path/to/transform/libprotocoltransf.so" function="transformFrom2D"/>
        </connector>

        <connector name="protocol_shapes">
            <reader bridge_name="protocol" reader_name="protocol_subscriber"/>
            <writer participant_profile="2Dshapes" publisher_profile="2d_publisher"/>
            <transformation file="/path/to/transform/libprotocoltransf.so" function="transformTo2D"/>
        </connector>
    </is>
