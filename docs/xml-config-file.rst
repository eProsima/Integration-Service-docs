Integration Services XML Configuration
======================================

Integration Services (IS) uses a XML configuration file to create its connectors. This XML file can contains the
the following sections, all inside a root <is> label.

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

Topic Types section allows you to specify which topic data types will be used by each participant and :ref:`types libraries`.

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

Profiles
--------

The profiles section follows the format of Fast-RTPS XML Profiles. In this section participants, publisher and
subscribers, with its configurations are defined.

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
If any of them uses the default implementation, its method can simply return nullptr.

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

Any number of connectors can be defined in our XML configuration file, but at least one is needed to make IS perform any work.
They must contain a subscriber and a publisher. Each of them is configured by a participant or bridge name and the subscriber's or publisher's name respectively.

In the follow example, we define a connector whose subscriber receives data from Fast-RTPS, and its publisher
writes that data to a text file. A :ref:`transformation libraries`'s function that adds the timestamp before the data is wrote is defined
too.

.. code-block:: xml

    <connector name="dump_to_file">
        <subscriber participant_name="rtps" subscriber_name="fastrtps_subscriber"/>
        <publisher bridge_name="file" publisher_name="file_publisher"/>
        <transformation file="libfile.so" function="addTimestamp"/>
    </connector>
