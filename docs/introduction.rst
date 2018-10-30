Getting Started
===============


Brief introduction to Integration Services
------------------------------------------

Integration Services allows to intercommunicate different systems, services and protocols using a common interface.

.. image:: IS-RTPS-Other.png
   :align: center

Integration Services defines *connectors* which are a pair of *subscriber* and *publisher*,
and optionally a *transformation function*.
Each connector, will communicate the data received from its *subscriber* with its *publisher*.
If a *transformation function* was defined, the data will travel from *subscriber* to the *transformation function*
which will apply its transformations, and the result will go to the *publisher*.

.. image:: fullconnector.png
   :align: center

Executing your first Integration Services
-----------------------------------------

Integration Services uses a xml configuration file to create its connectors.
The complete format of this file is explained in :ref:`Integration Services XML Configuration`.

To create our first application we will just create a new one with the followed content:

.. code-block:: xml

    <is>
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

        <connector name="domain_change">
            <subscriber participant_name="domain0" subscriber_name="is_subscriber"/>
            <publisher participant_name="domain5" publisher_name="is_publisher"/>
        </connector>
    </is>

Let's name this new file as **config.xml**. In the *examples* folder of Integration Services, there is an example named
*domain_change*. Building this example there is generated an application called DomainExample.

If you didn't build the example yet execute the following commands in the example folder.

::

    $ mkdir build && cd build
    $ cmake ..
    $ make

If you execute the DomainExample application as publisher a FastRTPS publisher at domain 0 will be created.

::

    $ ./DomainExample publisher

In the same way, executing DomainExample as subscriber will create a FastRTPS subscriber at domain 5.

::

    $ ./DomainExample subscriber

This mechanism is very similar to the FastRTPS HelloWorldExample, but this publisher and subscriber didn't communicate
because they belong to different domains. Keep both DomainExample instances running in different terminals.

We can use Integration Services to allow communication between both DomainExample participants.
We just need to open the folder where we saved our **config.xml** file and execute Integration Services with the file
as argument in another terminal.

::

    $ integration-services config.xml

Once Integration Services parses the configuration file and both participants matches, they will start to communicate.

But this is only a very simple example of what Integration Services can do.
With Integration Services we can communicate different protocols and services just implementing a few methods
from an interface in our own :ref:`bridge libraries`.
Or we can define custom data transformation making use of :ref:`transformation libraries`.
Integration Services allow us to define and use our own **TopicDataTypes**
through :ref:`types libraries` or  Fast-RTPS **XML Types**.
Another interesting functionallity is to replicate data from one subcriber to many publishers, or listen from many
subscriber while writing to one publisher, or just define a N to M relationship between publishers and subscribers.
Finally, Integration Services is able to communicate two applications that belong to different subnetworks,
or through internet and behind Firewalls and NAT using Fast-RTPS **TCP Transport**.
And of course, we can use all of these features at the same time.