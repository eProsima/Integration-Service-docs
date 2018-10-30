Libraries
=========

*Integration Services* has a builtin RTPS bridge, but you can specify any other procotol
implementing your own libraries.

There are three kind of libraries that the user can implement, **Bridge Library**, **Transformation Library**
and **Data Types Library**.

May be necessary generate data types from IDL files to communicate with *Fast-RTPS*,
or make use of Fast-RTPS dynamic types.

The :ref:`Integration Services XML Configuration` file must be adapted to each protocol.
**ISManager** will provide the parsed *properties* node inside *bridge* node to the :class:`create_bridge`
function as a vector of pairs, as defined in the :ref:`Bridge Libraries`.
The same applies for each publisher and subscriber inside *bridge* node and its *property* nodes.


Transformation Libraries
^^^^^^^^^^^^^^^^^^^^^^^^

Integration Services allows us to define transformation functions that will be applied on each :ref:`connector`.
Transformation functions are static functions that receives the input data,
apply some transformation and stores the result in the output data.
The connector will be configured with the function to call in each case.
There is a static data prototype in
`resource/templatelib.cpp <https://github.com/eProsima/Integration-Services/blob/master/resource/templatelib.cpp>`__:


These functions must have one of following interfaces:

Static Data
~~~~~~~~~~~

.. code-block:: cpp

	extern "C" USER_LIB_EXPORT void HelloWorldToKey(SerializedPayload_t* inputData, SerializedPayload_t* outputData)
	{
		// Input HelloWorld Data
		HelloWorldPubSubType hwPubSub;
		HelloWorld hwData;
		hwPubSub.deserialize(inputData, &hwData);

		// Input key Data
		samplePubSubType keyPubSub;
		sample keyData;

		// Custom transformation
		keyData.index() = hwData.index() % 256;
		keyData.key_value() = hwData.index() % 256;

		// Serialize keys
		outputData->reserve(static_cast<uint32_t>(keyPubSub.getSerializedSizeProvider(&keyData)()));
		keyPubSub.serialize(&keyData, outputData);
	}


Dynamic Data
~~~~~~~~~~~~

.. code-block:: cpp

    extern "C" USER_LIB_EXPORT void HelloWorldToKey(DynamicData* inputData, DynamicData* outputData)
    {
        // Custom transformation
        uint32_t temp = inputData->GetUint32Value(0);
        outputData->SetByteValue(temp % 256, 0);
        outputData->SetByteValue(temp % 256, 1);
    }

In both cases, the transformation function will parse the :class:`inputData`,
modify it at will and will store the result into :class:`outputData`.
See the :ref:`examples` for some already working implementations.

Types Libraries
^^^^^^^^^^^^^^^

Integration Services allows us to define types libraries to create custom data types.
These libraries must offer a function with the following declaration:

.. code-block:: cpp

    extern "C" USER_LIB_EXPORT TopicDataType* GetTopicType(const char *name);

It will be called with the TopicType name, and must return an instance of it (subclass of :class:`TopicDataType`).
If the provided type is unknown, the function must return :class:`nullptr`.

.. code-block:: cpp

	extern "C" USER_LIB_EXPORT TopicDataType* GetTopicType(const char *name)
	{
		if (strncmp(name, "HelloWorld", 11) == 0)
		{
			return new HelloWorldPubSubType();
		}
		return nullptr;
	}

The returned type, can be built using dynamic data, using an already generated IDL statically or implementing it
directly as :class:`TopicDataType` subclass.

.. code-block:: cpp

	extern "C" USER_LIB_EXPORT TopicDataType* GetTopicType(const char *name)
	{
		if (strncmp(name, "HelloWorld", 11) == 0)
		{
			// Create basic types
			DynamicTypeBuilder_ptr created_type_ulong = DynamicTypeBuilderFactory::GetInstance()->CreateUint32Builder();
			DynamicTypeBuilder_ptr created_type_string = DynamicTypeBuilderFactory::GetInstance()->CreateStringBuilder();
			DynamicTypeBuilder_ptr struct_type_builder = DynamicTypeBuilderFactory::GetInstance()->CreateStructBuilder();

			// Add members to the struct.
			struct_type_builder->AddMember(0, "index", created_type_ulong.get());
			struct_type_builder->AddMember(1, "message", created_type_string.get());
			struct_type_builder->SetName("HelloWorld");

			DynamicType_ptr dynType = struct_type_builder->Build();
			DynamicPubSubType *psType = new DynamicPubSubType(dynType);
			return psType;
		}
		return nullptr;
	}


See the :ref:`examples` for some already working implementations.

Bridge Libraries
^^^^^^^^^^^^^^^^

Integration Services allows us to define bridge libraries to integrate new protocols.
These libraries must offer the following function declarations:

* create_bridge:

.. code-block:: cpp

	extern "C" USER_LIB_EXPORT ISBridge* create_bridge(const char* name,
		const std::vector<std::pair<std::string, std::string>> *config)
	{
		CustomBridge* bridge = new CustomBridge(name, config);
		return bridge;
	}

As you can see, the instantiated bridge must implement :ref:`isbridge`.
Bridges are in charge of communicating subscribers with publisher and apply transformation functions as defined in
the :ref:`connector`.

* create_subscriber:

.. code-block:: cpp

	extern "C" USER_LIB_EXPORT ISSubscriber* create_subscriber(ISBridge *bridge, const char* name,
		const std::vector<std::pair<std::string, std::string>> *config)
	{
		CustomSubscriber* subscriber = new CustomSubscriber(name, config);
		return subscriber;
	}

The subscriber returned must implement :ref:`issubscriber`.
Subscribers must be able to receive data from the origin protocol.


* create_publisher:

.. code-block:: cpp

	extern "C" USER_LIB_EXPORT ISPublisher* create_publisher(ISBridge *bridge, const char* name,
		const std::vector<std::pair<std::string, std::string>> *config)
	{
		CustomPublisher* publisher = new CustomPublisher(name, config);
		return publisher;
	}

The publisher returned must implement :ref:`ispublisher`.
Publishers must be able to send data to the destination protocol.


In all functions, a vector of pairs of strings is provided if any property exists for each node in the xml
configuration file (see :ref:`Integration Services XML Configuration` for more information).

If some functions want to use the default implementation (RTPS), they must return :class:`nullptr`.

Integration Services will deallocate these objects from memory when the bridge is stopped.

See :ref:`Integration Services architecture` section for more information about the interfaces that any *Bridge Library*
must implement.

The responsability of how to instantiate your bridge, publisher and/or subscriber is on your *Bridge Library*,
but remember that "RTPS" publisher and subscribers will be filled automatically by ISManager with the configuration
from the *participant* node of the :ref:`Integration Services XML Configuration`.

See the :ref:`examples` for some already working implementations.
