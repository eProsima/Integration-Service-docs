Types Libraries
===============

Integration Services allow us to define types libraries to create custom data types. These libraries must offer a function with the follow declaration:

.. code-block:: cpp

    extern "C" USER_LIB_EXPORT TopicDataType* GetTopicType(const char *name);

It will be called with the TopicType name, and must return an instance of it (subclass of TopicDataType).
If the provided type is unknown, the function must return nullptr.

.. code-block:: cpp

	extern "C" USER_LIB_EXPORT TopicDataType* GetTopicType(const char *name)
	{
		if (strncmp(name, "HelloWorld", 11) == 0)
		{
			return new HelloWorldPubSubType();
		}
		return nullptr;
	}

The returned type, can be built using dynamic data, an already generated IDL statically or implementing it
directly as TopicDataType subclass.

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