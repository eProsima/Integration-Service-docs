// Static Data Start
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
// Static Data End

// Dynamic Data Start
extern "C" USER_LIB_EXPORT void HelloWorldToKey(DynamicData* inputData, DynamicData* outputData)
{
	// Custom transformation
	uint32_t temp = inputData->GetUint32Value(0);
	outputData->SetByteValue(temp % 256, 0);
	outputData->SetByteValue(temp % 256, 1);
}
// Dynamic Data End

// Create Bridge Start
extern "C" USER_LIB_EXPORT ISBridge* create_bridge(const char* name,
	const std::vector<std::pair<std::string, std::string>> *config)
{
	CustomBridge* bridge = new CustomBridge(name, config);
	return bridge;
}
// Create Bridge End

// Create Reader Start
extern "C" USER_LIB_EXPORT ISReader* create_reader(ISBridge *bridge, const char* name,
	const std::vector<std::pair<std::string, std::string>> *config)
{
	CustomReader* reader = new CustomReader(name, config);
	return reader;
}
// Create Reader End

// Create Writer Start
extern "C" USER_LIB_EXPORT ISWriter* create_writer(ISBridge *bridge, const char* name,
	const std::vector<std::pair<std::string, std::string>> *config)
{
	CustomWriter* writer = new CustomWriter(name, config);
	return writer;
}
// Create Writer End

// Class Bridge Start
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
// Class Bridge End

// Class Writer Start
class ISWriter
{
public:
	virtual bool write(eprosima::fastrtps::rtps::SerializedPayload_t* /*data*/) = 0;
	virtual bool write(eprosima::fastrtps::types::DynamicData* /*data*/) = 0;
	virtual ISBridge* setBridge(ISBridge *);
};
// Class Writer End

// Writer Write Start
bool write([...]) override { return false; }
// Writer Write End

// Class Reader Start
class ISReader
{
public:
	virtual void addBridge(ISBridge* bridge);
	virtual void on_received_data(eprosima::fastrtps::rtps::SerializedPayload_t* payload);
	virtual void on_received_data(eprosima::fastrtps::types::DynamicData* data);
};
// Class Reader End

// Reader Read Start
void on_received_data([...]) override { }
// Reader Read End

// Topic Type 1 Start
extern "C" USER_LIB_EXPORT TopicDataType* GetTopicType(const char *name);
// Topic Type 1 End

// Topic Type 2 Start
extern "C" USER_LIB_EXPORT TopicDataType* GetTopicType(const char *name)
{
	if (strncmp(name, "HelloWorld", 11) == 0)
	{
		return new HelloWorldPubSubType();
	}
	return nullptr;
}
// Topic Type 2 End

// Topic Type 3 Start
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
// Topic Type 3 End
