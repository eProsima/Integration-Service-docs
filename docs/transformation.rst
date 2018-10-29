Transformation Libraries
========================

Integration Services allow us to define transformation functions that will be applied on each :ref:`connector`.
Transformation functions are static functions that receives the input data and apply some transformation and stores the result in the output data.
These functions must have one of follow interfaces:

Static Data
~~~~~~~~~~~

.. code-block:: cpp

	extern "C" USER_LIB_EXPORT void HelloWorldToKey(SerializedPayload_t* inputBuffer, SerializedPayload_t* outputBuffer)
	{
		// Input HelloWorld Data
		HelloWorldPubSubType hwPubSub;
		HelloWorld hwData;
		hwPubSub.deserialize(inputBuffer, &hwData);

		// Input key Data
		samplePubSubType keyPubSub;
		sample keyData;

		// Custom transformation
		keyData.index() = hwData.index() % 256;
		keyData.key_value() = hwData.index() % 256;

		// Serialize keys
		outputBuffer->reserve(static_cast<uint32_t>(keyPubSub.getSerializedSizeProvider(&keyData)()));
		keyPubSub.serialize(&keyData, outputBuffer);
	}


Dynamic Data
~~~~~~~~~~~~

.. code-block:: cpp

    extern "C" USER_LIB_EXPORT void HelloWorldToKey(DynamicData* hwData, DynamicData* keysData)
    {
        // Custom transformation
        uint32_t temp = hwData->GetUint32Value(0);
        keysData->SetByteValue(temp % 256, 0);
        keysData->SetByteValue(temp % 256, 1);
    }

In both cases, the transformation function will parse the inputData, modify it at will and will store the result into outputData.
See the :ref:`examples` for some already working implementations.