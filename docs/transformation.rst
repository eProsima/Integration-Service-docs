Transformation Libraries
========================

Integration Services allow us to define transformation functions that will be applied on each :ref:`connector`.


These functions must have one of follow interfaces:

.. code-block:: cpp

    extern "C" USER_LIB_EXPORT void transform(DynamicData* inputData, DynamicData* outputData)
    {
        [...]
    }

To make use of dynamic data, or:

.. code-block:: cpp

    extern "C" USER_LIB_EXPORT void transform(SerializedPayload_t* inputData, SerializedPayload_t* outputData)
    {
        [...]
    }

To use static data.

In both cases, the transformation function will parse the inputData, modify it at will and will store the result into outputData.
See the :ref:`examples` for some already working implementations.