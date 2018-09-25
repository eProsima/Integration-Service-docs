Types Libraries
===============

Integration Services allow us to define types libraries to create custom data types. These libraries must offer a function with the follow declaration:

.. code-block:: cpp

    extern "C" USER_LIB_EXPORT TopicDataType* GetTopicType(const char *name);

It will be called with the TopicType name, and must return an instance of it (subclass of TopicDataType).
If the provided type is unknown, the function must return nullptr.

The returned type, can be built using dynamic data, an already generated IDL statically or implementing it
directly as TopicDataType subclass.