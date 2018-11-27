/* Type A
struct DDSTypeA
{
    long index;
    string category;
    string info;
};
// End Type A */
/* Type B
struct DDSTypeB
{
    octet sequenceNumber;
    string message;
};
// End Type B */
// Include
#include "DDSTypeAPubSubTypes.h"
#include "DDSTypeBPubSubTypes.h"
// End include
// Definitions
#if defined(_WIN32) && defined (BUILD_SHARED_LIBS)
    #if defined (_MSC_VER)
        #pragma warning(disable: 4251)
    #endif
  #if defined(integration_services_EXPORTS)
      #define  USER_LIB_EXPORT __declspec(dllexport)
  #else
    #define  USER_LIB_EXPORT __declspec(dllimport)
  #endif
#else
  #define USER_LIB_EXPORT
#endif
// End Definitions
// A to B
extern "C" void USER_LIB_EXPORT transformA_to_B(
        SerializedPayload_t *serialized_input,
        SerializedPayload_t *serialized_output)
{
    // Types definition
    DDSTypeA input_data;
    DDSTypeAPubSubType input_pst;
    DDSTypeB output_data;
    DDSTypeBPubSubType output_pst;

    // Deserialization
    input_pst.deserialize(serialized_input, &input_data);

    // Data transformation from A to B
    output_data.sequenceNumber = input_data.index % 256;
    output_data.message = input_data.category + ":" + input_data.info;

    // Serialization
    serialized_output->reserve(output_pst.m_typeSize);
    output_pst.serialize(&output_data, serialized_output);
}
// End A to B
// B to A
extern "C" void USER_LIB_EXPORT transformB_to_A(
        SerializedPayload_t *serialized_input,
        SerializedPayload_t *serialized_output)
{
    // Types definition
    DDSTypeB input_data;
    DDSTypeBPubSubType input_pst;
    DDSTypeA output_data;
    DDSTypeAPubSubType output_pst;

    // Deserialization
    input_pst.deserialize(serialized_input, &input_data);

    // Data transformation from B to A
    output_data.index = input_data.sequenceNumber;
    std::istringstream iss(input.message);
    std::string token;
    std::getline(iss, token, ':');
    output_data.category = token;
    std::getline(iss, token, ':');
    output_data.info = token;

    // Serialization
    serialized_output->reserve(output_pst.m_typeSize);
    output_pst.serialize(&output_data, serialized_output);
}
// End B to A