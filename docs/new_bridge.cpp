// Include start
#include "ISBridgeProtocol.h"
// Include end

// Define start
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
// Define end
// create_bridge start
extern "C" USER_LIB_EXPORT ISBridge* create_bridge(const char* name,
    const std::vector<std::pair<std::string, std::string>> *config)
{
    return nullptr;
}
// create_bridge end
// create_reader start
extern "C" USER_LIB_EXPORT ISReader* create_reader(ISBridge *bridge, const char* name,
    const std::vector<std::pair<std::string, std::string>> *config)
{
    return new ProtocolReader(name, config);
}
// create_reader end
// create_writer start
extern "C" USER_LIB_EXPORT ISWriter* create_writer(ISBridge *bridge, const char* name,
    const std::vector<std::pair<std::string, std::string>> *config)
{
    return new ProtocolWriter(name, config);
}
// create_writer end