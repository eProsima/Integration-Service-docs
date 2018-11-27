// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Define start
#ifndef _PROTOCOL_READER_H_
#define _PROTOCOL_READER_H_
// Define end

// Include start
#include "ISBridge.h"
// Include end
// ProtocolReader start
class ProtocolReader : public ISReader
{
private:
    std::string config1;
    std::string config2;
    bool config3;
public:
    ProtocolReader(const std::string &name, const std::vector<std::pair<std::string, std::string>> *config);
    ~ProtocolReader() override;

    void checkUpdates(); // Our custom reading method
};
// ProtocolReader end
// Constructor start
ProtocolReader::ProtocolReader(const std::string &name, const std::vector<std::pair<std::string, std::string>> *config)
{
    // Configure ProtocolReader instance with the given config
    // For example:
    for (auto pair : *config)
    {
        try
        {
            if (pair.first.compare("CONFIG1") == 0)
            {
                config1 = pair.second;
            }
            else if (pair.first.compare("CONFIG2") == 0)
            {
                config2 = pair.second;
            }
            else if (pair.first.compare("CONFIG3") == 0)
            {
                config3 = pair.second.compare("TRUE") == 0;
            }
        }
        catch (...)
        {
            return;
        }
    }
}
// Constructor end
// Read start
void ProtocolReader::checkUpdates()
{
    // Ask the source protocol for updates. Maybe called by a timer, or by event...
    // For example: A Get request to a WebService
    try
    {
        curlpp::Cleanup myCleanup;
        curlpp::Easy myRequest;
        std::ostringstream response;

        // Set the URL.
        // entity/{entityId}/attrs/{attrName}
        myRequest.setOpt<Url>(url + "/entity/" + config1 + "/attrs/" + config2);
        // Store the result in response
        myRequest.setOpt(new curlpp::options::WriteStream(&response));

        // Send request and get a result.
        myRequest.perform();

        // Custom TopicDataType that encapsulates the response:
        // struct Response
        // {
        //     string entityId; // To which entity apply the data (goes into the URL)
        //     string data; // response
        // };
        ResponsePubSubType response_pst;
        Response responseData;
        // Fill the data
        responseData.entityId = config1;
        responseData.data = response.str();
        SerializedPayload_t payload;
        response_pst.serialize(&payload, &responseData); // Serialize the Response into the payload

        // Call on_received_data method
        on_received_data(&payload);
    }
    catch (curlpp::LogicError & e)
    {
        LOG_ERROR(e.what()); // A LOG System. You should have access to the Fast-RTPS's Logger too.
    }
    catch (curlpp::RuntimeError & e)
    {
        LOG_ERROR(e.what());
    }
}
// Read end
// Destructor start
ProtocolReader::~ProtocolReader()
{
    // Free any taken resources, memory, etc.
}
// Destructor end
// Endif start
#endif // _PROTOCOL_READER_H_
// Endif end