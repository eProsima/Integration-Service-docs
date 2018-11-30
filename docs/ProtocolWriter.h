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
#ifndef _PROTOCOL_WRITER_H_
#define _PROTOCOL_WRITER_H_
// Define end

// Include start
#include <curlpp>
#include "ISBridge.h"
// Include end
// ProtocolWriter start
class ProtocolWriter : public ISWriter
{
private:
    std::string config1;
    std::string config2;
    bool config3;
public:
    ProtocolWriter(const std::string &name, const std::vector<std::pair<std::string, std::string>> *config);
    ~ProtocolWriter() override;

    bool write(SerializedPayload_t*) override;
    bool write(eprosima::fastrtps::types::DynamicData*) override { return false; }; // We don't use it
};
// ProtocolWriter end
// Constructor start
ProtocolWriter::ProtocolWriter(const std::string &name, const std::vector<std::pair<std::string, std::string>> *config)
{
    // Configure ProtocolWriter instance with the given config
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
// Write start
bool ProtocolWriter::write(SerializedPayload_t* payload)
{
    // Manage the payload to write into destination protocol
    // For example: A POST request to a WebService
    long code = 600;
    try
    {
        curlpp::Cleanup cleaner;
        curlpp::Easy request;

        // Custom TopicDataType that encapsulates a JSON as:
        // struct Json
        // {
        //     string entityId; // To which entity apply the data (goes into the URL)
        //     string data; // JSON data
        // };
        JsonPubSubType json_pst;
        Json json;
        json_pst.deserialize(payload, &json); // Deserialize the payload into the Json structure

        // Retrieve the data
        std::string entityId = json.entityId();
        std::string payload = json.data();
        // Create a Curl request
        request.setOpt(new curlpp::options::Url(url + "/entity/" + entityId + "/update"));
        std::list<std::string> header;
        header.push_back("Content-Type: application/json");
        request.setOpt(new curlpp::options::HttpHeader(header));
        request.setOpt(new curlpp::options::PostFields(payload));
        request.setOpt(new curlpp::options::PostFieldSize((long)payload.length()));

        // Perform the request
        request.perform();
        code = curlpp::infos::ResponseCode::get(request);
    }
    catch (curlpp::LogicError & e)
    {
        LOG_ERROR(e.what()); // A LOG System. You should have access to the Fast-RTPS's Logger too.
        code = 601;
    }
    catch (curlpp::RuntimeError & e)
    {
        LOG_ERROR(e.what());
        code = 602;
    }

    return (code / 100) == 2; // Return true if success (code family 200)
}
// Write end
// Destructor start
ProtocolWriter::~ProtocolWriter()
{
    // Free any taken resources, memory, etc.
}
// Destructor end
// Endif start
#endif // _PROTOCOL_WRITER_H_
// Endif end