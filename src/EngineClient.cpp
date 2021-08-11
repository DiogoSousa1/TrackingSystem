#include "Headers/EngineClient.h"

EngineClient::EngineClient(string ip, string port)
{
    sockerDescriptor = socket(AF_INET, SOCK_DGRAM, 0);
    address = {0};
    address.sin_family = AF_INET;

    address.sin_addr.s_addr = htons(stoul(ip));
    address.sin_port = htons(stoul(port));
    connect(sockerDescriptor, (sockaddr *)&address, sizeof(address));

}
bool EngineClient::sendToEngine(poseData dataToSend)
{
    return true;
}

EngineClient::~EngineClient()
{
}