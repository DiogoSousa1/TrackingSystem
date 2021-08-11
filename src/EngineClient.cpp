#include "Headers/EngineClient.h"

EngineClient::EngineClient(string ip, string port)
{
    sockerDescriptor = socket(AF_INET, SOCK_DGRAM, 0);
    address = {0};
    address.sin_family = AF_INET;
    //TODO: CHECK THIS STUFF
    address.sin_addr.s_addr = stoul(ip);
    address.sin_port = htons(6301);
    connect(sockerDescriptor, (sockaddr *)&address, sizeof(address));
}
bool EngineClient::sendToEngine(poseData dataToSend)
{
    FreeDOperators operators;
    CameraData data;
    data.Header = 0xD1;
    data.CameraId = 0;
    convertPoseToCameraData(dataToSend, 1, 1, &data);
    data.UserDefined = 1;
    if(sendto(sockerDescriptor,(char*)&data,sizeof(data), 0, (struct sockaddr*)&address,sizeof(address)) == -1) {
        std::cout << "Could not send data to engine";
        return false;
    };
    return true;
}

EngineClient::~EngineClient()
{
}