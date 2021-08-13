#include "Headers/EngineClient.h"

EngineClient::EngineClient(string ip, string port)
{
    socketDescriptor = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketDescriptor == -1)
    {
        std::cout << "Could not create socket\n";
    }
    address = {0};
    address.sin_family = AF_INET;
    inet_aton(ip.data(), &address.sin_addr);
    address.sin_port = htons(atoi(port.data()));
    int opt = 1;
    setsockopt(socketDescriptor, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));
}
bool EngineClient::sendToEngine(PoseData dataToSend)
{
    /*std::cout << "Sending to engine...\n";
    std::cout << "Position: x:" << dataToSend.position.x <<" y:"<< dataToSend.position.y << " z:" << dataToSend.position.z << std::endl;
*/
    CameraData data;
    data.Header = 0xD1;
    data.Checksum = 1;
    data.CameraId = 0;
    convertPoseToCameraData(dataToSend, 1, 1, &data);
    data.UserDefined = 1;
    if (sendto(socketDescriptor, (char *)&data, sizeof(data), 0, (struct sockaddr *)&address, sizeof(address)) == -1)
    {

        std::cout << "Could not send data to engine\n";
        return false;
    };
    return true;
}

EngineClient::~EngineClient()
{
    close(socketDescriptor);
    socketDescriptor = 0;
    address = {0};
    return;
}