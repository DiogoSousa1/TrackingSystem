#include "Headers/EngineClient.h"

EngineClient::EngineClient(string ip, string port)
{
    socketDescriptor = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketDescriptor == -1)
    {
        std::cerr << "Could not create socket" << endl;
    }
    address = {0};
    address.sin_family = AF_INET;
    inet_aton(ip.data(), &address.sin_addr);
    address.sin_port = htons(atoi(port.data()));
    int opt = 1;
    setsockopt(socketDescriptor, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));
}
bool EngineClient::sendToEngine(Vector3& position, Quaternion& rotation, unsigned int zoom, unsigned int focus)
{
    CameraData data;
    data.Header = 0xD1;
    data.Checksum = 1;
    data.CameraId = 0;
    convertPoseToCameraData(position, rotation, zoom, focus, &data);
    data.UserDefined = 1;
    if (sendto(socketDescriptor, (char *)&data, sizeof(data), 0, (struct sockaddr *)&address, sizeof(address)) == -1)
    {

        cerr << "Could not send data to engine\n";
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