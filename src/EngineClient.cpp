#include "Headers/EngineClient.h"
EngineClient::EngineClient(string ip, string port)
{
#ifdef _WIN32
	LoadWSA();
#endif
	socketDescriptor = socket(AF_INET, SOCK_DGRAM, 0);
	if (socketDescriptor == -1)
	{
		std::cerr << "Could not create socket" << endl;
	}
	address = { 0 };
	address.sin_family = AF_INET;
	inet_pton(AF_INET, (ip).c_str(), &(address.sin_addr));
	address.sin_port = htons(atoi(port.data()));
	
}

EngineClient::~EngineClient() {
}

bool EngineClient::sendToEngine(Vector3& position, Quaternion& rotation, unsigned int zoom, unsigned int focus)
{
	CameraData data;
	data.Header = 0xD1;
	data.Checksum = 1;
	data.CameraId = 0;
	convertPoseToCameraData(position, rotation, zoom, focus, &data);
	data.UserDefined = 1;
	if (sendto(socketDescriptor, (char*)&data, sizeof(data), 0, (struct sockaddr*)&address, sizeof(address)) == -1)
	{

		cerr << "Could not send data to engine\n";
		return false;
	};
	return true;
}



#ifdef _WIN32

void EngineClient::LoadWSA() {

	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
		fprintf(stderr, "Not able to startup winsock dll");
	}
}
#endif