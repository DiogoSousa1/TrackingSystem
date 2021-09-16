#include "Headers/NetworkClient.h"
NetworkClient::NetworkClient(string sendIP, string sendPort, string receiveIP, string receivePort)
{
#ifdef _WIN32
	LoadWSA();
#endif
	initializeSockets(sendIP, sendPort, receiveIP, receivePort);
}

void NetworkClient::initializeSockets(string sendIP, string sendPort, string receiveIP, string receivePort)
{
	//set receiver of data
	receiveSocketDescriptor = socket(AF_INET, SOCK_DGRAM, 0);
	if (receiveSocketDescriptor == -1)
	{
		std::cerr << "Could not create socket" << endl;
	}
	receiveAddress = {0};
	receiveAddress.sin_family = AF_INET;
	inet_pton(AF_INET, (receiveIP).c_str(), &(receiveAddress.sin_addr));
	receiveAddress.sin_port = htons(atoi(receivePort.data()));
	
	//set sender of data
	sendSocketDescriptor = socket(AF_INET, SOCK_DGRAM, 0);
	if (sendSocketDescriptor == -1)
	{
		std::cerr << "Could not create socket" << endl;
	}
	sendAddress = {0};
	sendAddress.sin_family = AF_INET;
	inet_pton(AF_INET, (sendIP).c_str(), &(sendAddress.sin_addr));
	sendAddress.sin_port = htons(atoi(sendPort.data()));
}

NetworkClient::~NetworkClient()
{
}

bool NetworkClient::sendToEngine(Vector3 &position, Quaternion &rotation, unsigned int zoom, unsigned int focus)
{
	CameraData data;
	data.Header = 0xD1;
	data.Checksum = 1;
	data.CameraId = 0;
	convertPoseToCameraData(position, rotation, zoom, focus, &data);
	data.UserDefined = 1;
	if (sendto(sendSocketDescriptor, (char *)&data, sizeof(data), 0, (struct sockaddr *)&sendAddress, sizeof(sendAddress)) == -1)
	{

		cerr << "Could not send data to engine\n";
		return false;
	};
	return true;
}

#ifdef _WIN32

void EngineClient::LoadWSA()
{

	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		fprintf(stderr, "Not able to startup winsock dll");
	}
}
#endif