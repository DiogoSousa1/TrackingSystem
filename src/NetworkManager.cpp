#include "Headers/NetworkManager.h"
NetworkManager::NetworkManager(string sendIP, string sendPort, string receivePort)
{
#ifdef _WIN32
	LoadWSA();
#endif
	initializeSockets(sendIP, sendPort, receivePort);
}

void NetworkManager::initializeSockets(string sendIP, string sendPort, string receivePort)
{
	//open descriptor for receiving data
	receiveSocketDescriptor = socket(AF_INET, SOCK_DGRAM, 0);
	if (receiveSocketDescriptor == -1)
	{
		std::cerr << "Could not create receive socket" << endl;
	}
	receiveAddress = {0};
	receiveAddress.sin_family = AF_INET;
	receiveAddress.sin_port = htons(atoi(receivePort.data()));
	receiveAddress.sin_addr.s_addr = INADDR_ANY;

	//set sender of data
	sendSocketDescriptor = socket(AF_INET, SOCK_DGRAM, 0);
	if (sendSocketDescriptor == -1)
	{
		std::cerr << "Could not create send socket" << endl;
	}
	sendAddress = {0};
	sendAddress.sin_family = AF_INET;
	inet_pton(AF_INET, (sendIP).c_str(), &(sendAddress.sin_addr));
	sendAddress.sin_port = htons(atoi(sendPort.data()));
}

NetworkManager::~NetworkManager()
{
}

void NetworkManager::receiveLensData(int *zoom, int *focus)
{
	thread t([&]
			 {
				 unsigned char *buff = new  unsigned char[30];
				 while (true)
				 {
					 recv(receiveSocketDescriptor, buff, 30, 0);
					 //TODO: FreeD packets from panasonic are 20-25 the values of zoom and focus
					 *zoom = ThreeBytesToSignedInt(buff[20],buff[21], buff[22]);
						*focus = ThreeBytesToSignedInt(buff[23], buff[24], buff[25]); 
					//interpret receive packet here
				}
			 });
}

bool NetworkManager::sendTrackingData(Vector3 &position, Quaternion &rotation, unsigned int zoom, unsigned int focus)
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