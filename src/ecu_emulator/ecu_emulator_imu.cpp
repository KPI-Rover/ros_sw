#include <iostream>
#include <string>
#include <cstring>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstdint>



class UDPClient {
	public:
		UDPClient(const char* ip_address, int port);
		
		UDPClient(const UDPClient&) = delete;
		UDPClient& operator=(const UDPClient&) = delete; 
		UDPClient(UDPClient&&) = delete;             
		UDPClient& operator=(UDPClient&&) = delete;

		
		int Init();
		bool Send(const std::vector<std::uint8_t> &data);
		~UDPClient();

	private:
		int sockfd_;
		sockaddr_in serverStruct_;
		char* server_address_;
		int client_portnum_;
};

UDPClient::UDPClient(const char* ip_address, int port): sockfd_(-1), client_portnum_(port), server_address_(new char[strlen(ip_address) + 1]) {
	strncpy(server_address_, ip_address, strlen(ip_address) + 1);
}

int UDPClient::Init() {
	std::cout << "initialize socket" << '\n';
	sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockfd_ == -1) {
		std::cout << "Error while creating socket" << '\n';
		return -1;
	}

	memset(&serverStruct_, 0, sizeof(serverStruct_));
    serverStruct_.sin_family = AF_INET;
    serverStruct_.sin_addr.s_addr = inet_addr(server_address_); // Server IP address (localhost)
    serverStruct_.sin_port = htons(client_portnum_);

	return 0;
}

bool UDPClient::Send(const std::vector<std::uint8_t> &data) {
	std::cout << "send" << '\n';
	if (sockfd_ < 0) {
		return false;
	}

	return sendto(sockfd_, data.data(), data.size(), 0, reinterpret_cast<sockaddr*>(&serverStruct_), sizeof(serverStruct_)) != -1;

}



UDPClient::~UDPClient() {
	shutdown(sockfd_, SHUT_WR);

	close(sockfd_);
	delete[] server_address_;
}

int main() {

	UDPClient client("127.0.0.1", 9999);
	if (client.Init() == -1) {
		return -1;
	}

	for (int i = 0; i < 10000000; i++)  {
		std::vector<std::uint8_t> data;
		uint8_t command_id = 0x06;
		uint16_t packet_number = i;
		float accel = 0.1;
		float gyro = 0.2;
		float orient = 0.3;
		
		data.push_back(command_id);
		
		printf("%d\n", packet_number);
		packet_number = htons(packet_number);
		
		uint8_t* bytes = reinterpret_cast<uint8_t*>(&packet_number);
    		for (size_t i = 0; i < 2; ++i) {
        		data.push_back(bytes[i]);
    		}
		
		uint32_t uint32Val;

        std::memcpy(&uint32Val, &accel, sizeof(float));
        uint32Val = htonl(uint32Val);
		
		
		bytes = reinterpret_cast<uint8_t*>(&uint32Val);
    		for (int i = 0; i < 3; i++){
	    		for (size_t i = 0; i < 4; ++i) {
				data.push_back(bytes[i]);
	    		}
	    	}
    	
        std::memcpy(&uint32Val, &gyro, sizeof(float));
		uint32Val = htonl(uint32Val);
		
     
        bytes = reinterpret_cast<uint8_t*>(&uint32Val);
        for (int i = 0; i < 3; i++){
            for (size_t i = 0; i < 4; ++i) {
            data.push_back(bytes[i]);
            }
		}
		
        std::memcpy(&uint32Val, &orient, sizeof(float));
		uint32Val = htonl(uint32Val);

		bytes = reinterpret_cast<uint8_t*>(&uint32Val);
    		for (int i = 0; i < 4; i++){
	    		for (size_t i = 0; i < 4; ++i) {
				data.push_back(bytes[i]);
	    		}
		}
		
		
		client.Send(data);

		sleep(1);

	}
	
    return 0;
}