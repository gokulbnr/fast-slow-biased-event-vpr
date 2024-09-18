#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <arpa/inet.h>
#include <thread>

class DvsFbkController {
    
    public:
        DvsFbkController(int, int, int, char*);
        ~DvsFbkController();

        int startThreads();
        // void stopThreads();
        
    private:
        // ROS stuff
        ros::NodeHandle nh;

        // For socket programming
        int PORT;
        int BUFFER_SZ;
        char* IP_ADDR;
        int WRITE_PORT;
        int serverSocket;
        int writeServerSocket;
        struct sockaddr_in write_server_addr;

        // UDP data handling
        std::deque<ssize_t> bytes_read;
        std::deque<char*> data_buffer;

        // For timestamp handling
        uint32_t prev_timestamp;
        double start_timestamp;
        double initialTime;
        double currentTime;

        // Thread instances
        std::thread udpReaderThread;
        std::thread processBytesThread;

        // DVS Feedback
        int dvs_count;

        // DVS bias parameters
        int PrBp_fine;
        int PrSFBp_fine;
        int OnBn_fine;
        int OffBn_fine;

        // DVS constraints
        float min_dvs_rate;
        float max_dvs_rate;

        // Slow Changes constraints
        int upcount;
        int downcount;
        float max_pixel_firing_rate_max;
        float max_pixel_firing_rate_min;


        // Private functions
        int connectSocket();
        void udpReadLoop();
        void processBytes();
        int dvsInitializations();
        void fbk_control(float);
};

DvsFbkController::DvsFbkController(int port_number, int write_port_number, int buffer_size, char* ip_address) : PORT(port_number), WRITE_PORT(write_port_number), BUFFER_SZ(buffer_size), IP_ADDR(ip_address) {
    // Initialize ROS node
    image_transport::ImageTransport it(nh);
    
    // Temporary variables
    prev_timestamp = -1;
    start_timestamp = -1;

    if(connectSocket() == 1)
        ROS_INFO("Socket connected successfully");
    else
        ROS_ERROR("Error connecting socket");

    dvs_count = 0;
    max_pixel_firing_rate_max = 57;
    max_pixel_firing_rate_min = 4;

    if(dvsInitializations() == 1)
        ROS_INFO("DVS biases initialized successfully");
    else
        ROS_ERROR("Error initializing DVS biases");

    ROS_INFO("DVS Controller object created successfully");
}

int DvsFbkController::connectSocket() {
    // Create socket
    serverSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (serverSocket == -1) {
        ROS_ERROR("Error creating socket");
        return 0;
    }

    writeServerSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (writeServerSocket == -1) {
        ROS_ERROR("Error creating socket");
        return 0;
    }

    // Set the receive buffer size
    if (setsockopt(serverSocket, SOL_SOCKET, SO_RCVBUF, &BUFFER_SZ, sizeof(BUFFER_SZ)) == -1) {
        std::cout << "Error setting socket buffer size" << std::endl;
    }

    // Bind to address
    sockaddr_in serverAddress{};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = inet_addr(IP_ADDR);
    serverAddress.sin_port = htons(PORT);

    std::memset(&write_server_addr, 0, sizeof(write_server_addr));
    write_server_addr.sin_family = AF_INET;
    write_server_addr.sin_port = htons(WRITE_PORT); // Change this to the desired port
    write_server_addr.sin_addr.s_addr = inet_addr(IP_ADDR);

    if (bind(serverSocket, reinterpret_cast<sockaddr*>(&serverAddress), sizeof(serverAddress)) == -1) {
        ROS_ERROR("Error binding socket");
        close(serverSocket);
        return 0;
    }
    return 1;
}

DvsFbkController::~DvsFbkController() {
    // outputFile.close();
    // Close socket
    close(serverSocket);
    close(writeServerSocket);
    ROS_INFO("DVS Controller object destroyed successfully");
}

int DvsFbkController::startThreads() {
    initialTime = ros::Time::now().toSec();
    udpReaderThread = std::thread(&DvsFbkController::udpReadLoop, this);
    processBytesThread = std::thread(&DvsFbkController::processBytes, this);
    return 1;
}

void DvsFbkController::udpReadLoop() {
    char dataBuffer[BUFFER_SZ];
    sockaddr_in clientAddress{};
    socklen_t clientAddrLen = sizeof(clientAddress);

    ROS_INFO("UDP Read Loop started");
    while (ros::ok()) {
        ssize_t bytesRead = recvfrom(serverSocket, dataBuffer, sizeof(dataBuffer), 0, reinterpret_cast<sockaddr*>(&clientAddress), &clientAddrLen);
        if (bytesRead == -1) {
            ROS_ERROR("Error receiving data");
            break;
        }
        data_buffer.push_back(dataBuffer);
        bytes_read.push_back(bytesRead);
    }
}

void DvsFbkController::processBytes() {
    while (ros::ok()) {
        if (!bytes_read.empty() && !data_buffer.empty()) {
            char* dataBuffer = data_buffer.front();
            ssize_t bytesRead = bytes_read.front();
            bytes_read.pop_front();
            data_buffer.pop_front();

            uint32_t sequence_number;
            std::memcpy(&sequence_number, &dataBuffer[0], sizeof(uint32_t));
            uint32_t num_events = int(((bytesRead / 4) - 1) / 2);

            for (int i = 0; i < num_events; ++i) {
                uint32_t address, timestamp;
                std::memcpy(&address, &dataBuffer[4 + (i * 8)], sizeof(uint32_t));
                address = ntohl(address);
                int bit_31 = (address >> 31) & 1;
                if(bit_31 == 0) {               
                    std::memcpy(&timestamp, &dataBuffer[8 + (i * 8)], sizeof(uint32_t));
                    timestamp = ntohl(timestamp);
                    if (prev_timestamp == -1) prev_timestamp = timestamp;
                    currentTime = initialTime + ((timestamp - prev_timestamp) * 1e-6);
                    if (start_timestamp == -1) start_timestamp = currentTime;
                    if (currentTime - start_timestamp >= 0.3) {
                        dvs_count++;
                        std::cout << float(dvs_count) / 0.3 << std::endl;
                        fbk_control(float(dvs_count) / 0.3);
                        start_timestamp = currentTime;
                        dvs_count = 0;
                    }
                    else dvs_count++;
                }
            }
        }
    }
}