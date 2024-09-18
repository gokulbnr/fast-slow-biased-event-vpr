#include <fast_and_slow_controller/DvsFbkControllerClass.hpp>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <cstring>

int DvsFbkController::dvsInitializations() {
    ssize_t bytes_sent;
    char* message;
    min_dvs_rate = 5e5;
    max_dvs_rate = 2.5e6;

    message = "setic_PrBp 6";
    bytes_sent = sendto(writeServerSocket, message, strlen(message), 0, (struct sockaddr*)&write_server_addr, sizeof(write_server_addr));
    if (bytes_sent < 0) perror("Error sending data");
    message = "setif_PrBp 119";
    bytes_sent = sendto(writeServerSocket, message, strlen(message), 0, (struct sockaddr*)&write_server_addr, sizeof(write_server_addr));
    if (bytes_sent < 0) perror("Error sending data");
    PrBp_fine = 119;

    message = "setic_PrSFBp 7";
    bytes_sent = sendto(writeServerSocket, message, strlen(message), 0, (struct sockaddr*)&write_server_addr, sizeof(write_server_addr));
    if (bytes_sent < 0) perror("Error sending data");
    message = "setif_PrSFBp 32";
    bytes_sent = sendto(writeServerSocket, message, strlen(message), 0, (struct sockaddr*)&write_server_addr, sizeof(write_server_addr));
    if (bytes_sent < 0) perror("Error sending data");
    PrSFBp_fine = 32;

    message = "setic_OnBn 1";
    bytes_sent = sendto(writeServerSocket, message, strlen(message), 0, (struct sockaddr*)&write_server_addr, sizeof(write_server_addr));
    if (bytes_sent < 0) perror("Error sending data");
    message = "setif_OnBn 63";
    bytes_sent = sendto(writeServerSocket, message, strlen(message), 0, (struct sockaddr*)&write_server_addr, sizeof(write_server_addr));
    if (bytes_sent < 0) perror("Error sending data");
    OnBn_fine = 63;

    message = "setic_OffBn 5";
    bytes_sent = sendto(writeServerSocket, message, strlen(message), 0, (struct sockaddr*)&write_server_addr, sizeof(write_server_addr));
    if (bytes_sent < 0) perror("Error sending data");
    message = "setif_OffBn 168";
    bytes_sent = sendto(writeServerSocket, message, strlen(message), 0, (struct sockaddr*)&write_server_addr, sizeof(write_server_addr));
    if (bytes_sent < 0) perror("Error sending data");
    OffBn_fine = 168;

    return 1;
}

void DvsFbkController::fbk_control(float event_rate) {
    ssize_t bytes_sent;
    int max_pixel_firing_rate_setting = max_pixel_firing_rate_max - ((max_pixel_firing_rate_max - max_pixel_firing_rate_min) * (event_rate - min_dvs_rate) / (max_dvs_rate - min_dvs_rate));
    int count_threshold = 2;

    max_pixel_firing_rate_setting = (max_pixel_firing_rate_setting > max_pixel_firing_rate_max) ? (max_pixel_firing_rate_max) : (max_pixel_firing_rate_setting);
    max_pixel_firing_rate_setting = (max_pixel_firing_rate_setting < max_pixel_firing_rate_min) ? (max_pixel_firing_rate_min) : (max_pixel_firing_rate_setting);

    if(max_pixel_firing_rate_setting == max_pixel_firing_rate_max) {
        max_pixel_firing_rate_setting = max_pixel_firing_rate_max;
        upcount++;
        if(upcount == count_threshold) {
            upcount = 0;
            
            std::string cmd;
            char* message;

            PrBp_fine += 5;
            cmd = "setif_PrBp ";
            message = new char[cmd.length() + std::strlen(std::to_string(int(PrBp_fine)).c_str()) + 1];
            std::strcpy(message, cmd.c_str());
            message = std::strcat(message, std::to_string(PrBp_fine).c_str());
            std::cout << message << std::endl;
            bytes_sent = sendto(writeServerSocket, message, strlen(message), 0, (struct sockaddr*)&write_server_addr, sizeof(write_server_addr));
            if (bytes_sent < 0) perror("Error sending data");

            PrSFBp_fine += 5;
            cmd = "setif_PrSFBp ";
            message = new char[cmd.length() + std::strlen(std::to_string(int(PrSFBp_fine)).c_str()) + 1];
            std::strcpy(message, cmd.c_str());
            message = std::strcat(message, std::to_string(PrSFBp_fine).c_str());
            std::cout << message << std::endl;
            bytes_sent = sendto(writeServerSocket, message, strlen(message), 0, (struct sockaddr*)&write_server_addr, sizeof(write_server_addr));
            if (bytes_sent < 0) perror("Error sending data");

            OnBn_fine -= 5;
            cmd = "setif_OnBn ";
            message = new char[cmd.length() + std::strlen(std::to_string(int(OnBn_fine)).c_str()) + 1];
            std::strcpy(message, cmd.c_str());
            message = std::strcat(message, std::to_string(OnBn_fine).c_str());
            std::cout << message << std::endl;
            bytes_sent = sendto(writeServerSocket, message, strlen(message), 0, (struct sockaddr*)&write_server_addr, sizeof(write_server_addr));
            if (bytes_sent < 0) perror("Error sending data");

            OffBn_fine += 5;
            cmd = "setif_OffBn ";
            message = new char[cmd.length() + std::strlen(std::to_string(int(OffBn_fine)).c_str()) + 1];
            std::strcpy(message, cmd.c_str());
            message = std::strcat(message, std::to_string(OffBn_fine).c_str());
            std::cout << message << std::endl;
            bytes_sent = sendto(writeServerSocket, message, strlen(message), 0, (struct sockaddr*)&write_server_addr, sizeof(write_server_addr));
            if (bytes_sent < 0) perror("Error sending data");
        }
    }
    else if(max_pixel_firing_rate_setting == max_pixel_firing_rate_min) {
        max_pixel_firing_rate_setting = max_pixel_firing_rate_min;
        downcount++;
        if(downcount == count_threshold) {
            downcount = 0;
            std::string cmd;
            char* message;

            PrBp_fine -= 5;
            cmd = "setif_PrBp ";
            message = new char[cmd.length() + std::strlen(std::to_string(int(PrBp_fine)).c_str()) + 1];
            std::strcpy(message, cmd.c_str());
            message = std::strcat(message, std::to_string(PrBp_fine).c_str());
            std::cout << message << std::endl;
            bytes_sent = sendto(writeServerSocket, message, strlen(message), 0, (struct sockaddr*)&write_server_addr, sizeof(write_server_addr));
            if (bytes_sent < 0) perror("Error sending data");

            PrSFBp_fine -= 5;
            cmd = "setif_PrSFBp ";
            message = new char[cmd.length() + std::strlen(std::to_string(int(PrSFBp_fine)).c_str()) + 1];
            std::strcpy(message, cmd.c_str());
            message = std::strcat(message, std::to_string(PrSFBp_fine).c_str());
            std::cout << message << std::endl;
            bytes_sent = sendto(writeServerSocket, message, strlen(message), 0, (struct sockaddr*)&write_server_addr, sizeof(write_server_addr));
            if (bytes_sent < 0) perror("Error sending data");

            OnBn_fine += 5;
            cmd = "setif_OnBn ";
            message = new char[cmd.length() + std::strlen(std::to_string(int(OnBn_fine)).c_str()) + 1];
            std::strcpy(message, cmd.c_str());
            message = std::strcat(message, std::to_string(OnBn_fine).c_str());
            std::cout << message << std::endl;
            bytes_sent = sendto(writeServerSocket, message, strlen(message), 0, (struct sockaddr*)&write_server_addr, sizeof(write_server_addr));
            if (bytes_sent < 0) perror("Error sending data");

            OffBn_fine -= 5;
            cmd = "setif_OffBn ";
            message = new char[cmd.length() + std::strlen(std::to_string(int(OffBn_fine)).c_str()) + 1];
            std::strcpy(message, cmd.c_str());
            message = std::strcat(message, std::to_string(OffBn_fine).c_str());
            std::cout << message << std::endl;
            bytes_sent = sendto(writeServerSocket, message, strlen(message), 0, (struct sockaddr*)&write_server_addr, sizeof(write_server_addr));
            if (bytes_sent < 0) perror("Error sending data");
        }
    }
    else {
        upcount = 0;
        downcount = 0;
    }
    
    std::string cmd = "setif_RefrBp ";
    char* message = new char[cmd.length() + std::strlen(std::to_string(int(max_pixel_firing_rate_setting)).c_str()) + 1];
    std::strcpy(message, cmd.c_str());
    message = std::strcat(message, std::to_string(max_pixel_firing_rate_setting).c_str());
    std::cout << message << std::endl;
    bytes_sent = sendto(writeServerSocket, message, strlen(message), 0, (struct sockaddr*)&write_server_addr, sizeof(write_server_addr));
    if (bytes_sent < 0) perror("Error sending data");
}

int main(int argc, char **argv) {

    // Initialize ROS node
    ros::init(argc, argv, "dvs_fbk_controller_node");

    // Read configuration info from
    std::string package_path = ros::package::getPath("fast_and_slow_controller_ros");
    std::string config_file_path = package_path + "/config/config.yaml";
    YAML::Node config = YAML::LoadFile(config_file_path);

    // Interpret configuration info
    std::string ip_address = config["ip_address"].as<std::string>();
    char* ip_address_char = new char[ip_address.length() + 1];
    std::strcpy(ip_address_char, ip_address.c_str());
    int port_number = config["port_number"].as<int>();
    int write_port_number = config["write_port_number"].as<int>();
    int buffer_size = config["buffer_size"].as<int>();
    ROS_INFO("Configuration info loaded successfully");

    // Initialize DVS Controller Object
    DvsFbkController dvs_fbk_controller(port_number, write_port_number, buffer_size, ip_address_char);

    if(dvs_fbk_controller.startThreads() == 1)
        ROS_INFO("All process threads started successfully");
    else
        ROS_ERROR("Error starting all threads");

    // ROS loop
    while (ros::ok()) {
        // Process buffered data
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    // dvs_fbk_controller.stopThreads();
    return 0;
}