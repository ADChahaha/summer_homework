#pragma once
#include <unistd.h>
#include <opencv4/opencv2/opencv.hpp>
#include <arpa/inet.h>
#include <iostream>
#include <thread>
#include <string>
#include <queue>
#include <deque>
#include <vector>
#include <map>
#include "data.hpp"
#include "pnp.hpp"
#include "get_armor.hpp"
#include "kalman.hpp"
#include "pose.hpp"

#define MESSAGE_BUFFER_SIZE 10240

class Client
{
private:
    int client_fd;       //用于和服务器交流的fd
    sockaddr_in server_addr;
    int data_id = 0;
    
public:
    Client(const std::string& server_ip,int port);
    ~Client();
    void send_message(const TransformData& data);
    MessageBuffer receive_message();
    void send_message(const std::string& str);
    void send_message(const std::string& from,const std::string& to);
    void next_frame();
};