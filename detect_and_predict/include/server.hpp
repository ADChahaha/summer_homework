#pragma once
#include <unistd.h>
#include <arpa/inet.h>
#include <iostream>
#include <thread>
#define BUFFER_SIZE 10240


class Server
{
private:
    int server_fd;
    int port; 
    sockaddr_in address;
    void handle_client(int client_socket);
public:
    Server(int port_);
    ~Server();
    void run();
    
};