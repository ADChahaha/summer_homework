#include "server.hpp"

Server::Server(int port_) : port(port_)
{
    //socket,获得文件标识符
    if(!(server_fd = socket(AF_INET,SOCK_STREAM,0)))
    {
        std::cout << "create socket failed." << std::endl;
        exit(EXIT_FAILURE);
    }
    //ip地址和端口获取
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);
    //bind,绑定端口
    if(bind(server_fd,(struct sockaddr *)&address,sizeof(address)) < 0)
    {
        std::cout << "bind failed." << std::endl;
        exit(EXIT_FAILURE);
    }
}

Server::~Server()
{
    close(server_fd);
}

void Server::run()
{
    int addrlen = sizeof(address);
    //将服务器切换为监听模式
    if(listen(server_fd,5))
    {
        std::cout << "listen failed." << std::endl;
        exit(EXIT_FAILURE);
    }
    
    std::cout << "listening..." << std::endl;


    //开始监听
    while(true)
    {
        int client_socket;
        if((client_socket = accept(server_fd,(struct sockaddr *)&address,
        (socklen_t*)&addrlen)) < 0)
        {
            std::cout << "accpet failed." << std::endl;
            exit(EXIT_FAILURE);
        }
        //创建线程处理
        std::thread(handle_client,client_socket).detach();
    } 
}

void Server::handle_client(int client_socket)
{
    char buffer[BUFFER_SIZE];
    std::cout << "connected a client" << std::endl;
    
    //读取客户端发来的消息
    read(client_socket,buffer,BUFFER_SIZE);
    std::cout << "message from client:\n" << buffer << std::endl;
    
    //发送给客户端消息
    const char* hello = "Hello! I'm server.";
    write(client_socket,hello,sizeof(hello));
    std::cout << "send a message" << std::endl;
    close(client_socket);
}