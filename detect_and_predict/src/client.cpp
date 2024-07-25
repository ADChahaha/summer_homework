#include "client.hpp"


Client::Client(const std::string& server_ip,int port)
{
    //创建fd
    if((client_fd = socket(AF_INET,SOCK_STREAM,0)) < 0)
    {
        std::cout << "socket failed." << std::endl;
        exit(EXIT_FAILURE);
    }  

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    if(inet_pton(AF_INET,server_ip.c_str(),&server_addr.sin_addr) <= 0)
    {
        std::cout << "Invalid address / Address not supported" << std::endl;
        exit(EXIT_FAILURE);
    }

    //连接服务器
    if(connect(client_fd,(struct sockaddr *)&server_addr,
    sizeof(server_addr)) < 0)
    {
        std::cout << "Connection failed" << std::endl;;
        exit(EXIT_FAILURE);
    }
}

//todo 改成发送string
void Client::send_message(const TransformData& data)
{
    MessageBuffer message;
    message.Offset = 0;
    message.DataTotalLength = sizeof(data);
    message.DataID = data_id;
    message.DataLength = sizeof(data);
    message.MessageType = TRANSFORM;
    std::memcpy(message.Data, &data, sizeof(data));
    write(client_fd,&message,sizeof(message));
    std::cout << "sent a message." << std::endl;
}

void Client::send_message(const std::string& str)
{
    //假设str大小在一个缓冲区范围内
    MessageBuffer message;
    message.Offset = 0;
    std::memcpy(message.Data, str.c_str(), str.size() + 1);
    message.DataLength = str.size() + 1;
    message.DataTotalLength = message.DataLength;
    message.MessageType = STRING_MSG;
    message.DataID = data_id;
    write(client_fd,&message,sizeof(message));
}

void Client::next_frame()
{
    data_id++;
}

void Client::send_message(const std::string& from,const std::string& to)
{
    MessageBuffer message;
    message.Offset = 0;
    TransformRequestData data;
    memcpy(data.From,from.c_str(),from.size() + 1);
    memcpy(data.To,to.c_str(),to.size() + 1);
    memcpy(message.Data,&data,10218);
    message.DataLength = 10218;
    message.DataTotalLength = message.DataLength;
    message.MessageType = TRANSFORM_REQUEST;
    message.DataID = data_id;
    write(client_fd,&message,sizeof(message));
}

MessageBuffer Client::receive_message()
{
    MessageHead head;
    uchar buffer[MESSAGE_BUFFER_SIZE];
    int head_length = sizeof(head);
    MessageBuffer message;
    
    //接受处理消息
    //解决tcp拆包 
    //读取数据头，查看长度
    read(client_fd,&buffer,head_length);
    memcpy(&head,buffer,head_length);
    if(head.Start != 0X0D00)
    {
        std::cout << "start error!" << std::endl;
        exit(EXIT_FAILURE);
    }
    size_t data_and_end_size = sizeof(MessageBuffer) - sizeof(MessageHead);
    size_t cur_size = 0;
    //循环读取，直到读到整个数据段
    while(true)
    {
        if(cur_size == data_and_end_size)
            break;
        cur_size += read(client_fd,buffer + head_length + cur_size,
        data_and_end_size - cur_size);
        if(cur_size < 0)
        {
            std::cout << "server disconneted." << std::endl;
            exit(EXIT_FAILURE);
        }
    }
    memcpy(&message,buffer,head_length + data_and_end_size);
    if(message.End != 0X0721)
    {
        std::cout << std::hex << message.End << std::dec << std::endl;
        std::cout << "end error!" << std::endl;
        exit(EXIT_FAILURE);
    }
    return message;
    


}
Client::~Client()
{
    close(client_fd);
}

