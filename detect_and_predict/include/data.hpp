#pragma once

typedef struct 
{
    //head
    unsigned short Start;
    unsigned short MessageType;
    unsigned int DataID;
    unsigned int DataTotalLength;
    unsigned int Offset;
    unsigned int DataLength;
}MessageHead;

typedef struct 
{
    //head
    unsigned short Start = 0X0D00;
    unsigned short MessageType;
    unsigned int DataID;
    unsigned int DataTotalLength;
    unsigned int Offset;
    unsigned int DataLength;
    //data  
    unsigned char Data[10218];
    //end
    unsigned short End = 0X0721;
}MessageBuffer;

enum MessageType
{
    STRING_MSG = 0X0000,
    IMAGE_MSG = 0X1145,
    CAMERA_INFO = 0X1419,
    TRANSFORM = 0X1981,
    TRANSFORM_REQUEST = 0X1982
};

typedef struct 
{
    double CameraMatrix[9];
    double DistortionCoefficients[5];
}CameraInfoData;

typedef struct 
{
    double Translation[3];
    double Rotation[4];
}TransformData;

typedef struct 
{
    char From[10218 / 2];
    char To[10218 / 2];
}TransformRequestData;
