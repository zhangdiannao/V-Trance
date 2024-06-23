#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "opencv2/opencv.hpp"
#include "MvCameraControl.h"
#include <vector>
#include <time.h>

using namespace cv;
using namespace std;

#define GUI_FLAG 0

typedef struct image_data
{
    uint8_t *p;
    mutex lock;
} image_data;

/*全局变量*/
void *handle;            // 摄像头设备句柄
int fd;                  // 套接字文件描述符
struct sockaddr_in addr; // 服务器地址
int STOP_FLAG = 0;       // 停止运动
image_data image_data_buffer[5];
uint8_t image_data_buffer_head = 0;
uint8_t image_data_buffer_tail = 0;
condition_variable ADD_FLAG;
condition_variable REDUCE_FLAG;
clock_t BEGIN, END;
uint8_t time_flag = 0;
/*全局变量结束*/

void motor_stop()
{
    uint8_t command[17] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x0B, 0x01, 0x10, 0x00, 0x64, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00};
    write(fd, command, 17);
}

void motor_move_right()
{
    uint8_t command[17] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x0B, 0x01, 0x10, 0x00, 0x64, 0x00, 0x02, 0x04, 0x03, 0xE8, 0x00, 0x00};
    write(fd, command, 17);
}
void motor_move_left()
{
    uint8_t command[17] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x0B, 0x01, 0x10, 0x00, 0x64, 0x00, 0x02, 0x04, 0xFC, 0x18, 0xFF, 0xFF};
    write(fd, command, 17);
}

Point trance_v(Mat &frame, double &k, double &b, double &distance)
{

    vector<Point> brightness_max_point_array;
    clock_t begin, end;
    begin = clock();
    // 遍历frame的每一列
    for (int x = 0; x < frame.cols; x += 2)
    {
        uint8_t brightness_max = 0;
        Point point_brightness_max(0, 0);
        for (int y = 0; y < frame.rows; y += 2)
        {
            uint8_t current_point_brightness = frame.at<uchar>(y, x);
            if (current_point_brightness)
            {
                if (current_point_brightness > brightness_max)
                {
                    brightness_max = current_point_brightness;
                    point_brightness_max.x = x;
                    point_brightness_max.y = y;
                }
            }
        }
        brightness_max_point_array.push_back(Point(point_brightness_max.x, point_brightness_max.y));
    }
    end = clock();
    // printf("time use:%lf\n", (double)(end - begin) / CLOCKS_PER_SEC * 1000);
#if 0
    for (int i = 0; i < brightness_max_point_array.size(); i++)
    {
        printf("%d,%d\n", brightness_max_point_array.at(i).x, brightness_max_point_array.at(i).y);
    }
#endif

    // 拟合直线
    Vec4f line_param;
    fitLine(brightness_max_point_array, line_param, DIST_L2, 0, 0.01, 0.01);
    k = line_param[1] / line_param[0];
    b = line_param[3] - k * line_param[2];
    // 找到离直线最远的那个点
    double max_distence = 0;
    Point target(0, 0);
    for (int i = 0; i < brightness_max_point_array.size(); i++)
    {
        int x = brightness_max_point_array.at(i).x;
        int y = brightness_max_point_array.at(i).y;
        double current_distence = abs((k * x - y + b) / sqrt(k * k + 1));
        if (current_distence > max_distence)
        {
            max_distence = current_distence;
            target.x = x;
            target.y = y;
        }
    }
    distance = max_distence;

    return target;
}

bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }

    return true;
}

void thread_camera_work()
{
    int res;
#if GUI_FLAG
    namedWindow("frame", WINDOW_FREERATIO);
    namedWindow("frame_show", WINDOW_FREERATIO);
#endif
    while (1)
    {
        // 获取摄像头句柄
        MV_CC_DEVICE_INFO_LIST stDeviceList = {0};
        res = MV_CC_EnumDevices(MV_GIGE_DEVICE, &stDeviceList);
        if (res != MV_OK)
        {
            printf("[Cmaera Thread]Enum devices failed!\n");
            continue;
        }
        if (stDeviceList.nDeviceNum == 0)
        {
            printf("[Camera Thread]No device online!\n");
            this_thread::sleep_for(chrono::seconds(5));
            continue;
        }
        // 打印设备信息
        printf("[Camera Thread]Find %d devices:\n", stDeviceList.nDeviceNum);
        MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[0];
        PrintDeviceInfo(pDeviceInfo);
        // 创建摄像头句柄
        res = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
        if (res != MV_OK)
        {
            printf("[Camera Thread]Create device handle failed!\n");
            continue;
        }
        // 打开设备
        res = MV_CC_OpenDevice(handle);
        if (res != MV_OK)
        {
            printf("[Camera Thread]Open device failed!\n");
            continue;
        }
        // 探测网络最佳包大小(只对GigE相机有效)
        if (stDeviceList.pDeviceInfo[0]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
            if (nPacketSize > 0)
            {
                res = MV_CC_SetIntValue(handle, "GevSCPSPacketSize", nPacketSize);
                if (res != MV_OK)
                {
                    printf("Warning: Set Packet Size fail!\n");
                }
            }
            else
            {
                printf("Warning: Get Packet Size fail!\n");
            }
        }
        // 设置触发模式为off
        res = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if (MV_OK != res)
        {
            printf("[Camera Thread]Set trigger mode fail!\n");
            continue;
        }
        // 设置图像格式
        res = MV_CC_SetEnumValue(handle, "PixelFormat", 0x01080001);
        if (MV_OK != res)
        {
            printf("[Camera Thread]Set pixel fromat fail!\n");
            continue;
        }
        // 开始取流
        res = MV_CC_StartGrabbing(handle);
        if (res != MV_OK)
        {
            printf("[Camera Thread]Start grabbing failed!\n");
        }
        // 获取数据包大小
        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        res = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
        if (MV_OK != res)
        {
            printf("[Camera Thread]Get PayloadSize fail!\n");
            this_thread::sleep_for(chrono::seconds(5)); // 5秒后重试
            continue;
        }
        MV_FRAME_OUT_INFO_EX stImageInfo = {0};
        memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
        unsigned char *pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
        if (NULL == pData)
        {
            this_thread::sleep_for(chrono::seconds(5)); // 5秒后重试
            continue;
        }
        unsigned int nDataSize = stParam.nCurValue;
        while (1)
        {
            res = MV_CC_GetOneFrameTimeout(handle, pData, nDataSize, &stImageInfo, 1000);
            if (res == MV_OK)
            {
                // 获取到Mat数据
                Mat frame(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC1, pData);
                // 往当前尾巴位置写入数据
                unique_lock<mutex> lk(image_data_buffer[image_data_buffer_tail].lock);
                // 判断还能不能写入
                while ((image_data_buffer_tail + 1) % 5 == image_data_buffer_head)
                {
                    printf("[Warning]缓冲区已满,无法写入!\n");
                    REDUCE_FLAG.notify_all();
                    ADD_FLAG.wait(lk);
                }
                printf("向%d位置写入图像...\n", image_data_buffer_tail);
                for (int x = 0; x < 720; x++)
                {
                    for (int y = 0; y < 1080; y++)
                    {
                        image_data_buffer[image_data_buffer_tail].p[x * 1080 + y] = frame.at<uchar>(y, x);
                    }
                }
                image_data_buffer_tail += 1;
                image_data_buffer_tail %= 5;
                lk.unlock();
            }
            else
            {
                printf("[Camera Thread]No data!\n");
                break;
            }
        }
        if (pData != NULL)
        {
            free(pData);
            pData = NULL;
        }
        // 关闭设备
        res = MV_CC_CloseDevice(handle);
        if (MV_OK != res)
        {
            printf("[Camera Thread]Close device fail!\n");
            break;
        }

        // 销毁句柄
        res = MV_CC_DestroyHandle(handle);
        if (MV_OK != res)
        {
            printf("[Camera Thread]Destroy handle fail!\n");
            break;
        }
        handle = NULL;
    }
}

void thread_socket_work()
{
    while (1)
    {
        // 创建通信的套接字
        fd = socket(AF_INET, SOCK_STREAM, 0);
        if (fd == -1)
        {
            printf("[Error]fd:%d\n", fd);
            this_thread::sleep_for(chrono::seconds(5));
            continue;
        }
        // 配置服务器地址
        addr.sin_family = AF_INET;
        addr.sin_port = htons(502); // 大端端口
        inet_pton(AF_INET, "192.168.2.10", &addr.sin_addr.s_addr);
        int res = connect(fd, (struct sockaddr *)&addr, sizeof(addr));
        if (res == -1)
        {
            perror("[Error]connect");
            continue;
        }
        printf("[Success]Connected to server!\n");
        while (1)
        {
            uint8_t buf[1024];
            // 接收数据
            memset(buf, 0, sizeof(buf));
            int len = read(fd, buf, sizeof(buf));
            if (len > 0)
            {
#if 0
                printf("[Server]:");
                for (int i = 0; i < len; i++)
                {
                    printf("%02X ", buf[i]);
                }
                printf("\n");
#endif
            }
            else if (len == 0)
            {
                printf("[Socket Thread]Server closed!\n");
                break;
            }
            else
            {
                perror("[error]read");
                break;
            }
        }
        close(fd);
    }
}

void image_handler()
{
    while (1)
    {
        // 从头部取出数据
        unique_lock<mutex> lk(image_data_buffer[image_data_buffer_head].lock);
        // 判断还能不能取出
        while (image_data_buffer_head == image_data_buffer_tail)
        {
            // printf("[Warning]缓冲区为空!\n");
            ADD_FLAG.notify_all();
            REDUCE_FLAG.wait(lk);
        }
        printf("处理%d位置的图像...\n", image_data_buffer_head);
        uint16_t point_array[720][2];
        for (int x = 0; x < 720; x++)
        {
            uint8_t brightness_max = 0;
            uint16_t point_brightnexx_max_x = 0;
            uint16_t point_brightnexx_max_y = 0;
            for (int y = 0; y < 1080; y++)
            {
                if (image_data_buffer[image_data_buffer_head].p[x * 1080 + y] > brightness_max)
                {
                    brightness_max = image_data_buffer[image_data_buffer_head].p[x * 1080 + y];
                    point_brightnexx_max_x = x;
                    point_brightnexx_max_y = y;
                }
                point_array[x][0] = point_brightnexx_max_x;
                point_array[x][1] = point_brightnexx_max_y;
            }
        }
        if (time_flag == 0)
        {
            time_flag = 1;
            BEGIN = clock();
        }
        else if (time_flag == 1)
        {
            time_flag = 0;
            END = clock();
            printf("Time used:%.1lf\n", (double)(END - BEGIN) / CLOCKS_PER_SEC * 1000);
        }
        image_data_buffer_head += 1;
        image_data_buffer_head %= 5;
        lk.unlock();
    }
}

int main()
{
    // 初始化缓冲区
    for (int i = 0; i < 5; i++)
    {
        image_data_buffer[i].p = (uint8_t *)malloc(777600);
        if (image_data_buffer[i].p == NULL)
        {
            printf("[Error]Malloc failed!\n");
            return 1;
        }
    }
    printf("[Success]Malloc ok!\n");
    // 创建线程
    thread thread_camera(thread_camera_work);
    thread thread_socket(thread_socket_work);
    thread thread_reduce(image_handler);
    // 线程分离
    thread_camera.detach();
    thread_socket.detach();
    thread_reduce.detach();
    // 用户控制
    while (1)
    {
        char buffer[1024];
        scanf("%s", buffer);
        if (strcmp(buffer, "s") == 0)
        {
            printf("[User]Stop!\n");
            motor_stop();
            STOP_FLAG = 1;
        }
        else if (strcmp(buffer, "l") == 0)
        {
            printf("[User]Left!\n");
            motor_move_left();
            STOP_FLAG = 0;
        }
        else if (strcmp(buffer, "r") == 0)
        {
            printf("[User]Right!\n");
            motor_move_right();
        }
    }
    return 0;
}