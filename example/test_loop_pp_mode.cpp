#include <iostream>
#include "../include/eu_harmonic.h"
#include <thread>
#include <iomanip>
#include <csignal>
#include <mutex>

// 轮廓位置模式控制示例程序

void printHexArray(bool isSend, const unsigned char *data, size_t length)
{
    if (isSend)
        std::cout << "send:";
    else
        std::cout << "receive:";
    for (size_t i = 0; i < length; ++i)
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(data[i]) << " ";
    std::cout << std::endl;
}

std::mutex CoutMutex;
void sendCallback(int devIndex, const harmonic_CanMsg *msg)
{
    std::unique_lock<std::mutex> locker(CoutMutex);
    std::cout << "[0x" << std::hex << msg->cob_id << std::dec << "]";
    printHexArray(true, msg->data, msg->len);
}
void receiveCallback(int devIndex, const harmonic_CanMsg *msg)
{
    std::unique_lock<std::mutex> locker(CoutMutex);
    std::cout << "[0x" << std::hex << msg->cob_id << std::dec << "]";
    printHexArray(false, msg->data, msg->len);
}


int devIndex = 0;

void signal_handler(int signal)
{
    harmonic_freeDLL(devIndex);
    exit(1);
}

int main()
{
    signal(SIGINT, signal_handler);
    if (HARMONIC_SUCCESS != harmonic_initDLL(harmonic_DeviceType_Canable, devIndex, harmonic_Baudrate_1000))
    {
        std::cout << "[error]test open failed!" << std::endl;
        harmonic_freeDLL(devIndex);
        return -1;
    }

    // harmonic_setSendDataCallBack(sendCallback);
    // harmonic_setReceiveDataCallBack(receiveCallback);

    huint8 id = 1;
    huint32 vel = 2000000;
    huint32 acc = vel;
    huint32 dec = acc;
    bool isRelative = false;   // 是否是相对位置
    bool isImmediately = true; // 是否立即生效
    bool isUpdate = false;     // 是否采用更新位置模式
    hint32 startPos = 0;
    int step = 1000; // 轨迹点步长
    int itpv = 10;   // 插补周期，单位ms

    if (HARMONIC_SUCCESS != harmonic_getActualPos(devIndex, id, &startPos))
    {
        std::cout << "[test]getActualPos failed !" << std::endl;
        harmonic_freeDLL(devIndex);
        return -1;
    }

    std::thread th([=]()
                   {
     hint32 pos = startPos;
 while (1)
    {
        if (HARMONIC_SUCCESS != harmonic_profilePositionControl(devIndex, id, pos, vel, acc, dec, isRelative, isImmediately, isUpdate))
        {
            std::cout << "[test]test pp control failed!" << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(itpv));//!需根据平台选择高精度定时器以获得更好的控制效果
        pos += step;
    } });
    th.join();
    harmonic_freeDLL(devIndex);
    return 0;
}
