#include <iostream>
#include "../include/eu_harmonic.h"
#include <thread>
#include <iomanip>
#include <csignal>
#include <mutex>

// 轮廓速度模式控制示例程序

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
    huint32 acc = 100000;
    huint32 dec = 100000;
    hint32 targetVel = 100000;
    bool isUpdate = false; // 是否采用更新位置模式

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    while (1)
    {
        if (HARMONIC_SUCCESS != harmonic_profileVelocityControl(devIndex, id, targetVel, acc, dec, isUpdate))
        {
            std::cout << "[test]test pv control failed!" << std::endl;
            break;
        }
        std::cout << std::dec << "[test]running at a speed of " << targetVel << "..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        targetVel = -targetVel;
    }
    harmonic_freeDLL(devIndex);
    return 0;
}