#include <iostream>
#include "../include/eu_harmonic.h"
#include <iomanip>
#include <thread>
#include <csignal>
#include <mutex>

// 内插位置模式控制示例程序

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
int devIndex_0 = 0;
int devIndex_1 = 1;
std::thread th1;
std::thread th2;
bool IsRun = true;

void signal_handler(int signal)
{
    IsRun = false;
    if (th1.joinable())
        th1.join();
    if (th2.joinable())
        th2.join();
    harmonic_freeDLL(devIndex_0);
    harmonic_freeDLL(devIndex_1);
    exit(1);
}

void ppModeControl(int devIndex, int id)
{
    hint32 pos = 1000000;
    while (IsRun)
    {
        std::cout << "[" + std::to_string(id) + "]moving to " + std::to_string(pos) << std::endl;
        if (HARMONIC_SUCCESS != harmonic_profilePositionControl(devIndex, id, pos, 1000000, 1000000, 1000000,true))
        {
            std::cout << "[error]Failed to control! id:" << std::to_string(id) << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(3000)); //! 需根据平台选择高精度定时器以获得更好的控制效果
        pos = -pos;
    };
}

int main()
{
    signal(SIGINT, signal_handler);

    if (HARMONIC_SUCCESS != harmonic_initDLL(harmonic_DeviceType_Canable, devIndex_0, harmonic_Baudrate_1000))
    {
        std::cout << "[error]test open failed!" << std::endl;
        return -1;
    }
    if (HARMONIC_SUCCESS != harmonic_initDLL(harmonic_DeviceType_Canable, devIndex_1, harmonic_Baudrate_1000))
    {
        std::cout << "[error]test open failed!" << std::endl;
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // harmonic_setSendDataCallBack(sendCallback);
    // harmonic_setReceiveDataCallBack(receiveCallback);

    th1 = std::thread(ppModeControl, devIndex_0, 1);
    th2 = std::thread(ppModeControl, devIndex_1, 2);
    while (1)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return 0;
}