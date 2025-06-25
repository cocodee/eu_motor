#include <iostream>
#include "../include/eu_harmonic.h"
#include <iomanip>
#include <mutex>
#include <thread>
#include <csignal>

// SDO方式读取从站数据示例程序

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
void sendCallback(const harmonic_CanMsg *msg)
{
    std::unique_lock<std::mutex> locker(CoutMutex);
    std::cout << "[0x" << std::hex << msg->cob_id << std::dec << "]";
    printHexArray(true, msg->data, msg->len);
}
void receiveCallback(const harmonic_CanMsg *msg)
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

void readMotorInfo(int devIndex, int id)
{
    while (IsRun)
    {
        if (HARMONIC_SUCCESS != harmonic_initDLL(harmonic_DeviceType_USB2CAN, devIndex, harmonic_Baudrate_1000))
        {
            std::cout << "[error]test open failed!" << std::endl;
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        harmonic_NodeState state;
        if (HARMONIC_SUCCESS != harmonic_getNodeState(devIndex, id, &state))
            std::cout << "[error]get node state failed!" << std::endl;
        else
            std::cout << "node state:" << state << std::endl;

        huint32 deviceType = -1;
        if (HARMONIC_SUCCESS != harmonic_getDeviceType(devIndex, id, &deviceType))
            std::cout << "[error]get device type failed!" << std::endl;
        else
            std::cout << "device type:" << deviceType << std::endl;

        huint32 vId = 0;
        if (HARMONIC_SUCCESS != harmonic_getVendorID(devIndex, id, &vId))
            std::cout << "[error]get vendor id failed!" << std::endl;
        else
            std::cout << "vendor ID:" << vId << std::endl;

        char devName[64];
        if (HARMONIC_SUCCESS != harmonic_getDeviceName(devIndex, id, devName))
            std::cout << "[error]get device name failed!" << std::endl;
        else
            std::cout << "device name:" << devName << std::endl;

        huint32 pId = 0;
        if (HARMONIC_SUCCESS != harmonic_getProductCode(devIndex, id, &pId))
            std::cout << "[error]get product code failed!" << std::endl;
        else
            std::cout << "product Code:" << pId << std::endl;

        huint32 serialNum = 0;
        if (HARMONIC_SUCCESS != harmonic_getSerialNumber(devIndex, id, &serialNum))
            std::cout << "[error]get serial number failed!" << std::endl;
        else
            std::cout << "serial number: " << serialNum << std::endl;

        char hVersion[64];
        if (HARMONIC_SUCCESS != harmonic_getHardwareVersion(devIndex, id, hVersion))
            std::cout << "[error]get hardware version failed!" << std::endl;
        else
            std::cout << "hardware Version:" << hVersion << std::endl;

        char sVersion[64];
        if (HARMONIC_SUCCESS != harmonic_getSoftwareVersion(devIndex, id, sVersion))
            std::cout << "[error]get software version failed!" << std::endl;
        else
            std::cout << "software Version:" << sVersion << std::endl;
        harmonic_freeDLL(devIndex);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

int main()
{
    signal(SIGINT, signal_handler);

    // harmonic_setSendDataCallBack(sendCallback);
    // harmonic_setReceiveDataCallBack(receiveCallback);

    th1 = std::thread(readMotorInfo, devIndex_0, 2);
    th2 = std::thread(readMotorInfo, devIndex_1, 1);
    while (1)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return 0;
}