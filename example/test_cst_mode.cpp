#include <iostream>
#include "../include/eu_harmonic.h"
#include <iomanip>
#include <thread>
#include <csignal>
#include <mutex>

// 同步力矩模式控制示例程序

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

huint8 devIndex = 0;

void signal_handler(int signal)
{
    harmonic_freeDLL(devIndex);
    exit(1);
}

void setTor(int id, int tor, bool isSync)
{
    huint8 data[4];
    data[0] = (unsigned)tor & 0x000000ff;
    data[1] = ((unsigned)tor & 0x0000ff00) >> 8;
    data[2] = ((unsigned)tor & 0x00ff0000) >> 16;
    data[3] = ((unsigned)tor & 0xff000000) >> 24;
    harmonic_writeCanData(devIndex, 0x200 + id, data, 4);
    if (isSync)
    {
        huint8 data[1] = {0};
        harmonic_writeCanData(devIndex, 0x80, data, 1);
    }
}

int main()
{
    signal(SIGINT, signal_handler);

    if (HARMONIC_SUCCESS != harmonic_initDLL(harmonic_DeviceType_Canable, devIndex, harmonic_Baudrate_500))
    {
        std::cout << "[error]test open failed!" << std::endl;
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // harmonic_setSendDataCallBack(sendCallback);
    // harmonic_setReceiveDataCallBack(receiveCallback);

    huint8 id = 1;
    bool isSync = true; // 是否采用同步方式
    huint8 type = isSync ? 0x01 : 0xFF;
    hint16 startTor = 0; // 存放规划起始位置值
    int itpv = 4;        // 插补周期，单位ms
    int step = 1;        // 轨迹点步长

    if (HARMONIC_SUCCESS != harmonic_setOperateMode(devIndex, id, harmonic_OperateMode_CyclicSyncTorque))
    {
        std::cout << "[test]setOperateMode failed !" << std::endl;
        harmonic_freeDLL(devIndex);
        return -1;
    }
    if (HARMONIC_SUCCESS != harmonic_setInterpolationTimePeriodValue(devIndex, id, itpv))
    {
        std::cout << "[test]setInterpolationTimePeriodValue failed !" << std::endl;
        harmonic_freeDLL(devIndex);
        return -1;
    }
    if (HARMONIC_SUCCESS != harmonic_setSyncCounter(devIndex, id, 0))
    {
        std::cout << "[test]setSyncCounter failed !" << std::endl;
        harmonic_freeDLL(devIndex);
        return -1;
    }
    if (HARMONIC_SUCCESS != harmonic_setRPDOCobId(devIndex, id, 0, (0x80 << 24) + 0x200 + id))
    {
        std::cout << "[test]setRPDOCobId failed !" << std::endl;
        harmonic_freeDLL(devIndex);
        return -1;
    }
    if (HARMONIC_SUCCESS != harmonic_setRPDOMaxMappedCount(devIndex, id, 0, 0))
    {
        std::cout << "[test]setRPDOMaxMappedCount failed !" << std::endl;
        harmonic_freeDLL(devIndex);
        return -1;
    }
    if (HARMONIC_SUCCESS != harmonic_setRPDOTransmitType(devIndex, id, 0, type))
    {
        std::cout << "[test]setRPDOTransmitType failed !" << std::endl;
        harmonic_freeDLL(devIndex);
        return -1;
    }
    if (HARMONIC_SUCCESS != harmonic_setRPDOMapped(devIndex, id, 0, 0, (0x6071 << 16) + 0x10))
    {
        std::cout << "[test]setRPDOMapped failed !" << std::endl;
        harmonic_freeDLL(devIndex);
        return -1;
    }
    if (HARMONIC_SUCCESS != harmonic_setRPDOMaxMappedCount(devIndex, id, 0, 1))
    {
        std::cout << "[test]setRPDOMaxMappedCount failed !" << std::endl;
        harmonic_freeDLL(devIndex);
        return -1;
    }
    if (HARMONIC_SUCCESS != harmonic_setNodeState(devIndex, id, harmonic_NMTState_Reset_Node))
    {
        std::cout << "[test]setNodeState failed !" << std::endl;
        harmonic_freeDLL(devIndex);
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (HARMONIC_SUCCESS != harmonic_setNodeState(devIndex, id, harmonic_NMTState_Start_Node))
    {
        std::cout << "[test]setNodeState failed !" << std::endl;
        harmonic_freeDLL(devIndex);
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (HARMONIC_SUCCESS != harmonic_setRPDOCobId(devIndex, id, 0, 0x200 + id))
    {
        std::cout << "[test]setRPDOCobId failed !" << std::endl;
        harmonic_freeDLL(devIndex);
        return -1;
    }
    if (HARMONIC_SUCCESS != harmonic_setControlword(devIndex, id, 0x06))
    {
        std::cout << "[test]setControlword failed !" << std::endl;
        harmonic_freeDLL(devIndex);
        return -1;
    }
    if (HARMONIC_SUCCESS != harmonic_setControlword(devIndex, id, 0x07))
    {
        std::cout << "[test]setControlword failed !" << std::endl;
        harmonic_freeDLL(devIndex);
        return -1;
    }
    if (HARMONIC_SUCCESS != harmonic_setControlword(devIndex, id, 0x0F))
    {
        std::cout << "[test]setControlword failed !" << std::endl;
        harmonic_freeDLL(devIndex);
        return -1;
    }
    if (HARMONIC_SUCCESS != harmonic_getActualTorque(devIndex, id, &startTor))
    {
        std::cout << "[test]harmonic_getActualTorque failed !" << std::endl;
        harmonic_freeDLL(devIndex);
        return -1;
    }
    hint16 tor = startTor;
    while (1)
    {
        setTor(id, tor, isSync);
        std::cout << std::dec << tor << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(itpv)); //! 需根据平台选择高精度定时器以获得更好的控制效果
        tor += step;
    };
    return 0;
}