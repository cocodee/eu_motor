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
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // harmonic_setSendDataCallBack(sendCallback);
    // harmonic_setReceiveDataCallBack(receiveCallback);

    int id = 21;
    int autoReportMs = 100;
    for (int index = 0; index < 4; ++index)
    {
        harmonic_setTPDOCobId(devIndex, id, index, (0x80 << 24) + 0x180 + index * 0x100 + id); // 无效TPDO CobId(0x1800+index,0x01)
        harmonic_setTPDOMaxMappedCount(devIndex, id, index, 0);                                // 设置TPDO映射数量(0x1a00+index,0x00)
        harmonic_setTPDOTransmitType(devIndex, id, index, 0xFF);                               // 设置TPDO传输类型(0x1800+index,0x02)
        harmonic_setTPDOInhibitTime(devIndex, id, index, 0);                                   // 设置InhibitTime(0x1800+index,0x03)
        harmonic_setTPDOEventTimer(devIndex, id, index, autoReportMs);                         // 设置TPDO定时发送周期(0x1800+index,0x05)
        harmonic_setTPDOSYNCStartValue(devIndex, id, index, 0);                                // 设置同步帧起始值(0x1800+index,0x06)(最多只能发送8字节)
    }
    for (int index = 0; index < 4; ++index)
    {
        harmonic_setLocalRPDOCobId(index, (0x80 << 24) + 0x180 + index * 100 + id); // 无效RPDO Cobid(0x1400+index,0x01)
        harmonic_setLocalRPDOMaxMappedCount(index, 0);                              // 设置RPDO映射数量为0(0x1600+index,0x00)
        harmonic_setLocalRPDOTransmitType(index, 0xFF);                             // 设置RPDO传输类型(0x1400+index,0x02)
        harmonic_setLocalRPDOInhibitTime(index, 0);                                 // 设置InhibitTime(0x1400+index,0x03)
        harmonic_setLocalRPDOEventTimer(index, autoReportMs);                       // 设置RPDO定时接收周期(0x1400+index,0x05)
        harmonic_setLocalRPDOSYNCStartValue(index, 0);                              // 设置同步帧起始值(0x1400+index,0x06)
    }

    //    local
    harmonic_setLocalRPDOMapped(0, 0, (0x6064 << 16) + 0x20); // 实际位置4字节,设置第1个映射地址(0x1600,0x01)
    harmonic_setLocalRPDOMapped(0, 1, (0x606C << 16) + 0x20); // 实际速度4字节,设置第2个映射地址(0x1600,0x02)
    harmonic_setLocalRPDOMaxMappedCount(0, 2);                // 设置RPDO1映射数量(0x1600,0x00)

    harmonic_setLocalRPDOMapped(1, 0, (0x6077 << 16) + 0x10); // 实际力矩2字节设置第1个映射地址(0x1601,0x01)（1个PDO只能映射8个字节所以需要配置第2个RPDO）
    harmonic_setLocalRPDOMapped(1, 1, (0x6062 << 16) + 0x20); // 状态字2字节,设置第2映射地址(0x1601,0x02)
    harmonic_setLocalRPDOMapped(1, 2, (0x6041 << 16) + 0x10); // 状态字2字节,设置第3个映射地址(0x1601,0x03)
    harmonic_setLocalRPDOMaxMappedCount(1, 3);                // 设置RPDO2映射数量 (0x1601,0x00)

    harmonic_setLocalRPDOMapped(2, 0, (0x6079 << 16) + 0x20);   // 电压4字节设置第1个映射地址(0x1602,0x01)
    harmonic_setLocalRPDOMapped(2, 1, (0x2016 << 16) + 0x0108); // 温度字节,设置第2映射地址(0x1602,0x02)
    harmonic_setLocalRPDOMapped(2, 2, (0x603f << 16) + 0x10);   // 错误码2字节设置第3个映射地址(0x16012,0x03)
    harmonic_setLocalRPDOMaxMappedCount(2, 3);                  // 设置RPDO3映射数量 (0x1602,0x00)

    harmonic_setLocalRPDOMapped(3, 0, (0x6079 << 16) + 0x20);   // 电压4字节设置第2个映射地址(0x1603,0x01)
    harmonic_setLocalRPDOMapped(3, 1, (0x2016 << 16) + 0x0108); // 温度字节,设置第一个映射地址(0x1603,0x02)
    harmonic_setLocalRPDOMapped(3, 2, (0x603f << 16) + 0x10);   // 错误码2字节设置第3个映射地址(0x1603,0x03)
    harmonic_setLocalRPDOMaxMappedCount(3, 3);                  // 设置RPDO3映射数量 (0x1603,0x00)

    harmonic_setLocalRPDOCobId(0, 0x180 + id); // 有效RPDO1的cobid(0x1400,0x01)
    harmonic_setLocalRPDOCobId(1, 0x280 + id); // 有效RPDO2的cobid(0x1401,0x01)
    harmonic_setLocalRPDOCobId(2, 0x380 + id); // 有效RPDO3的cobid(0x1402,0x01)
    harmonic_setLocalRPDOCobId(3, 0x480 + id); // 有效RPDO4的cobid(0x1403,0x01)

    //    remote
    harmonic_setTPDOMapped(devIndex, id, 0, 0, (0x6064 << 16) + 0x20); // 设置第1个映射地址(0x1a00,0x01)
    harmonic_setTPDOMapped(devIndex, id, 0, 1, (0x606C << 16) + 0x20); // 设置第2个映射地址(0x1a00,0x02)
    harmonic_setTPDOMaxMappedCount(devIndex, id, 0, 2);                // 设置TPDO1映射数量(0x1a00,0x00)

    harmonic_setTPDOMapped(devIndex, id, 1, 0, (0x6077 << 16) + 0x10); // 设置第1个映射地址(0x1a01,0x01)（1个PDO只能映射8个字节所以需要配置第2个RPDO）
    harmonic_setTPDOMapped(devIndex, id, 1, 1, (0x6062 << 16) + 0x20); // 设置第2个映射地址(0x1a01,0x02)
    harmonic_setTPDOMapped(devIndex, id, 1, 2, (0x6041 << 16) + 0x10); // 设置第3个映射地址(0x1a01,0x03)
    harmonic_setTPDOMaxMappedCount(devIndex, id, 1, 3);                // 设置TPDO2映射数量(0x1a01,0x00)

    harmonic_setTPDOMapped(devIndex, id, 2, 0, (0x6079 << 16) + 0x20);   // 设置第1个映射地址(0x1a02,0x01)
    harmonic_setTPDOMapped(devIndex, id, 2, 1, (0x2016 << 16) + 0x0108); // 设置第2个映射地址(0x1a02,0x02)
    harmonic_setTPDOMapped(devIndex, id, 2, 2, (0x603f << 16) + 0x10);   // 设置第3个映射地址(0x1a02,0x03)
    harmonic_setTPDOMaxMappedCount(devIndex, id, 2, 3);                  // 设置TPDO3映射数量(0x1a02,0x00)

    harmonic_setTPDOMapped(devIndex, id, 3, 0, (0x6079 << 16) + 0x20);   // 设置第1个映射地址(0x1a03,0x02)
    harmonic_setTPDOMapped(devIndex, id, 3, 1, (0x2016 << 16) + 0x0108); // 设置第2个映射地址(0x1a03,0x01)
    harmonic_setTPDOMapped(devIndex, id, 3, 2, (0x603f << 16) + 0x10);   // 设置第3个映射地址(0x1a03,0x03)
    harmonic_setTPDOMaxMappedCount(devIndex, id, 3, 3);                  // 设置TPDO3映射数量(0x1a03,0x00)

    harmonic_setTPDOCobId(devIndex, id, 0, 0x180 + id); // 有效TPDO1的cobid(0x1800,0x01)
    harmonic_setTPDOCobId(devIndex, id, 1, 0x280 + id); // 有效TPDO2的cobid(0x1801,0x01)
    harmonic_setTPDOCobId(devIndex, id, 2, 0x380 + id); // 有效TPDO3的cobid(0x1802,0x01)
    harmonic_setTPDOCobId(devIndex, id, 3, 0x480 + id); // 有效TPDO4的cobid(0x1803,0x01)

    //    4、配置本地PDO数据，使其禁用(这部分可以去掉)
    for (int index = 0; index < 4; ++index)
    {
        harmonic_setLocalTPDOCobId(index, (0x80 << 24) + 0x200 + index * 100 + id);
        harmonic_setLocalTPDOMaxMappedCount(index, 0);
        harmonic_setLocalTPDOTransmitType(index, 0x00);
        harmonic_setLocalTPDOInhibitTime(index, 0);
        harmonic_setLocalTPDOEventTimer(index, 0);
        harmonic_setLocalTPDOSYNCStartValue(index, 0);
    }

    harmonic_setNodeState(devIndex, id, harmonic_NMTState_Reset_Node);
    harmonic_setNodeState(devIndex, id, harmonic_NMTState_Start_Node);
    harmonic_freeDLL(devIndex);
    return 0;
}