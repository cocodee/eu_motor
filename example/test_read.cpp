#include <iostream>
#include "../include/eu_harmonic.h"
#include <iomanip>
#include <mutex>
#include <thread>
#include <csignal>
#include <string>      // 用于 std::stoi
#include <stdexcept>   // 用于捕获 std::stoi 的异常
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

int devIndex = 1;

void signal_handler(int signal)
{
    harmonic_freeDLL(devIndex);
    exit(1);
}

int main(int argc, char **argv)
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

    // --- 1. 参数检查 ---
    // 检查是否提供了足够的参数。程序名本身是第一个参数，所以我们需要 argc == 2。
    if (argc != 2)
    {
        // 如果参数不正确，打印用法提示并退出
        std::cerr << "Usage: " << argv[0] << " <node_id>" << std::endl;
        std::cerr << "Example: " << argv[0] << " 21" << std::endl;
        return 1; // 返回非零值表示错误
    }

    int id;
    // --- 2. 解析参数 ---
    try
    {
        // 使用 std::stoi 将字符串参数 (argv[1]) 转换为整数
        id = std::stoi(argv[1]);
    }
    catch (const std::invalid_argument& e)
    {
        std::cerr << "[Error] Invalid node ID. Please provide a valid number." << std::endl;
        return 1;
    }
    catch (const std::out_of_range& e)
    {
        std::cerr << "[Error] Node ID is out of range for an integer." << std::endl;
        return 1;
    }

    std::cout << "Attempting to query node with ID: " << id << std::endl;

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
    return 0;
}