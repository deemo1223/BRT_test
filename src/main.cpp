#include <iostream>
#include <iomanip>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <cmath>

bool set_up_can_interface(const std::string& interface_name, int bitrate){
    // check input bit rate
    if (bitrate <= 0){
        std::cerr << "Invalid bit rate: "<< bitrate << std::endl;
        return false;
    }

    // send system command to first set down CAN interface
    std::string down_cmd = 
        "ip link set " + interface_name + " down";

    std::system(down_cmd.c_str());

    // send system command to set up CAN interface
    std::string up_cmd = 
        "ip link set " + interface_name + 
        " up type can bitrate " + std::to_string(bitrate) + 
        " restart-ms 100";
    
    int ret = std::system(up_cmd.c_str());
    if (ret != 0){
        std::cerr << "Failed to bring up "<< interface_name << ".\n"
                  << "Please run this program with sudo."
                  << std::endl;
        return false;
    }

    std::cout << interface_name << " is up at " << std::to_string(bitrate) << " bps" << std::endl;
    
    return true;
}


int main()
{
    const char* can_interface = "can0";
    int bitrate = 500000;

    if (!set_up_can_interface(can_interface, bitrate)){
        return 1;
    }

    const uint8_t device_id = 0x01;
    const double resolution = 4096.0;
    const double wheel_circumference_mm = 60.0;

    uint32_t zero_count = 0;

    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0)
    {
        perror("socket");
        return 1;
    }

    // 可选：关闭本机发送回环，避免读到自己发出的帧
    int loopback = 0;
    setsockopt(sock, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    // 设置 read 超时，避免一直卡住
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100 ms
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, can_interface);

    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("ioctl");
        close(sock);
        return 1;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        perror("bind");
        close(sock);
        return 1;
    }

    std::cout << "CAN socket opened on " << can_interface << std::endl;

    while (true)
    {
        struct can_frame tx_frame;
        std::memset(&tx_frame, 0, sizeof(tx_frame));

        // BRT 手册读取编码器值：
        // CAN ID: 0x01
        // Data: 04 01 01 00
        tx_frame.can_id = device_id;
        tx_frame.can_dlc = 4;
        tx_frame.data[0] = 0x04;
        tx_frame.data[1] = device_id;
        tx_frame.data[2] = 0x01;
        tx_frame.data[3] = 0x00;

        int nbytes = write(sock, &tx_frame, sizeof(tx_frame));
        if (nbytes != sizeof(tx_frame))
        {
            perror("write");
            usleep(100000);
            continue;
        }

        std::cout << "TX: ID=0x"
                  << std::hex << static_cast<int>(tx_frame.can_id)
                  << " Data=04 01 01 00"
                  << std::dec << std::endl;

        bool got_response = false;

        // 尝试在短时间内读取返回帧
        for (int attempt = 0; attempt < 5; ++attempt)
        {
            struct can_frame rx_frame;
            std::memset(&rx_frame, 0, sizeof(rx_frame));

            nbytes = read(sock, &rx_frame, sizeof(rx_frame));

            if (nbytes < 0)
            {
                // 超时或者无数据
                continue;
            }

            std::cout << "RX: ID=0x"
                      << std::hex << rx_frame.can_id
                      << " Data=";

            for (int i = 0; i < rx_frame.can_dlc; ++i)
            {
                std::cout << std::setw(2) << std::setfill('0')
                          << static_cast<int>(rx_frame.data[i]) << " ";
            }

            std::cout << std::dec << std::endl;

            // 只处理真正的传感器返回：
            // Data: 07 01 01 b0 b1 b2 b3
            if (rx_frame.can_id == device_id &&
                rx_frame.can_dlc >= 7 &&
                rx_frame.data[0] == 0x07 &&
                rx_frame.data[1] == device_id &&
                rx_frame.data[2] == 0x01)
            {
                uint32_t count =
                    static_cast<uint32_t>(rx_frame.data[3]) |
                    (static_cast<uint32_t>(rx_frame.data[4]) << 8) |
                    (static_cast<uint32_t>(rx_frame.data[5]) << 16) |
                    (static_cast<uint32_t>(rx_frame.data[6]) << 24);
                
                // calculate the length to the nearest 0.1mm
                double length_mm =
                    (static_cast<double>(count))
                    * wheel_circumference_mm
                    / resolution;
                
                length_mm = std::round(length_mm * 10.0) / 10.0;

                std::cout << "Raw count: " << count
                          << " | Length: " << length_mm << " mm"
                          << std::endl;

                got_response = true;
                break;
            }
        }

        if (!got_response)
        {
            std::cout << "No valid sensor response." << std::endl;
        }

        usleep(100000);  // 100 ms
    }

    close(sock);
    return 0;
}

