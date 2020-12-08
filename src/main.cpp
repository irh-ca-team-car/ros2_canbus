#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <sys/time.h>
#include <fcntl.h>
#include <string>
#include <map>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <signal.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <poll.h>
#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#define UNINITIALIZED_SOCKET (-1)
#define CAN_MESSAGE_TIMEOUT (100)
using namespace rclcpp;
int channel = 0;
Node::SharedPtr n;
std::string out_topic;
int init_can_socket(const char *can_channel,
                    struct timeval *tv);
int sock = UNINITIALIZED_SOCKET;
#define SYSTEM(A, ...)                \
    {                                 \
        char m[1000];                 \
        sprintf(m, A, ##__VA_ARGS__); \
        system(m);                    \
    }
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    n = std::make_shared<rclcpp::Node>("ros2_canbus_reader");

    channel = n->declare_parameter<int>("channel", 0);
    out_topic = n->declare_parameter<std::string>("out_topic", "canbus");

    SYSTEM("ifconfig can%d down\n", channel);
    SYSTEM("ip link set can%d type can bitrate 500000\n", channel);
    SYSTEM("ip link set up can%d\n", channel);
    SYSTEM("ifconfig can%d up\n", channel);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = CAN_MESSAGE_TIMEOUT;

    char can_string_buffer[16];
    snprintf(can_string_buffer, 16, "can%u", channel);

    sock = init_can_socket(can_string_buffer, &timeout);

    auto pub = n->create_publisher<can_msgs::msg::Frame>(out_topic, 100);
    while (rclcpp::ok())
    {
        struct can_frame rx_frame;
        memset(&rx_frame, 0, sizeof(rx_frame));
        int recv_bytes = 0;

        recv_bytes = read(sock, &rx_frame, sizeof(rx_frame));
        if (recv_bytes > 0)
        {
            can_msgs::msg::Frame f;
            for (int i = 0; i < 8; i++)
                f.data[i] = rx_frame.data[i];
            f.id = rx_frame.can_id;
            f.dlc = rx_frame.can_dlc;
            pub->publish(f);
        }
    }
    close(sock);

    return 0;
}

int init_can_socket(const char *can_channel,
                    struct timeval *tv)
{
    if (can_channel == NULL)
    {
        return UNINITIALIZED_SOCKET;
    }

    int valid = UNINITIALIZED_SOCKET;
    int sock = UNINITIALIZED_SOCKET;

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));

    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (sock < 0)
    {
        perror("Opening CAN socket failed:");
        exit(-1);
    }
    else
    {
        strncpy(ifr.ifr_name, can_channel, IFNAMSIZ);

        valid = ioctl(sock, SIOCGIFINDEX, &ifr);

        if (valid < 0)
        {
            perror("Finding CAN index failed:");
            exit(-1);
        }
    }

    //If a timeout has been specified set one here since it should be set before
    //the bind call
    if (valid >= 0 && tv != NULL)
    {
        valid = setsockopt(sock,
                           SOL_SOCKET,
                           SO_RCVTIMEO,
                           tv,
                           sizeof(struct timeval));

        if (valid < 0)
        {
            perror("Setting timeout failed:");
            exit(-1);
        }
    }

    if (valid >= 0)
    {
        struct sockaddr_can can_address;

        memset(&can_address, 0, sizeof(can_address));
        can_address.can_family = AF_CAN;
        can_address.can_ifindex = ifr.ifr_ifindex;

        valid = bind(sock,
                     (struct sockaddr *)&can_address,
                     sizeof(can_address));

        if (valid < 0)
        {
            perror("Socket binding failed:");
            exit(-1);
        }
    }

    // Clean up resources and close the connection if it's invalid.
    if (valid < 0)
    {
        close(sock);
        sock = UNINITIALIZED_SOCKET;
    }

    return sock;
}