#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <cstring>

#define UBX_SYNC_CHAR1 0xB5
#define UBX_SYNC_CHAR2 0x62

// Function definitions (as before)
int openSerialPort(const char* device);
void calculateChecksum(const std::vector<uint8_t>& payload, uint8_t& ckA, uint8_t& ckB);
bool sendUBXMessage(int fd, uint8_t cls, uint8_t id, const std::vector<uint8_t>& payload);

// Open serial port (same as before)
int openSerialPort(const char* device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Unable to open port");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B38400);
    cfsetospeed(&options, B38400);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    tcsetattr(fd, TCSANOW, &options);

    return fd;
}

void calculateChecksum(const std::vector<uint8_t>& payload, uint8_t& ckA, uint8_t& ckB) {
    ckA = 0;
    ckB = 0;
    for (uint8_t byte : payload) {
        ckA += byte;
        ckB += ckA;
    }
}

bool sendUBXMessage(int fd, uint8_t cls, uint8_t id, const std::vector<uint8_t>& payload) {
    uint16_t length = payload.size();
    std::vector<uint8_t> message;

    message.push_back(UBX_SYNC_CHAR1);
    message.push_back(UBX_SYNC_CHAR2);
    message.push_back(cls);
    message.push_back(id);
    message.push_back(length & 0xFF);
    message.push_back((length >> 8) & 0xFF);
    message.insert(message.end(), payload.begin(), payload.end());

    uint8_t ckA, ckB;
    calculateChecksum(std::vector<uint8_t>(message.begin() + 2, message.end()), ckA, ckB);
    message.push_back(ckA);
    message.push_back(ckB);

    ssize_t written = write(fd, message.data(), message.size());
    return written == static_cast<ssize_t>(message.size());
}

int main() {
    const char* port = "/dev/ttyACM0"; // Adjust this if needed
    int fd = openSerialPort(port);
    if (fd < 0) return 1;

    std::vector<uint8_t> payload;

    // UBX-CFG-VALSET Header
    payload.push_back(0x00); // Version
    payload.push_back(0x00); // Layer (0: RAM only)
    payload.push_back(0x00); // Reserved
    payload.push_back(0x00); // Reserved

    // CFG-MSGOUT-NMEA_GGA_USB KeyID: 0x209100BA, Value: 1 (enable)
    payload.push_back(0xBA); payload.push_back(0x00); payload.push_back(0x91); payload.push_back(0x20);
    payload.push_back(0x01); payload.push_back(0x00); payload.push_back(0x00); payload.push_back(0x00);

    // CFG-MSGOUT-NMEA_RMC_USB KeyID: 0x209100B5, Value: 1 (enable)
    payload.push_back(0xB5); payload.push_back(0x00); payload.push_back(0x91); payload.push_back(0x20);
    payload.push_back(0x01); payload.push_back(0x00); payload.push_back(0x00); payload.push_back(0x00);

    // If you want to enable for UART1 instead, change KeyIDs to:
    // CFG-MSGOUT-NMEA_GGA_UART1: 0x20910001
    // CFG-MSGOUT-NMEA_RMC_UART1: 0x20910005

    std::cout << "Sending UBX-CFG-VALSET to enable GGA and RMC output..." << std::endl;
    if (sendUBXMessage(fd, 0x06, 0x8A, payload)) {
        std::cout << "Configuration sent successfully." << std::endl;
    } else {
        std::cerr << "Failed to send configuration." << std::endl;
    }

    close(fd);
    return 0;
}
