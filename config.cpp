#include <iostream>
#include <vector>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>

#define UBX_SYNC_CHAR1 0xB5
#define UBX_SYNC_CHAR2 0x62

// ===================== SERIAL PORT =====================
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

// ===================== UBX HELPERS =====================
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
    message.reserve(8 + payload.size());

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
    return written == (ssize_t)message.size();
}

// ===================== CONFIG FUNCTIONS =====================
// Enable Secondary Output (CFG-OUT-SECONDARY = 1)
bool enableSecondaryOutput(int fd) {
    std::vector<uint8_t> payload;
    payload.push_back(0x00); // version
    payload.push_back(0x00); // layer (0: RAM only)
    payload.push_back(0x00); // reserved
    payload.push_back(0x00); // reserved

    // KeyID 0x20910227 (CFG-OUT-SECONDARY)
    payload.push_back(0x27);
    payload.push_back(0x02);
    payload.push_back(0x91);
    payload.push_back(0x20);

    // Value = 1
    payload.push_back(0x01);
    payload.push_back(0x00);
    payload.push_back(0x00);
    payload.push_back(0x00);

    return sendUBXMessage(fd, 0x06, 0x8A, payload);
}

// Enable NAV-PVT output on USB (both primary & secondary will use this)
bool enableNavPvtOnUSB(int fd) {
    std::vector<uint8_t> payload;
    payload.push_back(0x00); // version
    payload.push_back(0x00); // layer (0: RAM only)
    payload.push_back(0x00); // reserved
    payload.push_back(0x00); // reserved

    // KeyID 0x20910007 (CFG-MSGOUT-UBX_NAV_PVT_USB)
    payload.push_back(0x07);
    payload.push_back(0x00);
    payload.push_back(0x91);
    payload.push_back(0x20);

    // Value = 1 (enable output every nav cycle)
    payload.push_back(0x01);
    payload.push_back(0x00);
    payload.push_back(0x00);
    payload.push_back(0x00);

    return sendUBXMessage(fd, 0x06, 0x8A, payload);
}

// ===================== NAV-PVT PARSER =====================
void parseNavPvt(const uint8_t* payload, bool secondary) {
    int32_t lon = *(int32_t*)(payload + 24);
    int32_t lat = *(int32_t*)(payload + 28);
    int32_t height = *(int32_t*)(payload + 32);
    uint8_t fixType = payload[20];
    uint8_t flags = payload[21];

    double lonDeg = lon / 1e7;
    double latDeg = lat / 1e7;
    double heightM = height / 1000.0;

    std::cout << (secondary ? "[Secondary]" : "[Primary]  ")
              << " FixType=" << (int)fixType
              << " Lat=" << latDeg
              << " Lon=" << lonDeg
              << " Height=" << heightM << " m"
              << " Flags=0x" << std::hex << (int)flags << std::dec
              << std::endl;
}

// ===================== MAIN LOOP =====================
int main() {
    const char* port = "/dev/ttyACM0"; // Adjust if needed
    int fd = openSerialPort(port);
    if (fd < 0) return 1;

    std::cout << "Enabling secondary output..." << std::endl;
    if (!enableSecondaryOutput(fd))
        std::cerr << "Failed to enable secondary output" << std::endl;

    std::cout << "Enabling NAV-PVT output on USB..." << std::endl;
    if (!enableNavPvtOnUSB(fd))
        std::cerr << "Failed to enable NAV-PVT output" << std::endl;

    std::cout << "Reading NAV-PVT messages..." << std::endl;

    uint8_t buf[100];
    int idx = 0;
    uint8_t header[2] = {UBX_SYNC_CHAR1, UBX_SYNC_CHAR2};

    while (true) {
        uint8_t byte;
        if (read(fd, &byte, 1) > 0) {
            if (idx < 2) {
                if (byte == header[idx]) idx++;
                else idx = 0;
            } else {
                buf[idx - 2] = byte;
                idx++;
                if (idx >= 2 + 4) {
                    uint8_t cls = buf[0];
                    uint8_t id  = buf[1];
                    uint16_t len = buf[2] | (buf[3] << 8);
                    if (idx >= 2 + 4 + len + 2) {
                        if (cls == 0x01 && id == 0x07) {
                            // UBX-NAV-PVT message
                            bool secondary = buf[21] & 0x20; // flags bit 5 = isSecondary
                            parseNavPvt(buf + 4, secondary);
                        }
                        idx = 0;
                    }
                }
            }
        }
    }

    close(fd);
    return 0;
}
