#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <cstring>

#define UBX_SYNC_CHAR1 0xB5
#define UBX_SYNC_CHAR2 0x62

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

bool readUBXRxmCOR(int fd) {
    const int HEADER_SIZE = 6;
    const int PAYLOAD_SIZE = 16;  // UBX-RXM-COR has a fixed payload of 16 bytes
    const int TOTAL_SIZE = HEADER_SIZE + PAYLOAD_SIZE + 2;

    uint8_t buffer[TOTAL_SIZE];
    int totalRead = 0;

    for (int attempts = 0; attempts < 2000; ++attempts) {
        usleep(1000);
        int r = read(fd, &buffer[totalRead], 1);
        if (r > 0) {
            totalRead += r;

            if (totalRead >= HEADER_SIZE &&
                buffer[0] == UBX_SYNC_CHAR1 &&
                buffer[1] == UBX_SYNC_CHAR2 &&
                buffer[2] == 0x02 && buffer[3] == 0x34) {

                uint16_t len = buffer[4] | (buffer[5] << 8);
                if (len != PAYLOAD_SIZE) {
                    std::cerr << "Unexpected UBX-RXM-COR payload length: " << len << std::endl;
                    return false;
                }

                while (totalRead < TOTAL_SIZE) {
                    r = read(fd, &buffer[totalRead], 1);
                    if (r > 0) totalRead += r;
                }

                const uint8_t* payload = buffer + HEADER_SIZE;

                uint8_t version = payload[0];
                uint8_t type = payload[1]; // 0=none, 1=RTCM, 2=SPARTN, etc.
                uint8_t status = payload[2]; // flags: bits 0=used, 1=corrections available
                uint8_t reserved1 = payload[3];
                uint32_t lastBytes = payload[4] | (payload[5] << 8) | (payload[6] << 16) | (payload[7] << 24);
                uint32_t totalBytes = payload[8] | (payload[9] << 8) | (payload[10] << 16) | (payload[11] << 24);
                uint32_t age = payload[12] | (payload[13] << 8) | (payload[14] << 16) | (payload[15] << 24);

                std::cout << "Correction Type: ";
                switch (type) {
                    case 1: std::cout << "RTCM"; break;
                    case 2: std::cout << "SPARTN"; break;
                    case 3: std::cout << "CLAS"; break;
                    default: std::cout << "Unknown"; break;
                }
                std::cout << std::endl;

                std::cout << "Corrections Used: " << ((status & 0x01) ? "YES" : "NO") << std::endl;
                std::cout << "Corrections Available: " << ((status & 0x02) ? "YES" : "NO") << std::endl;
                std::cout << "Bytes received in last epoch: " << lastBytes << std::endl;
                std::cout << "Total bytes received: " << totalBytes << std::endl;
                std::cout << "Age of corrections (ms): " << age << std::endl;

                return true;
            }
        }
    }

    std::cerr << "⚠️  UBX-RXM-COR response not received." << std::endl;
    return false;
}

int main() {
    const char* port = "/dev/ttyACM0"; // Adjust this if needed
    int fd = openSerialPort(port);
    if (fd < 0) return 1;

    std::cout << "📤 Polling UBX-RXM-COR for correction status..." << std::endl;
    if (!sendUBXMessage(fd, 0x02, 0x34, {})) {
        std::cerr << "Failed to send UBX-RXM-COR." << std::endl;
        return 1;
    }

    readUBXRxmCOR(fd);

    close(fd);
    return 0;
}
