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

bool readUBXMonVer(int fd) {
    const int HEADER_SIZE = 6;
    const int RESPONSE_HEADER_LEN = 6;
    const int MAX_PAYLOAD = 100;

    uint8_t buffer[HEADER_SIZE + MAX_PAYLOAD + 2]; // header + payload + checksum
    int totalRead = 0;

    // Wait and read the UBX-MON-VER message (class=0x0A, id=0x04)
    for (int attempts = 0; attempts < 1000; ++attempts) {
        usleep(1000);
        int r = read(fd, &buffer[totalRead], 1);
        if (r > 0) {
            totalRead += r;

            if (totalRead >= 6 &&
                buffer[0] == UBX_SYNC_CHAR1 &&
                buffer[1] == UBX_SYNC_CHAR2 &&
                buffer[2] == 0x0A && buffer[3] == 0x04) {
                uint16_t len = buffer[4] | (buffer[5] << 8);

                while (totalRead < 6 + len + 2) {
                    r = read(fd, &buffer[totalRead], 1);
                    if (r > 0) totalRead += r;
                }

                // Extract version strings
                const uint8_t* payload = buffer + 6;

                std::string swVersion(reinterpret_cast<const char*>(payload), 30);
                std::string hwVersion(reinterpret_cast<const char*>(payload + 30), 30);

                std::cout << "🟢 Software Version: " << swVersion.c_str() << std::endl;
                std::cout << "🟢 Hardware Version: " << hwVersion.c_str() << std::endl;

                // Optional extensions (up to N additional 30-byte strings)
                int extCount = (len - 60) / 30;
                for (int i = 0; i < extCount; ++i) {
                    std::string ext(reinterpret_cast<const char*>(payload + 60 + i * 30), 30);
                    std::cout << "🔧 Extension[" << i << "]: " << ext.c_str() << std::endl;
                }

                return true;
            }
        }
    }

    std::cerr << "⚠️  UBX-MON-VER response not received." << std::endl;
    return false;
}

int main() {
    const char* port = "/dev/ttyACM0"; // Adjust this if needed
    int fd = openSerialPort(port);
    if (fd < 0) return 1;

    std::cout << "📤 Sending UBX-MON-VER poll..." << std::endl;
    if (!sendUBXMessage(fd, 0x0A, 0x04, {})) {
        std::cerr << "Failed to send UBX-MON-VER." << std::endl;
        return 1;
    }

    std::cout << "📥 Waiting for UBX-MON-VER response..." << std::endl;
    readUBXMonVer(fd);

    close(fd);
    return 0;
}
