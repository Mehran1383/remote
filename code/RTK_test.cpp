#include <iostream>
#include <string>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

// UBX protocol constants
#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62
#define UBX_CLASS_CFG 0x06
#define UBX_ID_CFG_PRT 0x00
#define UBX_ID_CFG_NTRIP 0x4E
#define UBX_ID_CFG_MSG 0x01
#define UBX_ID_CFG_NAV5 0x24
#define UBX_ID_CFG_RATE 0x08
#define UBX_ID_CFG_CFG 0x09
#define UBX_CLASS_NAV 0x01
#define UBX_ID_NAV_PVT 0x07

// Serial port configuration
int configureSerialPort(const char* port, int baudRate) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        perror("Error opening serial port");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    
    // Set baud rate
    cfsetispeed(&options, baudRate);
    cfsetospeed(&options, baudRate);
    
    // 8N1 configuration
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    
    // Enable receiver, ignore modem control lines
    options.c_cflag |= (CLOCAL | CREAD);
    
    // Raw input mode
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // Disable software flow control
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    
    // Raw output mode
    options.c_oflag &= ~OPOST;
    
    // Set timeout to 100ms
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;
    
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

// Calculate UBX checksum
void calculateChecksum(const std::vector<uint8_t>& payload, uint8_t& ck_a, uint8_t& ck_b) {
    ck_a = 0;
    ck_b = 0;
    for (uint8_t byte : payload) {
        ck_a += byte;
        ck_b += ck_a;
    }
}

// Send UBX message
bool sendUbxMessage(int fd, uint8_t msgClass, uint8_t msgId, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> message;
    message.push_back(UBX_SYNC1);
    message.push_back(UBX_SYNC2);
    message.push_back(msgClass);
    message.push_back(msgId);
    
    uint16_t length = payload.size();
    message.push_back(length & 0xFF);
    message.push_back((length >> 8) & 0xFF);
    
    message.insert(message.end(), payload.begin(), payload.end());
    
    uint8_t ck_a, ck_b;
    calculateChecksum(payload, ck_a, ck_b);
    message.push_back(ck_a);
    message.push_back(ck_b);
    
    return write(fd, message.data(), message.size()) == message.size();
}

// Configure ZED-F9R for RTK
bool configureRtk(int fd, const std::string& caster, uint16_t port, 
                 const std::string& mountpoint, const std::string& username, 
                 const std::string& password) {
    // 1. Configure USB port for UBX protocol
    std::vector<uint8_t> prtPayload = {
        0x03,       // USB port
        0x00,       // Reserved
        0x00, 0x00, // TX ready
        0x00, 0x00, 0x00, 0x00, // Mode (UBX only)
        0x00, 0x00, 0x00, 0x00, // Baud rate (not used for USB)
        0x01, 0x00, // Input protocol: UBX
        0x01, 0x00, // Output protocol: UBX
        0x00, 0x00, // Flags
        0x00, 0x00  // Reserved
    };
    if (!sendUbxMessage(fd, UBX_CLASS_CFG, UBX_ID_CFG_PRT, prtPayload)) {
        std::cerr << "Failed to configure USB port" << std::endl;
        return false;
    }
    sleep(1); // Allow time for processing

    // 2. Configure NTRIP client
    std::vector<uint8_t> ntripPayload(64, 0);
    ntripPayload[0] = 2; // NTRIP version 2.0
    
    // Copy caster address (max 64 bytes)
    size_t copyLen = std::min(caster.size(), (size_t)64);
    std::memcpy(&ntripPayload[4], caster.c_str(), copyLen);
    
    // Port (little-endian)
    ntripPayload[68] = port & 0xFF;
    ntripPayload[69] = (port >> 8) & 0xFF;
    
    // Copy mountpoint (max 64 bytes)
    copyLen = std::min(mountpoint.size(), (size_t)64);
    std::memcpy(&ntripPayload[70], mountpoint.c_str(), copyLen);
    
    // Copy username (max 64 bytes)
    copyLen = std::min(username.size(), (size_t)64);
    std::memcpy(&ntripPayload[134], username.c_str(), copyLen);
    
    // Copy password (max 64 bytes)
    copyLen = std::min(password.size(), (size_t)64);
    std::memcpy(&ntripPayload[198], password.c_str(), copyLen);
    
    // Flags: Enable NTRIP client (bit 0) and send GGA (bit 1)
    ntripPayload[262] = 0x03;
    
    if (!sendUbxMessage(fd, UBX_CLASS_CFG, UBX_ID_CFG_NTRIP, ntripPayload)) {
        std::cerr << "Failed to configure NTRIP client" << std::endl;
        return false;
    }
    sleep(1);

    // 3. Enable NAV-PVT message (1Hz)
    std::vector<uint8_t> msgPayload = {
        0x01, 0x07, // NAV-PVT message
        0x03,       // USB port
        0x01,       // Rate (1 Hz)
        0x00, 0x00  // Reserved
    };
    if (!sendUbxMessage(fd, UBX_CLASS_CFG, UBX_ID_CFG_MSG, msgPayload)) {
        std::cerr << "Failed to configure NAV-PVT message" << std::endl;
        return false;
    }
    sleep(1);

    // 4. Set dynamics model to automotive
    std::vector<uint8_t> nav5Payload = {
        0x00,       // Version
        0x00,       // Reserved
        0x00,       // Reserved
        0x08,       // Dynamics model: Automotive
        0x00,       // Reserved
        0x00,       // Reserved
        0x00, 0x00, // Reserved
        0x00, 0x00, // Reserved
        0x00, 0x00, // Reserved
        0x00, 0x00, // Reserved
        0x00, 0x00, // Reserved
        0x00, 0x00, // Reserved
        0x00, 0x00, // Reserved
        0x00, 0x00, // Reserved
        0x00, 0x00, // Reserved
        0x00, 0x00, // Reserved
        0x00, 0x00, // Reserved
        0x00, 0x00, // Reserved
        0x00, 0x00, // Reserved
        0x00, 0x00, // Reserved
        0x00, 0x00, // Reserved
        0x00, 0x00, // Reserved
        0x00, 0x00, // Reserved
        0x00, 0x00, // Reserved
        0x00, 0x00, // Reserved
        0x00, 0x00, // Reserved
        0x00, 0x00, // Reserved
        0x00, 0x00, // Reserved
        0x00, 0x00, // Reserved
        0x00, 0x00  // Reserved
    };
    if (!sendUbxMessage(fd, UBX_CLASS_CFG, UBX_ID_CFG_NAV5, nav5Payload)) {
        std::cerr << "Failed to configure dynamics model" << std::endl;
        return false;
    }
    sleep(1);

    // 5. Set measurement rate to 1Hz
    std::vector<uint8_t> ratePayload = {
        0xE8, 0x03, // Measurement rate: 1000ms (1Hz)
        0x00, 0x00, // Navigation rate: 1 cycle
        0x00, 0x00  // Time reference: UTC
    };
    if (!sendUbxMessage(fd, UBX_CLASS_CFG, UBX_ID_CFG_RATE, ratePayload)) {
        std::cerr << "Failed to configure measurement rate" << std::endl;
        return false;
    }
    sleep(1);

    // 6. Save configuration
    std::vector<uint8_t> cfgPayload = {
        0x00, 0x00, 0x00, 0x00, // Clear mask
        0xFF, 0xFF, 0x00, 0x00, // Save mask: All sections
        0x00, 0x00, 0x00, 0x00, // Load mask
        0x07                    // Device mask: BBR, Flash, EEPROM
    };
    if (!sendUbxMessage(fd, UBX_CLASS_CFG, UBX_ID_CFG_CFG, cfgPayload)) {
        std::cerr << "Failed to save configuration" << std::endl;
        return false;
    }
    sleep(1);

    return true;
}

// Read and parse UBX messages
void readUbxMessages(int fd) {
    std::vector<uint8_t> buffer(1024);
    uint8_t state = 0;
    uint8_t msgClass = 0, msgId = 0;
    uint16_t length = 0;
    uint16_t payloadIndex = 0;
    std::vector<uint8_t> payload;
    uint8_t ck_a = 0, ck_b = 0;
    uint8_t expected_ck_a = 0, expected_ck_b = 0;

    while (true) {
        int bytesRead = read(fd, buffer.data(), buffer.size());
        if (bytesRead <= 0) continue;

        for (int i = 0; i < bytesRead; i++) {
            uint8_t byte = buffer[i];

            switch (state) {
                case 0: // Wait for SYNC1
                    if (byte == UBX_SYNC1) state = 1;
                    break;
                case 1: // Wait for SYNC2
                    if (byte == UBX_SYNC2) state = 2;
                    else state = 0;
                    break;
                case 2: // Read message class
                    msgClass = byte;
                    state = 3;
                    break;
                case 3: // Read message ID
                    msgId = byte;
                    state = 4;
                    break;
                case 4: // Read length LSB
                    length = byte;
                    state = 5;
                    break;
                case 5: // Read length MSB
                    length |= (byte << 8);
                    payloadIndex = 0;
                    payload.resize(length);
                    state = (length > 0) ? 6 : 7;
                    break;
                case 6: // Read payload
                    payload[payloadIndex++] = byte;
                    if (payloadIndex >= length) state = 7;
                    break;
                case 7: // Read checksum A
                    expected_ck_a = byte;
                    state = 8;
                    break;
                case 8: // Read checksum B
                    expected_ck_b = byte;
                    calculateChecksum(payload, ck_a, ck_b);
                    
                    if (ck_a == expected_ck_a && ck_b == expected_ck_b) {
                        // Valid UBX message
                        if (msgClass == UBX_CLASS_NAV && msgId == UBX_ID_NAV_PVT) {
                            // Parse NAV-PVT message
                            uint8_t fixType = payload[20];
                            std::cout << "Fix Type: ";
                            switch (fixType) {
                                case 0: std::cout << "No fix"; break;
                                case 1: std::cout << "Dead reckoning"; break;
                                case 2: std::cout << "2D fix"; break;
                                case 3: std::cout << "3D fix"; break;
                                case 4: std::cout << "GNSS + dead reckoning"; break;
                                case 5: std::cout << "Time only"; break;
                                case 6: std::cout << "RTK fixed"; break;
                                case 7: std::cout << "RTK float"; break;
                                default: std::cout << "Unknown"; break;
                            }
                            
                            // Extract position (if valid fix)
                            if (fixType >= 2 && fixType <= 7) {
                                int32_t lat = *(int32_t*)&payload[28];
                                int32_t lon = *(int32_t*)&payload[32];
                                int32_t alt = *(int32_t*)&payload[36];
                                
                                double latitude = lat * 1e-7;
                                double longitude = lon * 1e-7;
                                double altitude = alt * 1e-3;
                                
                                std::cout << " | Position: " 
                                          << latitude << ", " 
                                          << longitude << ", " 
                                          << altitude << "m";
                            }
                            std::cout << std::endl;
                        }
                    }
                    state = 0;
                    break;
            }
        }
    }
}

int main() {
    // Perfect Point service credentials (replace with your actual credentials)
    std::string caster = "ppntrip.services.u-blox.com";
    uint16_t port = 2101;
    std::string mountpoint = "NEAR-SPARTN";
    std::string username = "sVSa5305Zh9m";
    std::string password = "1JduoQ&9op";
    
    // Configure serial port (adjust device path as needed)
    const char* serialPort = "/dev/ttyACM0";
    int baudRate = 38400; // Default ZED-F9R baud rate
    
    int fd = configureSerialPort(serialPort, baudRate);
    if (fd < 0) {
        std::cerr << "Failed to open serial port" << std::endl;
        return 1;
    }
    
    // Configure RTK
    if (!configureRtk(fd, caster, port, mountpoint, username, password)) {
        std::cerr << "RTK configuration failed" << std::endl;
        close(fd);
        return 1;
    }
    
    std::cout << "RTK configuration successful. Reading data..." << std::endl;
    
    // Read and process UBX messages
    readUbxMessages(fd);
    
    close(fd);
    return 0;
}