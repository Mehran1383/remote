#include <iostream>
#include <fstream>
#include <vector>
#include <serial/serial.h>
#include <unistd.h>
#include <cstring>

class UBloxF9R {
public:
    UBloxF9R(const std::string &port, uint32_t baudrate = 115200) 
        : serial(port, baudrate, serial::Timeout::simpleTimeout(1000)) {
        if (!serial.isOpen()) {
            throw std::runtime_error("Failed to open serial port");
        }
    }

    bool sendUbxMessage(uint8_t class_id, uint8_t message_id, const std::vector<uint8_t> &payload) {
        std::vector<uint8_t> packet;
        
        // Header
        packet.push_back(0xB5); // Sync char 1
        packet.push_back(0x62); // Sync char 2
        packet.push_back(class_id);
        packet.push_back(message_id);
        packet.push_back(payload.size() & 0xFF); // Length LSB
        packet.push_back((payload.size() >> 8) & 0xFF); // Length MSB
        
        // Payload
        packet.insert(packet.end(), payload.begin(), payload.end());
        
        // Calculate checksum
        uint8_t ck_a = 0, ck_b = 0;
        for (size_t i = 2; i < packet.size(); i++) {
            ck_a += packet[i];
            ck_b += ck_a;
        }
        
        packet.push_back(ck_a);
        packet.push_back(ck_b);
        
        // Send packet
        size_t bytes_written = serial.write(packet);
        return bytes_written == packet.size();
    }

    bool configureRTK(const std::string &server, const std::string &username, 
                      const std::string &password, uint16_t port) {
        // 1. Configure message rates (enable RTCM3 messages)
        std::vector<uint8_t> cfg_msg = {0x01, 0x05, 0x00, 0x01, 0x00, 0x01}; // RTCM 1074
        if (!sendUbxMessage(0x06, 0x01, cfg_msg)) return false;
        cfg_msg = {0x01, 0x05, 0x00, 0x01, 0x00, 0x01}; // RTCM 1084
        if (!sendUbxMessage(0x06, 0x01, cfg_msg)) return false;
        cfg_msg = {0x01, 0x05, 0x00, 0x01, 0x00, 0x01}; // RTCM 1094
        if (!sendUbxMessage(0x06, 0x01, cfg_msg)) return false;
        cfg_msg = {0x01, 0x05, 0x00, 0x01, 0x00, 0x01}; // RTCM 1124
        if (!sendUbxMessage(0x06, 0x01, cfg_msg)) return false;
        
        // 2. Set navigation mode to RTK
        std::vector<uint8_t> nav_mode = {0x03, 0x00}; // RTK mode
        if (!sendUbxMessage(0x06, 0x24, nav_mode)) return false;
        
        // 3. Configure PointPerfect L-Band (if using)
        // This would require your specific PointPerfect credentials
        
        // 4. Configure NTRIP client for correction data
        std::vector<uint8_t> ntrip_cfg;
        
        // Add server address (max 256 bytes)
        ntrip_cfg.push_back(0x01); // Version
        ntrip_cfg.push_back(0x00); // Reserved
        ntrip_cfg.push_back(0x00); // Reserved
        ntrip_cfg.push_back(0x00); // Reserved
        
        // Server address
        for (char c : server) {
            ntrip_cfg.push_back(static_cast<uint8_t>(c));
        }
        ntrip_cfg.push_back(0x00); // Null terminator
        
        // Username
        for (char c : username) {
            ntrip_cfg.push_back(static_cast<uint8_t>(c));
        }
        ntrip_cfg.push_back(0x00); // Null terminator
        
        // Password
        for (char c : password) {
            ntrip_cfg.push_back(static_cast<uint8_t>(c));
        }
        ntrip_cfg.push_back(0x00); // Null terminator
        
        // Mountpoint (for NTRIP)
        std::string mountpoint = "NEAR-SPARTN"; // PointPerfect mountpoint
        for (char c : mountpoint) {
            ntrip_cfg.push_back(static_cast<uint8_t>(c));
        }
        ntrip_cfg.push_back(0x00); // Null terminator
        
        // Port
        ntrip_cfg.push_back(port & 0xFF);
        ntrip_cfg.push_back((port >> 8) & 0xFF);
        
        // Send NTRIP configuration
        if (!sendUbxMessage(0x06, 0x72, ntrip_cfg)) return false;
        
        // 5. Save configuration
        std::vector<uint8_t> save_cfg = {0x00, 0x00, 0x00, 0x00};
        if (!sendUbxMessage(0x06, 0x09, save_cfg)) return false;
        
        return true;
    }

private:
    serial::Serial serial;
};

int main() {
    try {
        // Replace with your actual serial port (e.g., /dev/ttyACM0)
        UBloxF9R gnss("/dev/ttyACM0");
        
        // Replace with your PointPerfect credentials
        std::string server = "ppntrip.services.u-blox.com";
        std::string username = "sVSa5305Zh9m";
        std::string password = "1JduoQ&9op";
        uint16_t port = 2101;
        
        if (gnss.configureRTK(server, username, password, port)) {
            std::cout << "RTK configuration successful!" << std::endl;
        } else {
            std::cerr << "Failed to configure RTK" << std::endl;
            return 1;
        }
        
        // You can add code here to read and parse the GNSS data
        // to verify RTK functionality
        
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}