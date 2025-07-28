#include <iostream>
#include <cstring>
#include <vector>

#include <libserial/SerialPort.h>
#include <unistd.h>

#define PORT "/dev/ACM0"

using namespace LibSerial;

const uint8_t delay = 10e5;

// UBX Configuration Commands
const std::vector<uint8_t> CFG_RATE_5HZ = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A  // 5Hz update rate
};

const std::vector<uint8_t> CFG_NAV5_RTK = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 
    0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC  // RTK + Automotive mode
};

const std::vector<uint8_t> CFG_MSG_UBX_NAV_PVT = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE1  // Enable NAV-PVT messages
};

void sendUBX(SerialPort& port, const std::vector<uint8_t>& msg) 
{
    port.Write(msg);
    usleep(delay); 
}

int main() 
{
    SerialPort serial;
    
    try {
        serial.Open(PORT);
        serial.SetBaudRate(BaudRate::BAUD_115200);
        serial.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
        serial.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
        serial.SetParity(Parity::PARITY_NONE);
        serial.SetStopBits(StopBits::STOP_BITS_1);

        std::cout << "Configuring ZED-F9R..." << std::endl;

        // Send configuration commands
        sendUBX(serial, CFG_RATE_5HZ);
        sendUBX(serial, CFG_NAV5_RTK);
        sendUBX(serial, CFG_MSG_UBX_NAV_PVT);

        std::cout << "Configuration done. Reading GNSS data..." << std::endl;

        while (true) {
            if (serial.IsDataAvailable()) {
                std::string data;
                serial.ReadLine(data, '\n', 1024);
                std::cout << data << std::endl;
            }
            usleep(delay); 
        }
    } catch (const OpenFailed&) {
        std::cerr << "Failed to open serial port!" << std::endl;
        return 1;
    }

    serial.Close();
    return 0;
}