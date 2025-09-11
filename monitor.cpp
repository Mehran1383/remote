#include <iostream>
#include <fstream>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// ===================== SERIAL PORT =====================
int openSerialPort(const char* device, speed_t baudrate = B38400) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror(("Unable to open port " + std::string(device)).c_str());
        return -1;
    }

    struct termios options{};
    tcgetattr(fd, &options);
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);
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

// ===================== NMEA READER =====================
void logNmeaStream(int fd, std::ofstream& file, const std::string& tag) {
    std::string line;
    char c;
    while (read(fd, &c, 1) > 0) {
        if (c == '\n') {
            if (!line.empty() && line[0] == '$') {
                file << line << std::endl;
                std::cout << tag << " " << line << std::endl;
            }
            line.clear();
        } else if (c != '\r') {
            line += c;
        }
    }
}

// ===================== MAIN =====================
int main() {
    // Adjust devices as needed
    const char* usbPort   = "/dev/ttyACM2"; // Primary corrected solution
    const char* uart1Port = "/dev/ttyS1";   // Secondary GNSS-only solution

    int fdUSB   = openSerialPort(usbPort, B38400);
    int fdUART1 = openSerialPort(uart1Port, B38400);

    if (fdUSB < 0 || fdUART1 < 0) {
        std::cerr << "Error opening one of the ports" << std::endl;
        return 1;
    }

    std::ofstream primaryFile("primary_nmea.nmea");
    std::ofstream secondaryFile("secondary_nmea.nmea");

    if (!primaryFile.is_open() || !secondaryFile.is_open()) {
        std::cerr << "Error opening log files" << std::endl;
        return 1;
    }

    std::cout << "Logging started. Press Ctrl+C to stop." << std::endl;

    // Use select() so we can read both ports in one loop
    fd_set readfds;
    while (true) {
        FD_ZERO(&readfds);
        FD_SET(fdUSB, &readfds);
        FD_SET(fdUART1, &readfds);
        int maxfd = std::max(fdUSB, fdUART1) + 1;

        int rv = select(maxfd, &readfds, nullptr, nullptr, nullptr);
        if (rv > 0) {
            if (FD_ISSET(fdUSB, &readfds)) {
                logNmeaStream(fdUSB, primaryFile, "[Primary]");
            }
            if (FD_ISSET(fdUART1, &readfds)) {
                logNmeaStream(fdUART1, secondaryFile, "[Secondary]");
            }
        }
    }

    close(fdUSB);
    close(fdUART1);
    primaryFile.close();
    secondaryFile.close();
    return 0;
}
