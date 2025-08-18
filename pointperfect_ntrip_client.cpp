#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <netdb.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <csignal>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

static constexpr int SOCKET_RECONNECT_DELAY = 10;    // seconds
static constexpr int SERVER_RECONNECT_DELAY = 60;    // seconds
static constexpr int SOCKET_TIMEOUT = 5;             // seconds
static constexpr int SOCKET_MAX_RECV_TIMEOUTS = 12;  // count
static constexpr const char* DEFAULT_NTRIP_SERVER = "ppntrip.services.u-blox.com";
static constexpr int DEFAULT_NTRIP_PORT = 2101;
static constexpr int MOUNTPOINT_INFO_SIZE = 11;

static const std::vector<std::string> QUALITIES = {
    "NOFIX","GNSS","DGNSS","PPS","FIXED","FLOAT","DR","MAN","SIM"
};
static const int STATS_LEVEL = 100;

std::string now_timestamp() 
{
    time_t t = time(nullptr);
    char buf[64];
    strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", localtime(&t));
    return std::string(buf);
}

std::string base64_encode(const std::string &in) 
{
    static const char *b64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    int val = 0, valb = -6;
    for (unsigned char c : in) {
        val = (val << 8) + c;
        valb += 8;
        while (valb >= 0) {
            out.push_back(b64[(val >> valb) & 0x3F]);
            valb -= 6;
        }
    }
    if (valb > -6) 
        out.push_back(b64[((val << 8) >> (valb + 8)) & 0x3F]);
    while (out.size() % 4)
        out.push_back('=');
    return out;
}

void log_info(const std::string &s) { std::cout << "INFO: " << s << std::endl; }
void log_error(const std::string &s) { std::cerr << "ERROR: " << s << std::endl; }
void log_warning(const std::string &s) { std::cerr << "WARN: " << s << std::endl; }

struct MountPointInfo {
    std::string name;
    std::string identifier;
    std::string format;
    std::string format_details;
    std::string carrier;
    std::string nav_system;
    std::string network;
    std::string country;
    double lat;
    double lon;
};

class NtripClient {
public:
    NtripClient(const std::string &host, int port, const std::string &user, const std::string &pw)
        : host_(host), port_(port), user_(user), pass_(pw), streaming_(false), sock_fd_(-1)
    {}

    ~NtripClient() { stop_stream(); }

    std::vector<MountPointInfo> get_mountpoints() 
    {
        int fd = open_socket();
        if (fd < 0) 
            return {};

        std::string req = make_request("");
        ssize_t sent = send(fd, req.c_str(), req.size(), 0);
        if (sent < 0)
            return {};

        std::string data;
        char buf[1024];
        ssize_t r;
        
        while ((r = recv(fd, buf, sizeof(buf), 0)) > 0) {
            data.append(buf, buf + r);
        }
        close(fd);

        std::vector<MountPointInfo> mounts;
        std::istringstream iss(data);
        std::string line;

        while (std::getline(iss, line)) {
            if (line.rfind("STR;", 0) == 0) { // mountpoint stream
                
                std::vector<std::string> cols;
                std::istringstream ls(line);
                std::string token;

                while (std::getline(ls, token, ';')) 
                    cols.push_back(token);

                if (cols.size() >= MOUNTPOINT_INFO_SIZE) {
                    MountPointInfo mp;
                    mp.name = cols[1];
                    mp.identifier = cols[2];
                    mp.format = cols[3];
                    mp.format_details = cols[4];
                    mp.carrier = cols[5];
                    mp.nav_system = cols[6];
                    mp.network = cols[7];
                    mp.country = cols[8];
                    mp.lat = std::stod(cols[9]);
                    mp.lon = std::stod(cols[10]);
                    mounts.push_back(mp);
                }
            }
        }
        return mounts;
    }

    // Start streaming in background thread; callback will be invoked with raw bytes
    void start_stream(const std::string &mountpoint,
                      std::function<void(const std::vector<uint8_t>&)> callback)
    {
        stop_stream();
        streaming_ = true;
        callback_ = callback;
        mountpoint_ = mountpoint;
        worker_ = std::thread(&NtripClient::stream_thread, this);
    }

    void stop_stream(void) 
    {
        streaming_ = false;
        if (worker_.joinable()) 
            worker_.join();
        std::lock_guard<std::mutex> lk(sock_mtx_);
        if (sock_fd_ >= 0) { 
            close(sock_fd_); 
            sock_fd_ = -1; 
        }
    }

    // send GGA line (with CRLF) to the server's socket if available
    bool send_gga(const std::string &gga) 
    {
        std::lock_guard<std::mutex> lk(sock_mtx_);
        if (sock_fd_ < 0) 
            return false;

        ssize_t r = send(sock_fd_, gga.c_str(), gga.size(), 0);
        if (r < 0) {
            log_warning("Error sending GGA: " + std::string(strerror(errno)));
            return false;
        }
        return true;
    }

private:
    std::string make_request(const std::string &mount) 
    {
        std::string auth = base64_encode(user_ + ":" + pass_);
        std::ostringstream req;
        req << "GET /" << mount << " HTTP/1.1\r\n";
        req << "Host: " << host_ << ":" << port_ << "\r\n";
        req << "User-Agent: NTRIP Client\r\n";
        req << "Accept: */*\r\n";
        req << "Authorization: Basic " << auth << "\r\n";
        req << "Connection: close\r\n\r\n";
        return req.str();
    }

    int open_socket(void) 
    {
        struct addrinfo hints{}, *res = nullptr;
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        int rc = getaddrinfo(host_.c_str(), std::to_string(port_).c_str(), &hints, &res);
        if (rc != 0) {
            log_error(std::string("getaddrinfo: ") + gai_strerror(rc));
            return -1;
        }

        int fd = -1;
        for (struct addrinfo *p = res; p; p = p->ai_next) {
            fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
            if (fd < 0) 
                continue;
            // set timeout for connect
            struct timeval tv { SOCKET_TIMEOUT, 0 };
            setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
            if (connect(fd, p->ai_addr, p->ai_addrlen) == 0) {
                freeaddrinfo(res);
                return fd;
            }
            close(fd);
            fd = -1;
        }
        freeaddrinfo(res);
        log_error("Unable to connect to server");
        return -1;
    }

    void stream_thread()
    {
        int delay = 0;
        while (streaming_) {
            int fd = open_socket();
            if (fd < 0) {
                delay = SOCKET_RECONNECT_DELAY;
                log_warning("Error connecting; retrying in " + std::to_string(delay) + "s");
                if (!wait_or_stop(delay)) 
                    break;
                continue;
            }

            std::string req = make_request(mountpoint_);
            ssize_t sent = send(fd, req.c_str(), req.size(), 0);
            if (sent < 0) {
                close(fd);
                delay = SERVER_RECONNECT_DELAY;
                if (!wait_or_stop(delay)) 
                    break;
                continue;
            }

            // read initial response (headers)
            std::string header_data;
            char buf[2048];
            ssize_t r = recv(fd, buf, sizeof(buf), 0);
            if (r <= 0) {
                close(fd);
                delay = SERVER_RECONNECT_DELAY;
                if (!wait_or_stop(delay)) 
                    break;
                continue;
            }
            header_data.append(buf, buf + r);
            // split headers by CRLF CRLF
            size_t hdr_end = header_data.find("\r\n\r\n");
            std::vector<uint8_t> initial_data;
            if (hdr_end != std::string::npos) {
                std::string headers = header_data.substr(0, hdr_end);
                std::istringstream hs(headers);
                std::string status_line;
                std::getline(hs, status_line);
                // inspect status
                std::istringstream st(status_line);
                std::string http, code;
                st >> http >> code;
                if (code != "200") {
                    log_error("HTTP Error: " + status_line + " ; retrying in " + 
                              std::to_string(SERVER_RECONNECT_DELAY) + "s");
                    close(fd);
                    delay = SERVER_RECONNECT_DELAY;
                    if (!wait_or_stop(delay)) 
                        break;
                    continue;
                }
                // remainder after header end is initial payload
                std::string payload = header_data.substr(hdr_end + 4);
                initial_data.assign(payload.begin(), payload.end());
            } else {
                // no headers end found yet - treat whole chunk as initial payload
                initial_data.assign(header_data.begin(), header_data.end());
            }

            {
                // make socket available for GGA sends
                std::lock_guard<std::mutex> lk(sock_mtx_);
                sock_fd_ = fd;
            }

            if (!initial_data.empty() && callback_) 
                callback_(initial_data);

            int timeouts = 0;
            // set recv timeout on socket
            struct timeval tv { SOCKET_TIMEOUT, 0 };
            setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

            while (streaming_ && timeouts < SOCKET_MAX_RECV_TIMEOUTS) {
                ssize_t n = recv(fd, buf, sizeof(buf), 0);
                if (n > 0) {
                    timeouts = 0;
                    std::vector<uint8_t> bb(buf, buf + n);
                    if (callback_) 
                        callback_(bb);
                } else if (n == 0) {
                    log_warning("Connection closed by server, reconnecting");
                    delay = SERVER_RECONNECT_DELAY;
                    break;
                } else {
                    if (errno == EWOULDBLOCK || errno == EAGAIN) {
                        timeouts++;
                        continue;
                    } else {
                        log_warning(std::string("Recv error: ") + strerror(errno));
                        delay = SOCKET_RECONNECT_DELAY;
                        break;
                    }
                }
            }

            {
                std::lock_guard<std::mutex> lk(sock_mtx_);
                if (sock_fd_ >= 0) { close(sock_fd_); sock_fd_ = -1; }
            }
            close(fd);
            if (!wait_or_stop(delay)) 
                break;
        } 
    }

    bool wait_or_stop(int seconds) 
    {
        if (seconds <= 0) 
            return true;
        for (int i = 0; i < seconds && streaming_; ++i) 
            std::this_thread::sleep_for(1s);
        return streaming_;
    }

    std::string host_;
    int port_;
    std::string user_, pass_;
    std::atomic<bool> streaming_;
    std::thread worker_;
    std::mutex sock_mtx_;
    int sock_fd_;
    std::function<void(const std::vector<uint8_t>&)> callback_;
    std::string mountpoint_;
};

class NmeaParser {
public:
    using CallbackFn = std::function<void(const std::string&)>;
    NmeaParser(const std::map<std::regex, CallbackFn> &cbs) : callbacks_(cbs), buffering(false) {}

    void parse(const std::vector<uint8_t> &data) 
    {
        for (uint8_t b : data) {
            if (b == '$') {
                buffer.clear();
                buffer.push_back(b);
                buffering = true;
            } else if (buffering) {
                if ( (b >= 'A' && b <= 'Z') ||
                     (b >= '0' && b <= '9') ||
                     b==',' || b=='.' || b=='-' || b=='*') {
                    buffer.push_back(b);
                } else if (b == 0x0D) { // CR
                    if (buffer.size() > 3 && buffer[buffer.size() - 3] == '*') {
                        // parse checksum
                        std::string chkstr;
                        chkstr.push_back((char)buffer[buffer.size() - 2]);
                        chkstr.push_back((char)buffer[buffer.size() - 1]);
                        int chksum_received = -1;
                        try {
                            chksum_received = std::stoi(chkstr, nullptr, 16);
                        } catch (...) { chksum_received = -1; }

                        int chksum = 0;
                        for (size_t i = 1; i + 3 < buffer.size(); ++i) 
                            chksum ^= buffer[i];

                        if (chksum == chksum_received) {
                            std::string s(buffer.begin(), buffer.end());
                            // try callbacks
                            for (auto &kv : callbacks_) {
                                if (std::regex_search(s, kv.first)) {
                                    kv.second(s);
                                }
                            }
                        } else {
                            log_warning("chksum error");
                        }
                    }
                    buffering = false;
                } else {
                    buffering = false;
                }
            } 
        } 
    }

private:
    std::map<std::regex, CallbackFn> callbacks_;
    std::vector<char> buffer;
    bool buffering;
};

class PosixSerial {
public:
    PosixSerial(): fd_(-1) {}
    ~PosixSerial(){ close_port(); }

    bool open_port(const std::string &device, int baud = 115200) 
    {
        fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            log_error("Failed to open serial port: " + device + " : " + strerror(errno));
            return false;
        }
        // configure termios
        struct termios tio{};
        if (tcgetattr(fd_, &tio) != 0) {
            log_error("tcgetattr failed: " + std::string(strerror(errno)));
            close(fd_); 
            fd_ = -1; 
            return false;
        }
        cfmakeraw(&tio);
        // input flags: no parity, disable flow control
        tio.c_cflag &= ~CRTSCTS;
        tio.c_cflag |= CREAD | CLOCAL;
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS8;
        tio.c_cflag &= ~PARENB;

        speed_t sp = B115200;

        cfsetispeed(&tio, sp);
        cfsetospeed(&tio, sp);
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 1; // tenths of seconds
        tcflush(fd_, TCIFLUSH);
        if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
            log_error("tcsetattr failed: " + std::string(strerror(errno)));
            close(fd_); 
            fd_ = -1; 
            return false;
        }
        return true;
    }

    void close_port() 
    {
        if (fd_ >= 0) 
            close(fd_);
        fd_ = -1;
    }

    // read up to bufsize into vector, return bytes read
    ssize_t read_some(std::vector<uint8_t> &buf, size_t maxBytes) 
    {
        if (fd_ < 0) 
            return -1;

        buf.resize(maxBytes);
        ssize_t r = read(fd_, buf.data(), (ssize_t)maxBytes);
        if (r > 0) 
            buf.resize(r);
        else 
            buf.clear();

        return r;
    }

    ssize_t write_bytes(const std::vector<uint8_t> &data) 
    {
        if (fd_ < 0) 
            return -1;

        return write(fd_, data.data(), data.size());
    }

    // write raw string 
    ssize_t write_string(const std::string &s) 
    {
        if (fd_ < 0) 
            return -1;

        return write(fd_, s.c_str(), s.size());
    }

private:
    int fd_;
};

class PointPerfectClient {
public:
    PointPerfectClient(PosixSerial &serial, NtripClient &ntrip, const std::string &mountpoint,
                       int gga_interval, int distance, double epochs, std::ofstream *ubxfile, int stats_interval)
        : serial_(serial), ntrip_(ntrip), mountpoint_(mountpoint), gga_interval_(gga_interval),
          distance_(distance), max_epochs_(epochs), logfile_(logfile)
    {
        lat_ = 0.0; lon_ = 0.0;
        epoch_count_ = 0;
        dlat_threshold_ = distance_ * 360.0 / (6371000.0 * 2.0 * M_PI);
        dlon_threshold_ = 0.0;
        lastgga_ = 0;

        if (stats_interval > 0) {
            stats_epochs_.assign(QUALITIES.size(), 0);
            stats_total_ = 0;
            stats_interval_ = stats_interval;
        } else {
            stats_interval_ = 0;
        }
        // prepare NMEA parser handlers
        handlers_[std::regex("^\\$G[A-Z]GGA,")] = [this](const std::string &s){ handle_nmea_gga(s); };
        handlers_[std::regex("^\\$G[A-Z]RMC,")] = [this](const std::string &s){ handle_nmea_rmc(s); };
        parser_ = std::make_unique<NmeaParser>(handlers_);

        // open NMEA log
        std::string ts = now_timestamp();
        nmea_log_filename_ = "solution_log_" + ts + ".nmea";
        nmea_log_.open(nmea_log_filename_);

        if (!mountpoint_.empty()) {
            ntrip_.start_stream(mountpoint_, [this](const std::vector<uint8_t> &data){ handle_ntrip_data(data); });
        } else {
            // get mountpoints until we have data
            std::vector<MountPointInfo> mp;
            while (mp.empty()) {
                mp = ntrip_.get_mountpoints();
                if (mp.empty()) {
                    log_warning("No mountpoints available, retrying in 10s");
                    std::this_thread::sleep_for(std::chrono::seconds(SOCKET_RECONNECT_DELAY));
                } else {
                    mountpoints_ = mp;
                }
            }
            if (mountpoint_.empty()) {
                std::cout << "Available mountpoints:\n";
                for (const auto &p : mountpoints_) {
                    std::cout << std::setw(8) << p.name << ": " << p.format << ", " << p.country
                              << " (" << std::fixed << std::setprecision(3) << p.lat << "," << p.lon << ")\n";
                }
                std::exit(0);
            }
        }
    }

    ~PointPerfectClient() {
        ntrip_.stop_stream();
        if (nmea_log_.is_open()) 
            nmea_log_.close();
    }

    void loop_forever() {
        std::vector<uint8_t> buf;
        buf.reserve(2048);
        try {
            while (true) {
                ssize_t n = serial_.read_some(buf, 1024);
                if (n > 0) {
                    if (ubxfile_ && ubxfile_->is_open()) {
                        ubxfile_->write(reinterpret_cast<const char*>(buf.data()), buf.size());
                        ubxfile_->flush();
                    }
                    // send to parser
                    parser_->parse(buf);
                } else {
                    // no data; small sleep to avoid busy loop
                    std::this_thread::sleep_for(10ms);
                }
            }
        } catch (...) {
            ntrip_.stop_stream();
            if (nmea_log_.is_open()) nmea_log_.close();
        }
    }

private:
    void handle_ntrip_data(const std::vector<uint8_t> &data) {
        // forward to GNSS receiver via serial port
        serial_.write_bytes(data);
    }

    void handle_nmea_rmc(const std::string &sentence) {
        log_info(sentence);
        if (nmea_log_.is_open()) {
            nmea_log_ << sentence << "\n";
            nmea_log_.flush();
        }
    }

    void handle_nmea_gga(const std::string &sentence) {
        log_info(sentence);
        // split by comma
        std::vector<std::string> fields;
        std::istringstream ss(sentence);
        std::string tok;
        while (std::getline(ss, tok, ',')) fields.push_back(tok);
        int quality = 0;
        try { if (fields.size() > 6 && !fields[6].empty()) quality = std::stoi(fields[6]); } catch(...) { quality = 0; }
        double f_lat = 0.0, f_lon = 0.0;
        if (fields.size() > 2 && !fields[2].empty()) try { f_lat = std::stod(fields[2]); } catch(...) {}
        double lat = int(f_lat/100.0) + f_lat - int(f_lat/100.0)*100.0;
        // proper convert: deg = ddmm.mmm -> dd + mm.mmm/60
        lat = int(f_lat/100) + fmod(f_lat, 100.0)/60.0;
        if (fields.size() > 3 && fields[3] == "S") lat *= -1.0;

        if (fields.size() > 4 && !fields[4].empty()) try { f_lon = std::stod(fields[4]); } catch(...) {}
        double lon = int(f_lon/100) + fmod(f_lon, 100.0)/60.0;
        if (fields.size() > 5 && fields[5] == "W") lon *= -1.0;

        if (stats_interval_ > 0) {
            if (quality >=0 && quality < (int)stats_epochs_.size()) stats_epochs_[quality] += 1;
            stats_total_ += 1;
            if (stats_total_ % stats_interval_ == 0) {
                std::ostringstream out;
                for (size_t i=0;i<stats_epochs_.size();++i) {
                    if (stats_epochs_[i]>0) {
                        double pct = (double)stats_epochs_[i] / (double)stats_total_ * 100.0;
                        out << QUALITIES[i] << ": " << std::fixed << std::setprecision(1) << pct << "%, ";
                    }
                }
                log_info(out.str());
            }
        }

        if (quality != 0 && quality != 6) { // 0 = no fix, 6 = estimated (as in Python)
            process_position(lat, lon);
            if (gga_interval_ > 0) {
                auto now = std::chrono::steady_clock::now();
                if (lastgga_time_.time_since_epoch().count() == 0 ||
                    std::chrono::duration_cast<std::chrono::seconds>(now - lastgga_time_).count() > gga_interval_)
                {
                    // send GGA (add CRLF)
                    std::string out = sentence + "\r\n";
                    if (ntrip_.send_gga(out)) {
                        log_debug("GGA sent");
                        lastgga_time_ = now;
                    }
                }
            }
        }
    }

    void process_position(double lat, double lon) {
        epoch_count_ += 1;
        if (std::abs(lat - lat_) > dlat_threshold_ ||
            std::abs(lon - lon_) > dlon_threshold_ ||
            epoch_count_ > max_epochs_)
        {
            log_debug("updating position");
            lat_ = lat; lon_ = lon;
            epoch_count_ = 0;
            // dlon threshold scale by cos(lat)
            dlon_threshold_ = dlat_threshold_ * std::cos(lat_ * M_PI / 180.0);
            MountPointInfo mp = get_mountpoint(lat_, lon_);
            if (!mp.name.empty() && mp.name != mountpoint_) {
                log_info("Switching mountpoint to " + mp.name);
                ntrip_.start_stream(mp.name, [this](const std::vector<uint8_t>& d){ handle_ntrip_data(d); });
                mountpoint_ = mp.name;
            }
        }
    }

    MountPointInfo get_mountpoint(double lat, double lon) {
        if (mountpoints_.empty()) return MountPointInfo{};
        double factor_lon = std::cos(lat * M_PI / 180.0);
        double min_dist = 1e300;
        MountPointInfo nearest;
        for (const auto &m : mountpoints_) {
            double dist = (m.lat - lat)*(m.lat - lat) + ((m.lon - lon)*factor_lon)*((m.lon - lon)*factor_lon);
            if (dist < min_dist) { min_dist = dist; nearest = m; }
        }
        return nearest;
    }

    PosixSerial &serial_;
    NtripClient &ntrip_;
    std::string mountpoint_;
    std::vector<MountPointInfo> mountpoints_;
    std::unique_ptr<NmeaParser> parser_;
    std::map<std::regex, NmeaParser::CallbackFn> handlers_;
    std::ofstream nmea_log_;
    std::string nmea_log_filename_;
    std::ofstream *ubxfile_;

    int gga_interval_;
    int distance_;
    double max_epochs_;

    double lat_, lon_;
    int epoch_count_;
    double dlat_threshold_, dlon_threshold_;
    std::chrono::steady_clock::time_point lastgga_time_;
    int lastgga_;
    // stats
    std::vector<int> stats_epochs_;
    int stats_total_ = 0;
    int stats_interval_ = 0;
};

static volatile std::sig_atomic_t g_stop = 0;
void handle_sigint(int) { g_stop = 1; }

int main(void) {
    signal(SIGINT, handle_sigint);

    //std::string user = ;
    //std::string pass = :
    std::string server = DEFAULT_NTRIP_SERVER;
    int port = DEFAULT_NTRIP_PORT;
    std::string mountpoint = "";
    int ggainterval = 10;
    int distance = 10000;
    double epochs = 1e300;
    std::string logfilename = std::string("pointperfect_log_") + now_timestamp() + ".nmea";
    int stats = 0;

    std::string portname = "/dev/ttyACM0";
    int baud = 115200;

    PosixSerial serial;
    if (!serial.open_port(portname, baud)) {
        log_error("Unable to open serial port. Exiting.");
        return 1;
    }
    log_info("Opened serial port " + portname);

    // open ubx file
    std::ofstream logfile;
    std::ofstream *logptr = nullptr;
    if (!logfilename.empty()) {
        logfile.open(logfilename, std::ios::binary);
        if (!logfile.is_open()) {
            log_warning("Unable to open log file for writing");
        } else { 
            logptr = &logfile; 
            log_info("Writing NMEA to " + logfilename); 
        }
    }

    NtripClient ntrip(server, port, user, pass);

    // create client
    PointPerfectClient client(serial, ntrip, mountpoint, ggainterval, distance, epochs, logptr, stats);
    client.loop_forever();

    if (logfile.is_open()) logfile.close();
    serial.close_port();

    return 0;
}
