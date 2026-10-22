#define _GNU_SOURCE
#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <unordered_map>
#include <vector>

static std::string trim(const std::string &text) {
    const std::string whitespace = " \t\r\n\f\v";
    size_t start = text.find_first_not_of(whitespace);
    if (start == std::string::npos) {
        return "";
    }
    size_t end = text.find_last_not_of(whitespace);
    return text.substr(start, end - start + 1);
}

static std::string strip_nmea_line_ending(const std::string &text) {
    std::string out = text;
    while (!out.empty() && (out.back() == '\n' || out.back() == '\r')) {
        out.pop_back();
    }
    return out;
}

static bool has_valid_nmea_checksum(const std::string &line) {
    std::string text = trim(line);
    if (text.empty() || text[0] != '$') {
        return false;
    }
    size_t star = text.rfind('*');
    if (star == std::string::npos || star + 3 != text.size()) {
        return false;
    }
    unsigned int expected = 0;
    if (std::sscanf(text.c_str() + star + 1, "%x", &expected) != 1) {
        return false;
    }
    unsigned char actual = 0;
    for (size_t i = 1; i < star; ++i) {
        actual ^= static_cast<unsigned char>(text[i]);
    }
    return actual == static_cast<unsigned char>(expected);
}

static std::pair<std::optional<uint64_t>, std::string> parse_gts_line(const std::string &line) {
    std::string text = trim(line);
    if (text.rfind("$GTS,", 0) != 0) {
        return {std::nullopt, std::string()};
    }
    size_t comma = text.find(',', 5);
    if (comma == std::string::npos) {
        return {std::nullopt, std::string()};
    }
    std::string timestamp_text = text.substr(5, comma - 5);
    std::string chunk = text.substr(comma + 1);
    // Keep discarding GTS chunks until we encounter the first chunk whose
    // first non-whitespace character is '$' or '#'. Only then set the
    // `initial_gts_resync_done` flag so subsequent chunks are accepted.
    static bool initial_gts_resync_done = false;
    if (!initial_gts_resync_done) {
        size_t i = 0;
        while (i < chunk.size() && (chunk[i] == ' ' || chunk[i] == '\t' || chunk[i] == '\r' || chunk[i] == '\n')) {
            ++i;
        }
        if (i == chunk.size()) {
            // chunk is all whitespace — discard and keep waiting for first $/#
            return {std::nullopt, std::string()};
        }
        if (chunk[i] != '$' && chunk[i] != '#') {
            // not a valid starting char yet — discard and keep waiting
            return {std::nullopt, std::string()};
        }
        // Found the first valid-starting chunk: strip leading whitespace,
        // accept this chunk and mark resync as done.
        if (i > 0) {
            chunk.erase(0, i);
        }
        initial_gts_resync_done = true;
    }
    if (chunk.empty()) {
        return {std::nullopt, std::string()};
    }
    try {
        uint64_t timestamp = std::stoull(timestamp_text);
        return {timestamp, chunk};
    } catch (const std::exception &) {
        return {std::nullopt, std::string()};
    }
}

static bool find_next_top_level_prefix(const std::string &buffer, size_t start, size_t &pos) {
    size_t imu_pos = buffer.find("$IMU,", start);
    size_t gts_pos = buffer.find("$GTS,", start);
    if (imu_pos == std::string::npos && gts_pos == std::string::npos) {
        return false;
    }
    if (imu_pos == std::string::npos) {
        pos = gts_pos;
    } else if (gts_pos == std::string::npos) {
        pos = imu_pos;
    } else {
        pos = std::min(imu_pos, gts_pos);
    }
    return true;
}

static bool extract_top_level_frame(std::string &buffer, std::string &frame) {
    size_t prefix_pos;
    if (!find_next_top_level_prefix(buffer, 0, prefix_pos)) {
        if (buffer.size() > 4096) {
            size_t last_dollar = buffer.rfind('$');
            if (last_dollar == std::string::npos) {
                buffer.clear();
            } else if (last_dollar > 0) {
                buffer.erase(0, last_dollar);
            }
        }
        return false;
    }
    if (prefix_pos > 0) {
        buffer.erase(0, prefix_pos);
    }
    if (buffer.rfind("$IMU,", 0) == 0) {
        size_t pos = buffer.find('\n', 5);
        if (pos == std::string::npos) {
            return false;
        }
        frame = buffer.substr(0, pos + 1);
        buffer.erase(0, pos + 1);
        return true;
    }
    if (buffer.rfind("$GTS,", 0) == 0) {
        size_t next_prefix_pos;
        if (!find_next_top_level_prefix(buffer, 5, next_prefix_pos)) {
            return false;
        }
        frame = buffer.substr(0, next_prefix_pos);
        buffer.erase(0, next_prefix_pos);
        return true;
    }
    return false;
}

struct ImuRecord {
    uint64_t timestamp_us;
    double acc_x;
    double acc_y;
    double acc_z;
    double gyro_x;
    double gyro_y;
    double gyro_z;
    double q0;
    double q1;
    double q2;
    double q3;
    std::string raw_line;
};

class GtsFrameAssembler {
public:
    explicit GtsFrameAssembler(size_t max_buffers = 64, size_t max_frame_len = 65536)
        : max_buffers_(max_buffers), max_frame_len_(max_frame_len) {}

    static bool buffer_looks_complete(const std::string &buffer) {
        if (buffer.empty()) {
            return false;
        }
        if (buffer.size() >= 2 && buffer.substr(buffer.size() - 2) == "\r\n") {
            return true;
        }
        if (buffer.back() == '\n') {
            return true;
        }
        size_t star = buffer.rfind('*');
        if (star == std::string::npos || star + 3 > buffer.size()) {
            return false;
        }
        for (size_t i = star + 1; i < buffer.size(); ++i) {
            char c = buffer[i];
            if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f'))) {
                return false;
            }
        }
        return true;
    }

    std::vector<std::string> add_chunk(uint64_t timestamp, const std::string &chunk) {
        bool is_new = (buffers_.find(timestamp) == buffers_.end());
        if (is_new) {
            order_.push_back(timestamp);
            if (order_.size() > max_buffers_) {
                uint64_t old = order_.front();
                order_.erase(order_.begin());
                buffers_.erase(old);
            }
        }
        std::string &buffer = buffers_[timestamp];
        buffer += chunk;
        if (buffer.size() > max_frame_len_) {
            buffers_.erase(timestamp);
            order_.erase(std::remove(order_.begin(), order_.end(), timestamp), order_.end());
            return {};
        }

        std::vector<std::string> complete_frames;
        if (buffer_looks_complete(buffer)) {
            std::string out = buffer;
            if (out.size() >= 1 && out.back() != '\n') {
                out += "\r\n";
            } else if (out.size() >= 2 && out.substr(out.size() - 2) != "\r\n") {
                if (out.back() == '\n') {
                    out.insert(out.size() - 1, "\r");
                }
            }
            complete_frames.push_back(out);
            buffers_.erase(timestamp);
            order_.erase(std::remove(order_.begin(), order_.end(), timestamp), order_.end());
            return complete_frames;
        }

        if (is_new && buffers_.size() > 1) {
            std::vector<uint64_t> flushed;
            for (auto it = buffers_.begin(); it != buffers_.end();) {
                if (it->first != timestamp) {
                    if (!it->second.empty()) {
                        std::string out = it->second;
                        if (out.size() >= 1 && out.back() != '\n') {
                            out += "\r\n";
                        }
                        complete_frames.push_back(out);
                    }
                    flushed.push_back(it->first);
                    it = buffers_.erase(it);
                } else {
                    ++it;
                }
            }
            for (uint64_t old : flushed) {
                order_.erase(std::remove(order_.begin(), order_.end(), old), order_.end());
            }
        }
        return complete_frames;
    }

private:
    size_t max_buffers_;
    size_t max_frame_len_;
    std::unordered_map<uint64_t, std::string> buffers_;
    std::vector<uint64_t> order_;
};

static void handle_imu_frame(const std::string &line, std::vector<ImuRecord> &history) {
    std::string text = strip_nmea_line_ending(trim(line));
    if (text.empty() || text.rfind("$IMU", 0) != 0) {
        return;
    }
    std::string payload = text.substr(4);
    if (!payload.empty() && payload[0] == ',') {
        payload.erase(0, 1);
    }
    std::vector<std::string> fields;
    std::stringstream ss(payload);
    std::string field;
    while (std::getline(ss, field, ',')) {
        fields.push_back(field);
    }

#if 0
    std::cout << "[IMU] " << text << std::endl;
    if (fields.size() >= 11) {
        ImuRecord record;
        record.timestamp_us = std::stoull(fields[0]);
        record.acc_x = std::stod(fields[1]);
        record.acc_y = std::stod(fields[2]);
        record.acc_z = std::stod(fields[3]);
        record.gyro_x = std::stod(fields[4]);
        record.gyro_y = std::stod(fields[5]);
        record.gyro_z = std::stod(fields[6]);
        record.q0 = std::stod(fields[7]);
        record.q1 = std::stod(fields[8]);
        record.q2 = std::stod(fields[9]);
        record.q3 = std::stod(fields[10]);
        record.raw_line = text;

        std::cout << "      ts=" << record.timestamp_us
                  << " acc=(" << record.acc_x << ", " << record.acc_y << ", " << record.acc_z << ")"
                  << " gyro=(" << record.gyro_x << ", " << record.gyro_y << ", " << record.gyro_z << ")"
                  << " quat=(" << record.q0 << ", " << record.q1 << ", " << record.q2 << ", " << record.q3 << ")"
                  << std::endl;

        history.push_back(record);
        if (history.size() > 2048) {
            history.erase(history.begin(), history.begin() + (history.size() - 2048));
        }
    } else {
        std::cout << "      invalid IMU frame, fields=" << fields.size() << std::endl;
    }
#endif

}

static bool forward_gnss_frame(int out_fd, const std::string &frame) {
    if (frame.empty() || out_fd < 0) {
        std::cerr << "invalid GNSS frame or output file descriptor" << std::endl;
        return false;
    }
    else
    {
 //       std::cout << "[GNSS] Try to forward frames: " << frame << std::endl;
    }
    
    const char *data = frame.data();
    size_t total = frame.size();
    size_t written = 0;
    while (written < total) {
        ssize_t n = write(out_fd, data + written, total - written);
        if (n > 0) {
            written += static_cast<size_t>(n);
            continue;
        }
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                fd_set wf;
                FD_ZERO(&wf);
                FD_SET(out_fd, &wf);
                timeval tv{};
                tv.tv_sec = 0;
                tv.tv_usec = 200000; // wait up to 200ms
                int r = select(out_fd + 1, nullptr, &wf, nullptr, &tv);
                if (r > 0) {
                    continue; // ready to write
                }
                std::cerr << "write timeout (EAGAIN) when forwarding GNSS: " << strerror(errno) << std::endl;
                return false;
            }
            std::cerr << "write error when forwarding GNSS: " << strerror(errno) << std::endl;
            return false;
        }
    }
    return true;
}

static speed_t baud_constant(int baud) {
    switch (baud) {
        case 1200: return B1200;
        case 2400: return B2400;
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
#ifdef B921600
        case 921600: return B921600;
#endif
#ifdef B1000000
        case 1000000: return B1000000;
#endif
#ifdef B1152000
        case 1152000: return B1152000;
#endif
        default: throw std::runtime_error("unsupported baud rate: " + std::to_string(baud));
    }
}

static void configure_serial(int fd, int baud) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        throw std::runtime_error("tcgetattr failed: " + std::string(strerror(errno)));
    }
    speed_t speed = baud_constant(baud);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        throw std::runtime_error("tcsetattr failed: " + std::string(strerror(errno)));
    }
    tcflush(fd, TCIFLUSH);
}

static int open_serial_port(const std::string &path, int baud) {
    int fd = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        throw std::runtime_error("failed to open serial port " + path + ": " + std::string(strerror(errno)));
    }
    configure_serial(fd, baud);
    return fd;
}

static int open_output_port(const std::string &path, int baud) {
    // Open the output port non-blocking so writes never block the main loop.
    // The writer logic in `forward_gnss_frame` already handles EAGAIN via
    // select(), so opening non-blocking prevents the process from hanging
    // if the remote end isn't currently reading.
    int fd = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        throw std::runtime_error("failed to open output port " + path + ": " + std::string(strerror(errno)));
    }
    configure_serial(fd, baud);
    return fd;
}

static void run_forwarder(int in_fd, int out_fd) {
    std::string buffer;
    buffer.reserve(8192);
    std::vector<ImuRecord> imu_history;
    GtsFrameAssembler assembler;

    while (true) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(in_fd, &readfds);
        timeval tv{};
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        int ready = select(in_fd + 1, &readfds, nullptr, nullptr, &tv);
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            throw std::runtime_error("select failed: " + std::string(strerror(errno)));
        }
        if (ready == 0) {
            continue;
        }

        char read_buf[4096];
        ssize_t n = read(in_fd, read_buf, sizeof(read_buf));
        if (n <= 0) {
            continue;
        }
        buffer.append(read_buf, static_cast<size_t>(n));
        if (buffer.size() > 131072) {
            buffer.erase(0, buffer.size() - 65536);
        }
        while (true) {
            std::string frame;
            if (!extract_top_level_frame(buffer, frame)) {
                break;
            }
            std::string line = strip_nmea_line_ending(frame);
            if (line.empty()) {
                continue;
            }
            if (line.rfind("$IMU", 0) == 0) {
                handle_imu_frame(line, imu_history);
                continue;
            }
            if (line.rfind("$GTS", 0) == 0) {
                auto [timestamp, chunk] = parse_gts_line(line);
                if (!timestamp.has_value()) {
                    std::cerr << "Invalid GTS line: " << line << std::endl;
                    continue;
                }
                else
                {
//                    std::cout << "[GTS] timestamp=" << *timestamp << " chunk=" << chunk << std::endl;
                }
                auto frames = assembler.add_chunk(*timestamp, chunk);
                for (const auto &gnss_frame : frames) {
                    if (forward_gnss_frame(out_fd, gnss_frame)) {
//                        std::cout << "[GNSS] forwarded " << gnss_frame.size() << " bytes from timestamp " << *timestamp << std::endl;
//                        std::cout << "[GNSS] frame=" << gnss_frame << std::endl;
                    } else {
                        std::cerr << "Failed to forward GNSS frame for timestamp " << *timestamp << std::endl;
                    }
                }
                continue;
            }
            // Ignore other top-level lines from the source device.
        }
    }
}

static void print_usage(const char *argv0) {
    std::cout << "Usage: " << argv0 << " [--in-port /dev/ttyS0] [--baud 921600] [--out-port /dev/ttyVUSB0]" << std::endl;
}

int main(int argc, char **argv) {
    std::string in_port = "/dev/ttyS0";
    int baud = 921600;
    std::string out_port = "/dev/ttyVUSB0";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--in-port" && i + 1 < argc) {
            in_port = argv[++i];
        } else if (arg == "--baud" && i + 1 < argc) {
            baud = std::stoi(argv[++i]);
        } else if (arg == "--out-port" && i + 1 < argc) {
            out_port = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            print_usage(argv[0]);
            return 2;
        }
    }

    try {
        int in_fd = open_serial_port(in_port, baud);
        int out_fd = open_output_port(out_port, baud);
        std::cout << "Listening " << in_port << " at " << baud << " baud, forwarding GNSS to " << out_port << std::endl;
        run_forwarder(in_fd, out_fd);
        close(in_fd);
        close(out_fd);
    } catch (const std::exception &exc) {
        std::cerr << "Error: " << exc.what() << std::endl;
        return 1;
    }
    return 0;
}
