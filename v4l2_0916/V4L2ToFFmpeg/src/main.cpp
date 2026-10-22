#include "v4l2_capture.h"
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>

volatile sig_atomic_t keep_running = 1;

void signal_handler(int dummy) {
    keep_running = 0;
}

//   ./v4l2_to_ffmpeg /dev/video_before_isp /dev/video_after_isp /tmp/jpegs 1280 720 60
int main(int argc, char* argv[]) {
    if (argc != 7) {
        std::cout << "Usage: " << argv[0] << " <timestamp_device> <capture_device> <output_dir> <width> <height> <fps>" << std::endl;
        std::cout << "Example: " << argv[0] << " /dev/video0 /dev/video11 /tmp/jpegs 3840 2160 30" << std::endl;
        return -1;
    }

    std::string timestamp_device_path = argv[1];
    std::string capture_device_path = argv[2];
    std::string output_dir = argv[3];
    int width = atoi(argv[4]);
    int height = atoi(argv[5]);
    int fps = atoi(argv[6]);

    std::cout << "Timestamp device: " << timestamp_device_path << std::endl;
    std::cout << "Capture device: " << capture_device_path << std::endl;
    std::cout << "Output dir: " << output_dir << std::endl;
    std::cout << "Resolution: " << width << "x" << height << "@" << fps << "fps" << std::endl;

    signal(SIGINT, signal_handler);

    V4L2Capture capture(timestamp_device_path, capture_device_path);

    if (!capture.Initialize(width, height, fps)) {
        std::cerr << "Failed to initialize V4L2 capture device" << std::endl;
        return -1;
    }

    if (!capture.StartCapture(output_dir)) {
        std::cerr << "Failed to start capture" << std::endl;
        return -1;
    }

    std::cout << "Capturing... Press Ctrl+C to stop." << std::endl;

    while (keep_running) {
        // 主循环等待信号中断
        usleep(100000); // 休眠100ms
    }

    std::cout << "Stopping capture..." << std::endl;
    capture.StopCapture();

    return 0;
}