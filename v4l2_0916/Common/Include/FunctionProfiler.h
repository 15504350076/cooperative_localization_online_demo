#pragma once

#include <chrono>
#include <map>
#include <string>
#include <mutex>
#include <iostream>
#include <iomanip>

/**
 * 函数性能分析器
 * 使用方法：
 * 在函数开始处添加: PROFILE_FUNCTION_START(func_name)
 * 在函数结束前添加: PROFILE_FUNCTION_END(func_name)
 * 当函数被调用100次时，会自动打印平均执行频率
 */

namespace {
    struct ProfileData {
        int call_count;
        std::chrono::steady_clock::time_point start_time;
        double total_duration;
        mutable std::mutex mtx;

        ProfileData() : call_count(0), start_time(std::chrono::steady_clock::now()), total_duration(0.0) {}
    };

    std::map<std::string, ProfileData> profile_data_map;
    std::mutex global_mtx;
}

class FunctionProfiler {
public:
    static void start_timer(const std::string& func_name) {
        std::lock_guard<std::mutex> lock(global_mtx);
        auto& data = profile_data_map[func_name];
        std::lock_guard<std::mutex> data_lock(data.mtx);
        data.start_time = std::chrono::steady_clock::now();
    }

    static void record_call(const std::string& func_name) {
        std::lock_guard<std::mutex> lock(global_mtx);
        auto& data = profile_data_map[func_name];
        std::lock_guard<std::mutex> data_lock(data.mtx);
        
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration<double>(end_time - data.start_time).count();
        data.total_duration += duration;
        data.call_count++;
        
        // 每100次调用打印一次统计信息
        if (data.call_count % 100 == 0) {
            double avg_time = data.total_duration / data.call_count;
            double calls_per_second = 1.0 / avg_time;
            
            std::cout << "[PERFORMANCE] Function '" << func_name << "' - Last 100 calls: Avg Time=" 
                      << std::fixed << std::setprecision(6) << avg_time << "s, Freq=" 
                      << std::fixed << std::setprecision(2) << calls_per_second << " calls/sec" << std::endl;
            // 重置统计数据
            data.total_duration = 0.0;
            data.call_count = 0;
        }
    }
};

// 定义宏
#define PROFILE_FUNCTION_START(func_name) \
    FunctionProfiler::start_timer(#func_name)

#define PROFILE_FUNCTION_END(func_name) \
    FunctionProfiler::record_call(#func_name)

