#ifndef RM_UTILS_LOGGER_CENTER_HPP_
#define RM_UTILS_LOGGER_CENTER_HPP_

// std
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <ctime>
#include <sstream>
#include <fstream>
#include <memory>
#include <mutex>
#include <thread>
#include <queue>
#include <condition_variable>
#include <unordered_map>
#include <filesystem>
#include <fmt/core.h>

#define WHITE_ "\033[0m"
#define YELLOW_ "\033[33m"
#define RED_ "\033[31m"
#define ORANGE_ "\033[38;5;208m"
#define DARK_RED_ "\033[38;5;88m"

enum class LogType {
    INFO_, ERROR_, WARN_, DEBUG_, FATAL_
};

namespace fs = std::filesystem;

inline std::tm get_time() {
    std::time_t now = std::time(nullptr);
    std::tm tm_;
    localtime_r(&now, &tm_);
    return tm_;
}

inline std::tm tm_now = get_time();

struct Message {
    std::string formatted_msg;
    LogType type;
    std::tm tm_;
};

class Logger {
public:
    Logger() = default;
    Logger(const std::string& node_name_);
    // func
    template<typename... Args>
    void pka_logger(const LogType& type, const std::string& msg, Args&&... args) {
        std::time_t now = std::time(nullptr);
        std::tm tm_;
        localtime_r(&now, &tm_);

        std::string type_str;
        std::string formatted_msg = fmt::format(fmt::runtime(msg), std::forward<Args>(args)...);
        {
            std::lock_guard<std::mutex> lg(this->mtx);  // mutex only play a role in this scope
            this->message_queue.push({formatted_msg, type, tm_});
        }
        this->cond_var.notify_one();
    }
    // vars
    fs::path save_path = fs::path(std::getenv("HOME"));
    std::ofstream main_log_file;
    std::ofstream log_file;
    std::string node_name;
    std::tm tm_;

    ~Logger() {
        {
            std::lock_guard<std::mutex> lg(mtx);
            this->stop_thread = true;
        }
        this->cond_var.notify_one();
        if (log_processor.joinable()) log_processor.join();

        if (this->log_file.is_open()) this->log_file.close();
        if (this->main_log_file.is_open()) this->main_log_file.close();
    }

private:
    // func
    inline std::filesystem::path file_path_construct() {
        std::ostringstream oss;
        oss << std::put_time(&this->tm_, "%Y-%m-%d_%H-%M-%S");
        fs::path log_path = this->save_path / fs::path("pka_logs") / oss.str();

        if (!fs::exists(log_path)) {
            fs::create_directories(log_path);
        }

        return log_path;
    }
    inline std::filesystem::path filename_compose(std::string node_name_input) {
        std::ostringstream oss;
        oss << node_name_input << ".txt";

        fs::path log_file_path = this->file_path_construct() / oss.str();
        return log_file_path;
    }

    void log_process();
    // vars
    std::mutex mtx;
    std::mutex main_logger_mtx;
    std::queue<Message> message_queue;
    std::condition_variable cond_var;
    std::thread log_processor;
    bool stop_thread = false;
};

class LoggerCenter {
public:
    LoggerCenter() = default;
    // func
    static Logger& find_logger(std::string&& node_name);
    // vars
    static inline std::unordered_map<std::string, std::unique_ptr<Logger>> logger_pool;
    static inline std::mutex mtx;
};

#define PKA_ASSERT(judge)                                           \
    do {                                                            \
        if(!(judge)) {                                              \
            std::ostringstream oss;                                 \
            oss << "Assertion failed: [" << #judge << "]";          \
            std::cerr << oss.str() << std::endl;                    \
            std::abort();                                           \
        }                                                           \
    } while(0)

#define PKA_ASSERT_MSG(judge, msg)                                  \
    do {                                                            \
        if(!(judge)) {                                              \
            std::ostringstream oss;                                 \
            oss << "Assertion failed: [" << #judge << "], " << msg; \
            std::cerr << oss.str() << std::endl;                    \
            std::abort();                                           \
        }                                                           \
    } while(0)

#define PKA_INFO(node, ...) LoggerCenter::find_logger(node).pka_logger(LogType::INFO_, ##__VA_ARGS__)
#define PKA_ERROR(node, ...) LoggerCenter::find_logger(node).pka_logger(LogType::ERROR_, ##__VA_ARGS__)
#define PKA_WARN(node, ...) LoggerCenter::find_logger(node).pka_logger(LogType::WARN_, ##__VA_ARGS__)
#define PKA_DEBUG(node, ...) LoggerCenter::find_logger(node).pka_logger(LogType::DEBUG_, ##__VA_ARGS__)
#define PKA_FATAL(node, ...) LoggerCenter::find_logger(node).pka_logger(LogType::FATAL_, ##__VA_ARGS__)

#endif
