#include <rm_utils/pkaLoggerCenter.hpp>

Logger::Logger(const std::string& node_name_) : node_name(node_name_), tm_(tm_now) {
    this->log_file.open(this->filename_compose(this->node_name).string(), std::ios::out | std::ios::app);
    if (!this->log_file.is_open()) {
        std::cerr << "Cannot open the log file." << std::endl;
    }
    this->main_log_file.open(this->filename_compose(std::string("main")).string(), std::ios::out | std::ios::app);
    if (!this->main_log_file.is_open()) {
        std::cerr << "Cannot open the main log file." << std::endl;
    }

    this->log_processor = std::thread(&Logger::log_process, this);
}

void Logger::log_process() {
    while (true) {
        std::unique_lock<std::mutex> ul(this->mtx);
        this->cond_var.wait(ul, [this]() {
            return !this->message_queue.empty() || this->stop_thread;
        });

        while (!this->message_queue.empty()) {
            Message msg_ = this->message_queue.front();
            this->message_queue.pop();
            ul.unlock();

            std::ostringstream oss_term, oss_file;
            std::string type_str, color;

            switch (msg_.type)
            {
                case LogType::INFO_:
                    type_str = "INFO";
                    color = WHITE_;
                    break;
                case LogType::DEBUG_:
                    type_str = "DEBUG";
                    color = ORANGE_;
                    break;
                case LogType::ERROR_:
                    type_str = "ERROR";
                    color = RED_;
                    break;
                case LogType::WARN_:
                    type_str = "WARN";
                    color = YELLOW_;
                    break;
                case LogType::FATAL_:
                    type_str = "FATAL";
                    color = DARK_RED_;
                    break;
                default:
                    break;
            }

            oss_term << color << "[" << type_str << "] "
                     << "[" << std::put_time(&msg_.tm_, "%Y-%m-%d %H:%M:%S") << "] "
                     << "[" << this->node_name << "] "
                     << msg_.formatted_msg << WHITE_;
            std::cout << oss_term.str() << std::endl;

            oss_file << "[" << type_str << "] "
                     << "[" << std::put_time(&msg_.tm_, "%Y-%m-%d %H:%M:%S") << "] "
                     << "[" << this->node_name << "] "
                     << msg_.formatted_msg;
            
            this->log_file << oss_file.str() << std::endl;
            {
                std::lock_guard<std::mutex> lg_(this->main_logger_mtx);
                this->main_log_file << oss_file.str() << std::endl;
            }
            ul.lock();
        }
        if (this->stop_thread && this->message_queue.empty()) {
            break;
        }
    }
}

Logger& LoggerCenter::find_logger(std::string&& node_name) {
    std::lock_guard<std::mutex> lg(mtx);

    auto iterator = logger_pool.find(node_name);
    if (iterator == logger_pool.end()) {
        logger_pool[node_name] = std::make_unique<Logger>(node_name);
    }

    return *logger_pool[node_name];
}
