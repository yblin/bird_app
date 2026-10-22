//
// Copyright 2016-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_BASE_LOG_H_
#define CODELIBRARY_BASE_LOG_H_

// We allways include the following headers regardless of whether glog is
// present or not.
#include "codelibrary/base/ansi.h"
#include "codelibrary/base/message.h"

#if __has_include("glog/logging.h") && !defined(USE_CODELIBRARY_LOG)
#define __WITH_GLOG__
#include <glog/logging.h>
#else

#ifdef _MSC_VER
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif // _CRT_SECURE_NO_WARNINGS
#endif // _MSC_VER

#include <algorithm>
#include <chrono>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

#ifdef _WIN32
#include <process.h>
#else
#include <unistd.h>
#endif

#include "codelibrary/base/format.h"
#include "codelibrary/base/equal.h"

#ifdef ERROR
#undef ERROR
#endif

namespace cl {

/**
 * Simple logging framework in a single file.
 *
 * Example Usage:
 *
 *    LOG_ON(INFO);
 *    LOG(INFO) << "This is test info log";
 */
class Logger {
    Logger() {
        support_ansi_ = ansi::SupportAnsiEscapeCode();
    }

public:
    Logger(const Logger&) = delete;

    Logger& operator=(const Logger&) = delete;

    // The severity level for logger.
    enum Severity {
        LOG_NONE    = 0,
        LOG_FATAL   = 1,
        LOG_ERROR   = 2,
        LOG_WARNING = 3,
        LOG_INFO    = 4,
        LOG_DEBUG   = 5,
        LOG_VERBOSE = 6
    };

    /**
     * Store all log data.
     *
     * A record has the severity of logging, the position where the file and
     * line call the log.
     */
    struct Record {
        Record(const Severity& s, const char* f, int l)
            : severity(s), filename(f), line(l) {
            size_t t1 = filename.find_last_of('\\');
            size_t t2 = filename.find_last_of('/');
            if (t1 == std::string::npos) t1 = 0;
            if (t2 == std::string::npos) t2 = 0;
            if (t1 != 0 || t2 != 0)
                filename = filename.substr(std::max(t1, t2) + 1);

            time = std::chrono::system_clock::now();
        }

        Record(const Record&) = delete;

        Record& operator =(const Record&) = delete;

        // A semantic trick to enable message streaming.
        Record& operator +=(const Message& m) {
            message << m;

            return *this;
        }

        // Severity level for logging.
        const Severity severity;

        // Source code filename.
        std::string filename;

        // Source code line.
        const int line;

        // Message of record.
        Message message;

        // Record the time when LOG() took place.
        std::chrono::time_point<std::chrono::system_clock> time;
    };

    /**
     * A destination for the records that pass the severity filter.
     *
     * Setting a sink replaces the console output, so a host application can
     * route records into its own logging system. The log files are written
     * either way. A sink must not throw.
     */
    using Sink = void (*)(const Record& record, void* context) noexcept;

    /**
     * Get the singleton Logger object.
     *
     * The first time this method is called, a Logger object is constructed and
     * returned.
     * Consecutive calls will return the same object.
     */
    static Logger* GetInstance() {
        static Logger instance;
        return &instance;
    }

    /**
     * A semantic trick to enable logging streaming.
     */
    void operator +=(const Record& record) {
        Sink sink = nullptr;
        void* sink_context = nullptr;
        {
        std::lock_guard<std::mutex> guard(mutex_);
        sink = sink_;
        sink_context = sink_context_;

        // Print severity.
        std::string format = SeverityToString(record.severity);

        // Print date time.
        std::time_t tt = std::chrono::system_clock::to_time_t(record.time);
        std::tm tm = LocalTime(tt);
        format += Format("{:%m%d %H:%M:%S}", tm);

        auto since_epoch = record.time.time_since_epoch();
        std::chrono::seconds s =
                std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
        since_epoch -= s;

        // Print milliseconds.
        using Milliseconds = std::chrono::milliseconds;
        Milliseconds milliseconds =
                std::chrono::duration_cast<Milliseconds>(since_epoch);
        format += Format(".{:03}", milliseconds.count());

        // Print message.
        if (!record.message.IsMultiLine()) {
            format += Format(" {}:{}] {}\n", record.filename, record.line,
                             record.message.text());
        } else {
            format += Format(" {}:{}]\n", record.filename, record.line);
            format += Format("{}\n", record.message.text());
        }

        // Write to log files.
        WriteRecord(record.severity, format);

        // Print to console, unless a sink took it over.
        if (!sink) {
            PrintRecord(record.severity, format);
            fflush(stdout);
        }
        }

        if (sink) sink(record, sink_context);

        if (record.severity == LOG_FATAL) exit(1);
    }

    /**
     * Set the log sink, or nullptr to restore the console output.
     *
     * The sink receives the record itself rather than a formatted line, so the
     * host application can format it the way it formats its own logs. It is
     * called without holding the logger mutex, so a sink may itself log.
     */
    void set_sink(Sink sink, void* context = nullptr) {
        std::lock_guard<std::mutex> guard(mutex_);

        sink_ = sink;
        sink_context_ = context;
    }

    /**
     * Check if severity is valid.
     */
    bool CheckSeverity(const Severity& severity) const {
        return severity <= severity_level_;
    }

    void set_severity_level(const Severity& severity_level) {
        severity_level_ = severity_level;
    }

    const Severity& severity_level() const {
        return severity_level_;
    }

    /**
     * Output all log records to a single file.
     *
     * All severity levels are written to the specified file. The file is
     * truncated when it is opened. This overrides the per-severity log
     * directory mode if it was set before.
     */
    void set_log_file(const std::string& filename) {
        std::lock_guard<std::mutex> guard(mutex_);

        log_directory_.clear();
        severity_streams_.clear();
        file_stream_.close();

        file_stream_.open(filename, std::ios::out | std::ios::trunc);
        if (!file_stream_.is_open()) {
            fprintf(stderr, "Failed to open log file: %s\n",
                    filename.c_str());
        }
    }

    /**
     * Output log records to a directory, with one file for each severity
     * level.
     *
     * The name of each log file is <base>.<SEVERITY>.<TIMESTAMP>.<PID>.log.
     * The directory will not be created automatically. This overrides the
     * single log file mode if it was set before.
     */
    void set_log_directory(const std::string& directory,
                           const std::string& base = "app") {
        std::lock_guard<std::mutex> guard(mutex_);

        file_stream_.close();

        log_directory_ = directory;
        log_base_ = base;
        severity_streams_.clear();
    }

    /**
     * Close the log files and stop the file output.
     */
    void close_log_file() {
        std::lock_guard<std::mutex> guard(mutex_);

        file_stream_.close();
        severity_streams_.clear();
        log_directory_.clear();
    }

private:
    /**
     * Convert severity to string.
     */
    const char* SeverityToString(Severity severity) const {
        switch (severity) {
        case LOG_FATAL:
            return "F";
        case LOG_ERROR:
            return "E";
        case LOG_WARNING:
            return "W";
        case LOG_INFO:
            return "I";
        case LOG_DEBUG:
            return "D";
        case LOG_VERBOSE:
            return "V";
        default:
            return "N";
        }
    }

    /**
     * Get severity color.
     */
    constexpr const char* GetSeverityColor(Severity severity) const {
        switch (severity) {
        case LOG_FATAL:
            return ansi::RED;
        case LOG_ERROR:
            return ansi::ORANGE_RED;
        case LOG_WARNING:
            return ansi::YELLOW;
        case LOG_DEBUG:
            return ansi::MAGENTA;
        case LOG_VERBOSE:
            return ansi::CYAN;
        default:
            return ansi::WHITE;
        }
    }

    /**
     * Print record.
     */
    template <typename... Args>
    void PrintRecord(Severity severity, const std::string& format) const {
        if (!CanWriteToStdout()) return;

        if (severity == LOG_INFO || !support_ansi_) {
            Print(format);
        } else {
            Print("{}{}{}", GetSeverityColor(severity), format, ansi::RESET);
        }
    }

    /**
     * Write the record to the log files.
     *
     * The record is written to the single log file in the file mode, or to
     * the per-severity log file in the directory mode. The format does not
     * contain any ANSI escape code.
     */
    void WriteRecord(Severity severity, const std::string& format) {
        if (file_stream_.is_open()) {
            file_stream_ << format;
            file_stream_.flush();
            return;
        }

        if (!log_directory_.empty()) {
            std::ofstream& stream = GetSeverityStream(severity);
            if (stream.is_open()) {
                stream << format;
                stream.flush();
            }
        }
    }

    /**
     * Get the output stream of the given severity.
     *
     * The corresponding log file is created lazily at the first time it is
     * written.
     */
    std::ofstream& GetSeverityStream(Severity severity) {
        if (severity_streams_.empty()) {
            severity_streams_.resize(LOG_VERBOSE + 1);
        }

        std::ofstream& stream = severity_streams_[severity];
        if (!stream.is_open()) {
            std::string filename = Format("{}/{}.{}.{}.{}.log",
                                          log_directory_, log_base_,
                                          SeverityToFileName(severity),
                                          LogFileNameTime(), GetProcessId());
            stream.open(filename, std::ios::out | std::ios::trunc);
            if (!stream.is_open()) {
                fprintf(stderr, "Failed to open log file: %s\n",
                        filename.c_str());
            }
        }
        return stream;
    }

    /**
     * Convert the severity to the file name part.
     */
    static const char* SeverityToFileName(Severity severity) {
        switch (severity) {
        case LOG_FATAL:
            return "FATAL";
        case LOG_ERROR:
            return "ERROR";
        case LOG_WARNING:
            return "WARNING";
        case LOG_INFO:
            return "INFO";
        case LOG_DEBUG:
            return "DEBUG";
        case LOG_VERBOSE:
            return "VERBOSE";
        default:
            return "NONE";
        }
    }

    /**
     * Return the time string used in the log file name.
     */
    static std::string LogFileNameTime() {
        std::time_t tt = std::chrono::system_clock::to_time_t(
                std::chrono::system_clock::now());
        std::tm tm = LocalTime(tt);
        return Format("{:04}{:02}{:02}-{:02}{:02}{:02}",
                      tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                      tm.tm_hour, tm.tm_min, tm.tm_sec);
    }

    /**
     * Return the ID of the current process.
     */
    static int GetProcessId() {
#ifdef _WIN32
        return _getpid();
#else
        return getpid();
#endif
    }

    /**
     * Check whether the standard output can be written.
     *
     * A GUI application may have no console at all. In that case the stdout
     * handle is invalid or absent and any write to it fails; this is detected
     * here before the print is attempted, so the console print is skipped.
     */
    static bool CanWriteToStdout() {
#ifdef _WIN32
        HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
        if (h == INVALID_HANDLE_VALUE || h == nullptr) return false;

        DWORD mode = 0;
        if (GetConsoleMode(h, &mode) != 0) return true;

        // The stdout is redirected to a file or a pipe: writing is safe.
        return GetFileType(h) != FILE_TYPE_UNKNOWN;
#else
        return true;
#endif
    }

    /**
     * The logger severity upper limit.
     *
     * All log messages have its own severity and if it is higher than the limit
     * those messages are dropped.
     */
    Severity severity_level_ = LOG_FATAL;

    // Support ANSI or not.
    bool support_ansi_ = false;

    // The single log file for the file mode.
    std::ofstream file_stream_;

    // The per-severity log files for the directory mode.
    std::vector<std::ofstream> severity_streams_;

    // The directory of the per-severity log files.
    std::string log_directory_;

    // The base name of the per-severity log files.
    std::string log_base_ = "app";

    // The sink that replaces the console output, with its opaque context.
    Sink sink_ = nullptr;
    void* sink_context_ = nullptr;

    std::mutex mutex_;
};

} // namespace cl

// Enable the namespace for log severity.
#define LOG_SEVERITY(severity) cl::Logger::LOG_##severity

/**
 * Main logging macros.
 *
 * Example usage:
 *
 *   LOG(INFO) << "This is a info log";
 */
#ifdef LOG
#undef LOG
#endif
#define LOG(severity) \
    !(cl::Logger::GetInstance()->CheckSeverity(LOG_SEVERITY(severity))) ? \
        (void) 0 : (*cl::Logger::GetInstance()) += \
            cl::Logger::Record(LOG_SEVERITY(severity), __FILE__, __LINE__) += \
            cl::Message() << ""

#ifdef LOG_IF
#undef LOG_IF
#endif
#define LOG_IF(severity, expression) \
    !(expression) ? (void) 0 : LOG(severity)

/**
 * Turn on the log and set the severity level.
 *
 * Example usage:
 *
 *    LOG_ON(INFO);
 */
#define LOG_ON(severity) \
    cl::Logger::GetInstance()->set_severity_level(LOG_SEVERITY(severity))

/**
 * CHECK dies with a fatal error if condition is not true.
 */
#ifdef CHECK
#undef CHECK
#endif
#define CHECK(condition) \
    LOG_IF(FATAL, !(condition)) << "Check failed: " #condition ". "

/**
 * Equality check.
 */
#define CHECK_EQ(actual, expected)              \
    LOG_IF(FATAL, !cl::Equal(actual, expected)) \
        << "CHECK_"                             \
        << cl::CheckEQFailureMessage(#actual, #expected, actual, expected)

/**
 * Near equality check.
 */
#define CHECK_EQ_NEAR(a, b, absolute_range)                          \
    LOG_IF(FATAL, !(cl::Equal(a, b, absolute_range)))                \
        << "CHECK_"                                                  \
        << cl::CheckEQNearFailureMessage(#a, #b, #absolute_range, a, \
                                         b, absolute_range)

/**
 * Range equality check.
 */
#define CHECK_EQ_RANGE(actual_first, actual_last,              \
                       expected_first, expected_last)          \
    LOG_IF(FATAL, !(cl::Equal(actual_first, actual_last,       \
                              expected_first, expected_last))) \
         << "CHECK_"                                           \
         << cl::CheckEQRangeFailureMessage(#actual_first,      \
                                           #actual_last,       \
                                           #expected_first,    \
                                           #expected_last,     \
                                           actual_first,       \
                                           actual_last,        \
                                           expected_first,     \
                                           expected_last)

#endif // __has_include("glog/logging.h")

#endif // CODELIBRARY_BASE_LOG_H_
