/*
 * Log.h
 *
 *  Created on: Sep 4, 2022
 *      Author: francois
 */

#ifndef LOG_H_
#define LOG_H_

#include <stdio.h>
#include <mutex>
#include <ctime>

#include "config.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"

enum LogPriority
{
	DebugPriority, TracePriority, InfoPriority, WarnPriority, ErrorPriority, CriticalPriority
};

class Logger
{
private:
	LogPriority priority = LOG_DEFAULT_LEVEL;
	std::mutex log_mutex;
	const char* filepath = 0;
	FILE* file = 0;

public:
	static void SetPriority(LogPriority new_priority)
	{
		get_instance().priority = new_priority;
	}

	static void EnableFileOutput()
	{
		Logger& logger_instance = get_instance();
		logger_instance.filepath = LOG_DEFAULT_FILEPATH;
		logger_instance.enable_file_output();
	}

	static void EnableFileOutput(const char* new_filepath)
	{
		Logger& logger_instance = get_instance();
		logger_instance.filepath = new_filepath;
		logger_instance.enable_file_output();
	}

	template<typename... Args>
	static void Trace(const char* message, Args... args)
	{
		get_instance().log("\033[0;35m[Trace   ]\033[0;0m", TracePriority, message, args...);
	}

	template<typename... Args>
	static void Debug(const char* message, Args... args)
	{
		get_instance().log("\033[0;32m[Debug   ]\033[0;0m", DebugPriority, message, args...);
	}

	template<typename... Args>
	static void Info(const char* message, Args... args)
	{
		get_instance().log("\033[0;34m[Info    ]\033[0;0m", InfoPriority, message, args...);
	}

	template<typename... Args>
	static void Warn(const char* message, Args... args)
	{
		get_instance().log("\033[0;33m[Warn    ]\033[0;0m", WarnPriority, message, args...);
	}

	template<typename... Args>
	static void Error(const char* message, Args... args)
	{
		get_instance().log("\033[0;31m[Error   ]\033[0;0m", ErrorPriority, message, args...);
	}

	template<typename... Args>
	static void Critical(const char* message, Args... args)
	{
		get_instance().log("\033[0;41m[Critical]\033[0;0m", CriticalPriority, message, args...);
	}



	template<typename... Args>
	static void Trace(int line, const char* source_file, const char* message, Args... args)
	{
		get_instance().log(line, source_file, "\033[0;35m[Trace   ]\033[0;0m", TracePriority, message, args...);
	}

	template<typename... Args>
	static void Debug(int line, const char* source_file, const char* message, Args... args)
	{
		get_instance().log(line, source_file, "\033[0;32m[Debug   ]\033[0;0m", DebugPriority, message, args...);
	}

	template<typename... Args>
	static void Info(int line, const char* source_file, const char* message, Args... args)
	{
		get_instance().log(line, source_file, "\033[0;34m[Info    ]\033[0;0m", InfoPriority, message, args...);
	}

	template<typename... Args>
	static void Warn(int line, const char* source_file, const char* message, Args... args)
	{
		get_instance().log(line, source_file, "\033[0;33m[Warn    ]\033[0;0m", WarnPriority, message, args...);
	}

	template<typename... Args>
	static void Error(int line, const char* source_file, const char* message, Args... args)
	{
		get_instance().log(line, source_file, "\033[0;31m[Error   ]\033[0;0m", ErrorPriority, message, args...);
	}

	template<typename... Args>
	static void Critical(int line, const char* source_file, const char* message, Args... args)
	{
		get_instance().log(line, source_file, "\033[0;41m[Critical]\033[0;0m", CriticalPriority, message, args...);
	}

private:
	Logger() {}

	Logger(const Logger&) = delete;
	Logger& operator= (const Logger&) = delete;

	~Logger()
	{
		free_file();
		log_mutex.unlock();
	}

	static Logger& get_instance()
	{
		static Logger logger;
		return logger;
	}

	template<typename... Args>
	void log(const char* message_priority_str, LogPriority message_priority, const char* message, Args... args)
	{
		if (priority <= message_priority)
		{
			std::time_t current_time = std::time(0);
			std::tm* timestamp = std::localtime(&current_time);
			char buffer[80];
			strftime(buffer, 80, "%X", timestamp);

			log_mutex.lock();
			printf("%s", buffer);
			printf("%s", message_priority_str);
			printf(message, args...);
			printf("\n");

			if (file && LOG_FILE_ENABLE)
			{
				fprintf(file, "%s", buffer);
				fprintf(file, "%s", message_priority_str);
				fprintf(file, message, args...);
				fprintf(file, "\n");
			}
			log_mutex.unlock();
		}
	}

	template<typename... Args>
	void log(int line_number, const char* source_file, const char* message_priority_str, LogPriority message_priority, const char* message, Args... args)
	{
		if (priority <= message_priority)
		{
			std::time_t current_time = std::time(0);
			std::tm* timestamp = std::localtime(&current_time);
			char buffer[80];
			strftime(buffer, 80, "%X", timestamp);

			log_mutex.lock();
			printf("%s", buffer);
			printf("%s", message_priority_str);
			printf("[%s line %d]   ",source_file, line_number);
			printf(message, args...);
			printf("\n");

			if (file)
			{
				fprintf(file, "%s", buffer);
				fprintf(file, "%s", message_priority_str);
				fprintf(file, "[%s line %d]   ",source_file, line_number);
				fprintf(file, message, args...);
				fprintf(file, "\n");
			}
			log_mutex.unlock();
		}
	}

	bool enable_file_output()
	{
		free_file();

		file = std::fopen(filepath, "w+");

		if (file == 0)
		{
			return false;
		}

		return true;
	}

	void free_file()
	{
		if (file)
		{
			fclose(file);
			file = 0;
		}
	}
};


#define LOG_TRACE(Message) (Logger::Trace(__LINE__, __FILE__, Message))
#define LOG_DEBUG(Message) (Logger::Debug(__LINE__, __FILE__, Message))
#define LOG_INFO(Message) (Logger::Info(__LINE__, __FILE__, Message))
#define LOG_WARN(Message) (Logger::Warn(__LINE__, __FILE__, Message))
#define LOG_ERROR(Message) (Logger::Error(__LINE__, __FILE__, Message))
#define LOG_CRITICAL(Message) (Logger::Critical(__LINE__, __FILE__, Message))

#define LOGF_TRACE(Message, ...) (Logger::Trace(__LINE__, __FILE__, Message, __VA_ARGS__))
#define LOGF_DEBUG(Message, ...) (Logger::Debug(__LINE__, __FILE__, Message, __VA_ARGS__))
#define LOGF_INFO(Message, ...) (Logger::Info(__LINE__, __FILE__, Message, __VA_ARGS__))
#define LOGF_WARN(Message, ...) (Logger::Warn(__LINE__, __FILE__, Message, __VA_ARGS__))
#define LOGF_ERROR(Message, ...) (Logger::Error(__LINE__, __FILE__, Message, __VA_ARGS__))
#define LOGF_CRITICAL(Message, ...) (Logger::Critical(__LINE__, __FILE__, Message, __VA_ARGS__))

#pragma GCC diagnostic pop

#endif /* LOG_H_ */
