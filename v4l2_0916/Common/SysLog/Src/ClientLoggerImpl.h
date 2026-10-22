#pragma once

#include <string>
#include <mutex>

namespace log4cplus
{
	typedef int LogLevel;
	class Logger;
}

class ClientLoggerImpl
{
public:
	ClientLoggerImpl();
	~ClientLoggerImpl();

	void SetLogFilePath(const char* path);
	void SetLogFileName(const char* logFileName);
	void Init();

	void log(log4cplus::LogLevel level, const std::string& message);

private:
	void CreateDefaultConfig(const std::string& strConfigFile);
	std::string GetEnvPath(const std::string& varname);

	int LogInit(const std::string& propertyfilename);
	void LogInit(log4cplus::LogLevel level);

private:
	static log4cplus::Logger* logger_;
	std::string configPath_;
	std::string configFileName_;
	std::string configPropertyName_;
	bool init_;
	std::mutex lock_;

};
