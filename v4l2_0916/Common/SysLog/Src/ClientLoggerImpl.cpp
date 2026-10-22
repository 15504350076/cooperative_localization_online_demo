#include "ClientLoggerImpl.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>

#include "log4cplus/logger.h"
#include "log4cplus/fileappender.h"
#include "log4cplus/layout.h"
#include "log4cplus/ndc.h"
#include "log4cplus/helpers/loglog.h"
#include "log4cplus/helpers/property.h"
#include "log4cplus/loggingmacros.h"
#include "log4cplus/initializer.h"
#include "log4cplus/configurator.h"
#include "log4cplus/consoleappender.h"
#include "log4cplus/tchar.h"
#include <filesystem>

log4cplus::Logger* ClientLoggerImpl::logger_ = nullptr;

ClientLoggerImpl::ClientLoggerImpl()
	: init_(false)
{
	configFileName_ = "aibox.log";
	configPropertyName_ = "aibox.log_conf.properties";
}
ClientLoggerImpl::~ClientLoggerImpl()
{
	//log4cplus::Logger::shutdown();
	log4cplus::deinitialize();
}

void ClientLoggerImpl::SetLogFilePath(const char* path)
{
	if (path)
	{
		configPath_ = path;
		if (!configPath_.empty() && configPath_.substr(configPath_.size() - 1) != "/")
			configPath_ = configPath_ + "/";
	}
}
void ClientLoggerImpl::SetLogFileName(const char* logFileName)
{
	if (logFileName)
	{
		configFileName_ = logFileName;
		configPropertyName_ = configFileName_ + "_conf.properties";
	}
}

void ClientLoggerImpl::Init()
{
	if (!init_)
	{
		log4cplus::initialize();
		LogInit(configPath_ + configPropertyName_);
		init_ = true;
	}
}

std::string ClientLoggerImpl::GetEnvPath(const std::string& varname)
{
	std::string strVarDir = "";
#if defined(__WIN32__) || defined(_WIN32)
	size_t requiredSize;
	getenv_s(&requiredSize, NULL, 0, varname.c_str());
	if (requiredSize == 0)
		return strVarDir;
	char* libvar = new char[requiredSize];
	if (!libvar)
		return strVarDir;
	getenv_s(&requiredSize, libvar, requiredSize, varname.c_str());
	strVarDir = std::string(libvar);
	delete[] libvar;
#else
	char* libvar = secure_getenv(varname.c_str());
	if (libvar)
		strVarDir = std::string(libvar);
#endif
	return strVarDir;
}


int ClientLoggerImpl::LogInit(const std::string& propertyfilename)
{
	try {
		struct stat s;
		if (stat(propertyfilename.c_str(), &s) != 0) //create defalut config file
		{
			CreateDefaultConfig(propertyfilename);
		}
		log4cplus::PropertyConfigurator logconfig(LOG4CPLUS_TEXT(propertyfilename));
		logconfig.configure();

		static log4cplus::Logger logInstance = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("AIBOX"));
		logger_ = &logInstance;
	}
	catch (...) {
		return -1;
	}
	return 0;
}
void ClientLoggerImpl::CreateDefaultConfig(const std::string& strConfigFile)
{
	if (!configPath_.empty())
	{
		if (!std::filesystem::exists(configPath_))
			std::filesystem::create_directory(configPath_);
	}
	std::ofstream ofile;
	ofile.open(strConfigFile.c_str());
	if (ofile)
	{
		std::map<std::string, std::string> configKeyValueMap;
		std::string pattern = "%D{%m/%d/%y %H:%M:%S,%Q} [%t] %-5p %c - %m %n";
		configKeyValueMap["log4cplus.logger.AIBOX"] = "DEBUG, STDOUT, RF";

		configKeyValueMap["log4cplus.appender.STDOUT"] = "log4cplus::ConsoleAppender";
		configKeyValueMap["log4cplus.appender.STDOUT.layout"] = "log4cplus::PatternLayout";
		configKeyValueMap["log4cplus.appender.STDOUT.layout.ConversionPattern"] = pattern;
		configKeyValueMap["log4cplus.appender.STDOUT.filters.1"]="log4cplus::spi::LogLevelRangeFilter";
		configKeyValueMap["log4cplus.appender.STDOUT.filters.1.LogLevelMin"]="INFO";
		configKeyValueMap["log4cplus.appender.STDOUT.filters.1.LogLevelMax"]="FATAL";
		configKeyValueMap["log4cplus.appender.STDOUT.filters.1.AcceptOnMatch"]="true";

		configKeyValueMap["log4cplus.appender.RF"] = "log4cplus::AsyncAppender";
		configKeyValueMap["log4cplus.appender.RF.QueueLimit"] = "1000";
		configKeyValueMap["log4cplus.appender.RF.Appender"] = "log4cplus::RollingFileAppender";
		configKeyValueMap["log4cplus.appender.RF.Appender.ImmediateFlush"] = "true";
		configKeyValueMap["log4cplus.appender.RF.Appender.File"] = configPath_ + configFileName_;//"nvrsdk.log";
		configKeyValueMap["log4cplus.appender.RF.Appender.MaxFileSize"] = "5000KB";
		configKeyValueMap["log4cplus.appender.RF.Appender.MaxBackupIndex"] = "50";
		configKeyValueMap["log4cplus.appender.RF.Append"] = "true";
		configKeyValueMap["log4cplus.appender.RF.Appender.layout"] = "log4cplus::PatternLayout";
		configKeyValueMap["log4cplus.appender.RF.Appender.layout.ConversionPattern"] = pattern;
		std::string strDemiline = "\n";
#ifdef  Win32
		strDemiline = "\r\n";
#endif // Win32
		for (std::map<std::string, std::string>::iterator itr = configKeyValueMap.begin(); itr != configKeyValueMap.end(); ++itr)
		{
			ofile << itr->first << "=" << itr->second;
			ofile << strDemiline;
		}
		ofile.close();
	}
}
void  ClientLoggerImpl::LogInit(log4cplus::LogLevel level)
{
#if 0
	std::string strUserDir = "";
	std::string strLogName = "AdproXO";
	std::string strfilename = strUserDir + strLogName + ".log";

	log4cplus::SharedAppenderPtr append_console(new  log4cplus::ConsoleAppender());

	append_console->setName("appendconsole");

	log4cplus::SharedAppenderPtr appender_file(new  log4cplus::RollingFileAppender(strfilename, LOG_MAX_FILE_SIZE,LOG_BACK_FILE_NUMBER,true,true));
	appender_file->setName("main");

	log4cplus::tstring pattern = LOG4CPLUS_TEXT("%D{%m/%d/%y %H:%M:%S,%Q} [%t] %-5p %c - %m %n");

	
	//appender->setLayout(std::auto_ptr<log4cplus::Layout>(new log4cplus::TTCCLayout()));
	append_console->setLayout(std::unique_ptr<log4cplus::Layout>(new log4cplus::PatternLayout(pattern)));
	appender_file->setLayout(std::unique_ptr<log4cplus::Layout>(new log4cplus::PatternLayout(pattern)));

	static log4cplus::Logger logInstance = log4cplus::Logger::getInstance("XOClient");
	logger_ = &logInstance;
	logger_->addAppender(append_console);
	logger_->addAppender(appender_file);
	logger_->setLogLevel(level);
#endif
	
	
}

void ClientLoggerImpl::log(log4cplus::LogLevel level, const std::string& message)
{
	std::lock_guard<std::mutex> lck(lock_);
	if (!logger_->isEnabledFor(level))
	{
		return;
	}
	logger_->log(level, message);
}
