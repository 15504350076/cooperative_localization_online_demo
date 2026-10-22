#include <LogAdapter/ClientLogger.h>
#include <log4cplus/loglevel.h>
#include <cstdarg>
#include "ClientLoggerImpl.h"

#define LOG_MAX_FILE_SIZE 500*1024
#define LOG_BACK_FILE_NUMBER 5
#define NVRSDK_LOG_MAX_LENGTH	1024

ClientLogger* ClientLogger::pInstance_ = nullptr;

ClientLogger* ClientLogger::Instance()
{
	static std::once_flag oc_flg;
	std::call_once(oc_flg, [&]() { pInstance_ = new ClientLogger(); });
	return pInstance_;
}

ClientLogger::ClientLogger()
	: pImpl_(nullptr)
{
	pImpl_ = new ClientLoggerImpl();
}

ClientLogger::~ClientLogger()
{
	if (pImpl_)
		delete pImpl_;
}

void ClientLogger::SetLogFilePath(const char* path)
{
	if (pImpl_)
		pImpl_->SetLogFilePath(path);
}

void ClientLogger::SetLogFileName(const char* logFileName)
{
	if (pImpl_)
		pImpl_->SetLogFileName(logFileName);
}

void ClientLogger::Init()
{
	if (pImpl_)
		pImpl_->Init();
}

void ClientLogger::debug(const char *format, ...)
{
	if (!pImpl_)
		return;

	va_list args;
	char buffer[NVRSDK_LOG_MAX_LENGTH + 1] = { 0 };
	va_start(args, format);
	vsnprintf(buffer, NVRSDK_LOG_MAX_LENGTH, format, args);
	va_end(args);

	pImpl_->log(log4cplus::DEBUG_LOG_LEVEL, buffer);
}

void ClientLogger::info(const char *format, ...)
{
	if (!pImpl_)
		return;

	va_list args;
	char buffer[NVRSDK_LOG_MAX_LENGTH + 1] = { 0 };
	va_start(args, format);
	vsnprintf(buffer, NVRSDK_LOG_MAX_LENGTH, format, args);
	va_end(args);

	pImpl_->log(log4cplus::INFO_LOG_LEVEL, buffer);
}

void ClientLogger::warn(const char *format, ...)
{
	if (!pImpl_)
		return;

	va_list args;
	char buffer[NVRSDK_LOG_MAX_LENGTH + 1] = { 0 };
	va_start(args, format);
	vsnprintf(buffer, NVRSDK_LOG_MAX_LENGTH, format, args);
	va_end(args);
	pImpl_->log(log4cplus::WARN_LOG_LEVEL, buffer);
}

void ClientLogger::error(const char *format, ...)
{
	if (!pImpl_)
		return;

	va_list args;
	char buffer[NVRSDK_LOG_MAX_LENGTH + 1] = { 0 };
	va_start(args, format);
	vsnprintf(buffer, NVRSDK_LOG_MAX_LENGTH, format, args);
	va_end(args);
	pImpl_->log(log4cplus::ERROR_LOG_LEVEL, buffer);
}

void ClientLogger::fatal(const char *format, ...)
{
	if (!pImpl_)
		return;

	va_list args;
	char buffer[NVRSDK_LOG_MAX_LENGTH + 1] = { 0 };
	va_start(args, format);
	vsnprintf(buffer, NVRSDK_LOG_MAX_LENGTH, format, args);
	va_end(args);
	pImpl_->log(log4cplus::FATAL_LOG_LEVEL, buffer);
}
