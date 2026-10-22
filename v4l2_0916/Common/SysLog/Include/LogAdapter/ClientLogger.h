
#pragma once

class ClientLoggerImpl;
class ClientLogger
{
public:
	static ClientLogger* Instance();
	void SetLogFilePath(const char* path); // such as:  /var/log/ or null
	void SetLogFileName(const char* logFileName); //such as: aibox.log
	void Init();

	void debug(const char *format, ...);
	void info(const char *format, ...);
	void warn(const char *format, ...);
	void error(const char *format, ...);
	void fatal(const char *format, ...);
private:
	ClientLogger();
	~ClientLogger();
private:

private:
	static ClientLogger* pInstance_;
	ClientLoggerImpl* pImpl_;
};
