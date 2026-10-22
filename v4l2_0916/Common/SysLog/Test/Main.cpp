#include <iostream>
#include <thread>
#include <LogAdapter/ClientLogger.h>

int main()
{
    ClientLogger::Instance()->SetLogFilePath("./Logs/");
    ClientLogger::Instance()->SetLogFileName("test.log");
    ClientLogger::Instance()->Init();
    ClientLogger::Instance()->info("infotest=%d", 123);
    std::cout << "log test end\n";

    for (int i = 0; i < 100000; i++)
    {
        auto start = std::chrono::high_resolution_clock::now();
        ClientLogger::Instance()->info("infotest=%d", i);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        if (duration > 100)
        {
            std::cout << " Duration: " << duration << " ms\n";
        }
    }
}
