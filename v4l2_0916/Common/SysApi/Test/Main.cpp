/* 
    This is the main file for the SysApi test project.
    This project is used to test the SysApi library.
*/

#include <iostream>
#include <SysApi/SysApi.h>

int main(int argc, char *argv[])
{
    std::cout << "SysApi Test" << std::endl;
    sys_api_init(true, 2700);
    while (true)
    {
        sleep(1);
    }
    return 0;
}
