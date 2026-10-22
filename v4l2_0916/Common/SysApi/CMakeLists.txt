file(GLOB_RECURSE srcs CONFIGURE_DEPENDS Src/*.cpp Include/*.h)
add_library(SysApi STATIC ${srcs})
target_include_directories(SysApi PUBLIC Include)

find_package(Threads REQUIRED)
file(GLOB_RECURSE test_srcs CONFIGURE_DEPENDS Test/Main.cpp Test/*.h)
add_executable(SysApiTest ${test_srcs})
target_link_libraries(SysApiTest SysApi Threads::Threads)

file(GLOB_RECURSE shm_reader_test_srcs CONFIGURE_DEPENDS Test/SysShmWrapperReader.cpp Test/SysShmWrapperUtils.cpp)
add_executable(SysShmWrapperReader ${shm_reader_test_srcs})
target_link_libraries(SysShmWrapperReader SysApi Threads::Threads)

file(GLOB_RECURSE shm_writer_test_srcs CONFIGURE_DEPENDS Test/SysShmWrapperWriter.cpp Test/SysShmWrapperUtils.cpp)
add_executable(SysShmWrapperWriter ${shm_writer_test_srcs})
target_link_libraries(SysShmWrapperWriter SysApi Threads::Threads)

file(GLOB_RECURSE shm_writernoblock_test_srcs CONFIGURE_DEPENDS Test/SysShmWrapperWriterNoBlock.cpp Test/SysShmWrapperUtils.cpp)
add_executable(SysShmWrapperWriterNoBlock ${shm_writernoblock_test_srcs})
target_link_libraries(SysShmWrapperWriterNoBlock SysApi Threads::Threads)
