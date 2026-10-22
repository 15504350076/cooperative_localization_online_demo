#All the third party libraries are included here.

#For a third party library without cmake file, we can create an imported target for the library.
#By using an imported target with the INTERFACE_INCLUDE_DIRECTORIES property, 
#you eliminate the need to manually specify the include directories for each target that uses the SDK.
#This approach simplifies your CMake configuration and ensures consistency across all targets that depend on the SDK.

#For third party library with Findcmake file, we can use find package to.

#For third party library with CMakeLists.txt file, we can use add_subdirectory to include the library.

# We always use native compile instead of cross compile.
# Get the running processor by using Linux command uname -m and set it to RUNNING_ARCH
execute_process(
    COMMAND uname -m
    OUTPUT_VARIABLE RUNNING_ARCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Only support x86_64 and aarch64, other architectures are not supported.
if (NOT ${RUNNING_ARCH} MATCHES "x86_64" AND NOT ${RUNNING_ARCH} MATCHES "aarch64")
    message(FATAL_ERROR "Only support x86_64 and aarch64, other architectures are not supported.")
else()
    message(STATUS "Architecture ${RUNNING_ARCH} is supported.")
endif()

# Determine the Ubuntu version
execute_process(
    COMMAND lsb_release -rs
    OUTPUT_VARIABLE UBUNTU_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Set the parameter based on the Ubuntu version
if(UBUNTU_VERSION STREQUAL "20.04")
    set(UBUNTU_PARAM "ubuntu20.04")
elseif(UBUNTU_VERSION STREQUAL "22.04")
    set(UBUNTU_PARAM "ubuntu22.04")
elseif(UBUNTU_VERSION STREQUAL "24.04")
    set(UBUNTU_PARAM "ubuntu22.04") 
else()
    message(FATAL_ERROR "Unsupported Ubuntu version: ${UBUNTU_VERSION}")
endif()

find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)


# nlohmann JSON header only library
# Set the include directory for the nlohmann JSON library
# We don't add one more layer folder of nlohmann since we want header added with nlohmann/json.hpp
set(NLOHMANN_JSON_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/Include")
# Create an imported target for the nlohmann JSON library
add_library(NlohmannJson INTERFACE IMPORTED)
set_target_properties(NlohmannJson PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${NLOHMANN_JSON_INCLUDE_DIR}
)

# cares library target define and import
# Set the include directory for the cares library
set(CARES_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/Include/c-ares")
set(CARES_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/Lib/${RUNNING_ARCH}/${UBUNTU_PARAM}/c-ares/libcares.a")

# Create an imported target for the cares library
add_library(cares STATIC IMPORTED)
set_target_properties(cares PROPERTIES
    IMPORTED_LOCATION ${CARES_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${CARES_INCLUDE_DIR}
)

# osip2 library target define and import
# Set the include directory for the osip2 library
set(OSIP2_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/Include/osip2")
set(OSIP2_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/Lib/${RUNNING_ARCH}/${UBUNTU_PARAM}/osip2/libosip2.a")

# Create an imported target for the osip2 library
add_library(osip2 STATIC IMPORTED)
set_target_properties(osip2 PROPERTIES
    IMPORTED_LOCATION ${OSIP2_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${OSIP2_INCLUDE_DIR}
)

# osipparser2 library target define and import
# Set the include directory for the osipparser2 library
set(OSIPPARSER2_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/Include/osipparser2")
set(OSIPPARSER2_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/Lib/${RUNNING_ARCH}/${UBUNTU_PARAM}/osipparser2/libosipparser2.a")

# Create an imported target for the osipparser2 library
add_library(osipparser2 STATIC IMPORTED)
set_target_properties(osipparser2 PROPERTIES
    IMPORTED_LOCATION ${OSIPPARSER2_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${OSIPPARSER2_INCLUDE_DIR}
)

# eXosip2 library target define and import
# Set the include directory for the eXosip2 library
set(EXOSIP2_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/Include")
set(EXOSIP2_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/Lib/${RUNNING_ARCH}/${UBUNTU_PARAM}/eXosip2/libeXosip2.a")

# Create an imported target for the eXosip2 library
add_library(eXosip2 STATIC IMPORTED)
set_target_properties(eXosip2 PROPERTIES
    IMPORTED_LOCATION ${EXOSIP2_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${EXOSIP2_INCLUDE_DIR}
)

# jrtp library target define and import
# Set the include directory for the jrtp library
set(JRTP_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/Include/jrtplib")
set(JRTP_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/Lib/${RUNNING_ARCH}/${UBUNTU_PARAM}/jrtplib/libjrtp.a")

# Create an imported target for the jrtp library
add_library(jrtp STATIC IMPORTED)
set_target_properties(jrtp PROPERTIES
    IMPORTED_LOCATION ${JRTP_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${JRTP_INCLUDE_DIR}
)

#ASIO header only library
set(ASIO_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/Include/asio")
# Create an imported target for the ASIO library
add_library(ASIO INTERFACE IMPORTED)
set_target_properties(ASIO PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${ASIO_INCLUDE_DIR}
)

#Websocketpp header only library
set(WEBSOCKETPP_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/Include/websocketpp")
# Create an imported target for the Websocketpp library
add_library(Websocketpp INTERFACE IMPORTED)
set_target_properties(Websocketpp PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${WEBSOCKETPP_INCLUDE_DIR}
)


# log4cplus library target define and import
# Set the include directory for the log4cplus library
set(LOG4CPLUS_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/Include")
set(LOG4CPLUS_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/Lib/${RUNNING_ARCH}/${UBUNTU_PARAM}/log4cplus/liblog4cplus.a")

# Create an imported target for the log4cplus library
add_library(log4cplus STATIC IMPORTED)
set_target_properties(log4cplus PROPERTIES
    IMPORTED_LOCATION ${LOG4CPLUS_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${LOG4CPLUS_INCLUDE_DIR}
)

# BasicUsageEnvironment library target define and import
# Set the include directory for the BasicUsageEnvironment library
set(BASICUSAGEENVIRONMENT_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/Include/live555/BasicUsageEnvironment/include")
set(BASICUSAGEENVIRONMENT_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/Lib/${RUNNING_ARCH}/${UBUNTU_PARAM}/live555/libBasicUsageEnvironment.a")

# Create an imported target for the BasicUsageEnvironment library
add_library(BasicUsageEnvironment STATIC IMPORTED)
set_target_properties(BasicUsageEnvironment PROPERTIES
    IMPORTED_LOCATION ${BASICUSAGEENVIRONMENT_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${BASICUSAGEENVIRONMENT_INCLUDE_DIR}
)

# groupsock library target define and import
# Set the include directory for the groupsock library
set(GROUPSOCK_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/Include/live555/groupsock/include")
set(GROUPSOCK_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/Lib/${RUNNING_ARCH}/${UBUNTU_PARAM}/live555/libgroupsock.a")

# Create an imported target for the groupsock library
add_library(groupsock STATIC IMPORTED)
set_target_properties(groupsock PROPERTIES
    IMPORTED_LOCATION ${GROUPSOCK_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${GROUPSOCK_INCLUDE_DIR}
)

# UsageEnvironment library target define and import
# Set the include directory for the UsageEnvironment library
set(USAGEENVIRONMENT_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/Include/live555/UsageEnvironment/include")
set(USAGEENVIRONMENT_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/Lib/${RUNNING_ARCH}/${UBUNTU_PARAM}/live555/libUsageEnvironment.a")

# Create an imported target for the UsageEnvironment library
add_library(UsageEnvironment STATIC IMPORTED)
set_target_properties(UsageEnvironment PROPERTIES
    IMPORTED_LOCATION ${USAGEENVIRONMENT_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${USAGEENVIRONMENT_INCLUDE_DIR}
)

# liveMedia library target define and import
# Set the include directory for the liveMedia library
set(LIVEMEDIA_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/Include/live555/liveMedia/include")
set(LIVEMEDIA_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/Lib/${RUNNING_ARCH}/${UBUNTU_PARAM}/live555/libliveMedia.a")

# Create an imported target for the liveMedia library
add_library(liveMedia STATIC IMPORTED)
set_target_properties(liveMedia PROPERTIES
    IMPORTED_LOCATION ${LIVEMEDIA_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${LIVEMEDIA_INCLUDE_DIR}
)

# avcodec library target define and import
# Set the include directory for the avcodec library
set(AVCODEC_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/Include/ffmpeg")
set(AVCODEC_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/Lib/${RUNNING_ARCH}/${UBUNTU_PARAM}/ffmpeg/libavcodec.a")

# Create an imported target for the avcodec library
add_library(avcodec STATIC IMPORTED)
set_target_properties(avcodec PROPERTIES
    IMPORTED_LOCATION ${AVCODEC_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${AVCODEC_INCLUDE_DIR}
)

# avdevice library target define and import
# Set the include directory for the avdevice library
set(AVDEVICE_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/Include/ffmpeg")
set(AVDEVICE_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/Lib/${RUNNING_ARCH}/${UBUNTU_PARAM}/ffmpeg/libavdevice.a")

# Create an imported target for the avdevice library
add_library(avdevice STATIC IMPORTED)
set_target_properties(avdevice PROPERTIES
    IMPORTED_LOCATION ${AVDEVICE_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${AVDEVICE_INCLUDE_DIR}
)

# avfilter library target define and import
# Set the include directory for the avfilter library
set(AVFILTER_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/Include/ffmpeg")
set(AVFILTER_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/Lib/${RUNNING_ARCH}/${UBUNTU_PARAM}/ffmpeg/libavfilter.a")

# Create an imported target for the avfilter library
add_library(avfilter STATIC IMPORTED)
set_target_properties(avfilter PROPERTIES
    IMPORTED_LOCATION ${AVFILTER_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${AVFILTER_INCLUDE_DIR}
)

# avformat library target define and import
# Set the include directory for the avformat library
set(AVFORMAT_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/Include/ffmpeg")
set(AVFORMAT_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/Lib/${RUNNING_ARCH}/${UBUNTU_PARAM}/ffmpeg/libavformat.a")

# Create an imported target for the avformat library
add_library(avformat STATIC IMPORTED)
set_target_properties(avformat PROPERTIES
    IMPORTED_LOCATION ${AVFORMAT_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${AVFORMAT_INCLUDE_DIR}
)

# avutil library target define and import
# Set the include directory for the avutil library
set(AVUTIL_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/Include/ffmpeg")
set(AVUTIL_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/Lib/${RUNNING_ARCH}/${UBUNTU_PARAM}/ffmpeg/libavutil.a")

# Create an imported target for the avutil library
add_library(avutil STATIC IMPORTED)
set_target_properties(avutil PROPERTIES
    IMPORTED_LOCATION ${AVUTIL_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${AVUTIL_INCLUDE_DIR}
)

# swresample library target define and import
# Set the include directory for the swresample library
set(SWRESAMPLE_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/Include/ffmpeg")
set(SWRESAMPLE_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/Lib/${RUNNING_ARCH}/${UBUNTU_PARAM}/ffmpeg/libswresample.a")

# Create an imported target for the swresample library
add_library(swresample STATIC IMPORTED)
set_target_properties(swresample PROPERTIES
    IMPORTED_LOCATION ${SWRESAMPLE_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${SWRESAMPLE_INCLUDE_DIR}
)

# swscale library target define and import
# Set the include directory for the swscale library
set(SWSCALE_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/Include/ffmpeg")
set(SWSCALE_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/Lib/${RUNNING_ARCH}/${UBUNTU_PARAM}/ffmpeg/libswscale.a")

# Create an imported target for the swscale library
add_library(swscale STATIC IMPORTED)
set_target_properties(swscale PROPERTIES
    IMPORTED_LOCATION ${SWSCALE_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${SWSCALE_INCLUDE_DIR}
)

# postproc library target define and import
# Set the include directory for the postproc library
set(POSTPROC_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/Include/ffmpeg")
set(POSTPROC_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/Lib/${RUNNING_ARCH}/${UBUNTU_PARAM}/ffmpeg/libpostproc.a")

# Create an imported target for the postproc library
add_library(postproc STATIC IMPORTED)
set_target_properties(postproc PROPERTIES
    IMPORTED_LOCATION ${POSTPROC_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${POSTPROC_INCLUDE_DIR}
)

# tinyxml2 library target define and import
# Set the include directory for the postproc library
set(TINYXML2_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/Include")
set(TINYXML2_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/Lib/${RUNNING_ARCH}/${UBUNTU_PARAM}/tinyxml2/libtinyxml2.a")

# Create an imported target for the postproc library
add_library(tinyxml2 STATIC IMPORTED)
set_target_properties(tinyxml2 PROPERTIES
    IMPORTED_LOCATION ${TINYXML2_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${TINYXML2_INCLUDE_DIR}
)

