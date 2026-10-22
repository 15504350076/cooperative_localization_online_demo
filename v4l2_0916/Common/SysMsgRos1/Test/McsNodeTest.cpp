#include <SysApi/SysApi.h>
#include <SysMsg/SysMsg.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <thread>
#include <time.h>

class McsMessageHandler : public Msg::MessagerObserver
{
public:
  McsMessageHandler()
  {
    Msg::Messager::GetMessager().Subscribe("aibox_manual_flight", *this);
  }

  void OnMessage(const std::string &topic, const std::string &payload) override
  {
    sys_api_printf("McsMessageHandler: %s : %s\n", topic.c_str(), payload.c_str());
    if (topic == "aibox_manual_flight")
    {
        /* Expect example json: {"seq": 1, "stamp": 17222, "direction": "up", "step": 3} */
        auto manual_flight_json_obj = nlohmann::json::parse(payload);
        int seq = manual_flight_json_obj["seq"].get<int>();
        int stamp = manual_flight_json_obj["stamp"].get<int>();
        std::string direction = manual_flight_json_obj["direction"].get<std::string>();
        int step = manual_flight_json_obj["step"].get<int>();
        sys_api_printf("McsMessageHandler: seq: %d, stamp: %d, direction: %s, step: %d\n", seq, stamp, direction.c_str(), step);
    }
    else
    {
        sys_api_printf("McsMessageHandler: Unknown topic: %s\n", topic.c_str());
    }
  }
};

static void send_flight_axis_test(int argc, char* argv[])
{
    if (argc != 6)
    {
        sys_api_printf("Usage: sfa loop w x y z\n");
        return;
    }
    int loop = atoi(argv[1]);
    for (int i = 0; i < loop; i++)
    {
        struct timespec ts;
        static int seq = 0;
        seq++;
        clock_gettime(CLOCK_REALTIME, &ts);
        nlohmann::json quaternion_json_obj = {
            {"seq", seq},
            {"stamp", ts.tv_sec},
            {"w", atof(argv[2])},
            {"x", atof(argv[3])},
            {"y", atof(argv[4])},
            {"z", atof(argv[5])}
        };
        sys_api_printf("send_flight_axis_test: %s\n", quaternion_json_obj.dump().c_str());
        Msg::Messager::GetMessager().Publish("aibox_psdk_quaternion", quaternion_json_obj.dump());
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

int main(int argc, char* argv[])
{
    sys_api_init(true, 2780);
    auto& messager = Msg::Messager::GetMessager();
    messager.Init(argc, argv, "McsAppNode");
    messager.CreatePublisherTopics({"aibox_psdk_quaternion"});

    McsMessageHandler mcs_message_handler;

    sys_api_reg_cmd("sfa", (void*)send_flight_axis_test, "send fligh axis test command");

    messager.StartAsync(2);
    
    while(1)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    messager.StopAsync();
    return 0;
}
