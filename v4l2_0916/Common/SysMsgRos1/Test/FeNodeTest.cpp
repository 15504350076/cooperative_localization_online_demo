#include <SysApi/SysApi.h>
#include <SysMsg/SysMsg.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <thread>
#include <time.h>

class FeMessageHandler : public Msg::MessagerObserver
{
public:
    FeMessageHandler()
    {
        Msg::Messager::GetMessager().Subscribe("aibox_psdk_quaternion", *this);
    }

    void OnMessage(const std::string &topic, const std::string &payload) override
    {
        sys_api_printf("FeMessageHandler: %s : %s\n", topic.c_str(), payload.c_str());
        if (topic == "aibox_psdk_quaternion")
        {
            /* Expect example json: {"seq": 1, "stamp": 1729758907423, "w": 0.99, "x": -0.01, "y": -0.001, "z": -0.01} */
            auto quaternion_json_obj = nlohmann::json::parse(payload);
            int seq = quaternion_json_obj["seq"].get<int>();
            int stamp = quaternion_json_obj["stamp"].get<int>();
            float w = quaternion_json_obj["w"].get<float>();
            float x = quaternion_json_obj["x"].get<float>();
            float y = quaternion_json_obj["y"].get<float>();
            float z = quaternion_json_obj["z"].get<float>();
            sys_api_printf("FeMessageHandler: seq: %d, stamp: %d, w: %f, x: %f, y: %f, z: %f\n", seq, stamp, w, x, y, z);
        }
        else
        {
            sys_api_printf("FeMessageHandler: Unknown topic: %s\n", topic.c_str());
        }
    }
};

static void send_manual_flight_test(int argc, char *argv[])
{
    if (argc != 3)
    {
        sys_api_printf("Usage: smf direction step\n");
        return;
    }

    struct timespec ts;
    static int seq = 0;
    seq++;
    clock_gettime(CLOCK_REALTIME, &ts);
    nlohmann::json manual_flight_json_obj = {
        {"seq", seq},
        {"stamp", ts.tv_sec},
        {"direction", argv[1]},
        {"step", atoi(argv[2])}};
    sys_api_printf("send_manual_flight_test: %s\n", manual_flight_json_obj.dump().c_str());
    Msg::Messager::GetMessager().Publish("aibox_manual_flight", manual_flight_json_obj.dump());
}

static void send_message(const std::string &topic, const std::string &payload)
{
    auto j = nlohmann::json::parse(payload);

    struct timespec ts;
    static int seq = 0;
    seq++;
    clock_gettime(CLOCK_REALTIME, &ts);
    nlohmann::json msg = {
        {"seq", seq},
        {"stamp", ts.tv_sec}};
    msg.merge_patch(j);
    Msg::Messager::GetMessager().Publish(topic, msg.dump());
}

int main(int argc, char *argv[])
{
    // sys_api_init(true, 2781);
    if (argc != 3)
    {
        std::cout << "Usage: smf direction step" << std::endl;
        return -1;
    }

    auto &messager = Msg::Messager::GetMessager();
    messager.Init(argc, argv, "FeNode");
    messager.CreatePublisherTopics({argv[1]});

    FeMessageHandler fe_message_handler;

    // sys_api_reg_cmd("smf", (void*)send_manual_flight_test, "send manual fligh test command");

    send_message(argv[1], argv[2]);

    messager.StartAsync(2);

    // while (1)
    //{
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    //}

    messager.StopAsync();

    return 0;
}
