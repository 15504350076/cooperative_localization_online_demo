#include "SysMsg/SysMsg.h"

#include <iostream>
#include <thread>

static std::string BYE ="bye-bye";
static bool stop = false;

class Observer : public Msg::MessagerObserver
{
public:
  Observer()
  {
    Msg::Messager::GetMessager().Subscribe("topic_1", *this);
    Msg::Messager::GetMessager().Subscribe("topic_2", *this);
    Msg::Messager::GetMessager().Subscribe("topic_3", *this);
  }

  void OnMessage(const std::string &topic, const std::string &payload) override
  {
    std::cout << "Observer: " << topic << " : " << payload << std::endl;
    if (payload == BYE) {
      stop = true;
    }
  }
};

void Subscriber_Thread(bool &stop)
{
  do {
    Msg::Messager::GetMessager().RunOnce();
  } while (!stop);
}

void Publisher_Thread(const std::string &topic, int num, int delay_ms)
{
  char str[32];
  for (int i = 0; i < num; ++i) {
    sprintf(str, "%d", i);
    Msg::Messager::GetMessager().Publish(topic, str);
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
  }
}

int main(int argc, char* argv[])
{
  const auto pid = fork();
  if (pid < 0) {
    return EXIT_FAILURE;
  }

  if (pid == 0) {
    auto& messager = Msg::Messager::GetMessager();
    messager.Init(argc, argv, "SysMsg_Subscriber");
    Observer observer;
    std::thread thread(Subscriber_Thread, std::ref(stop));
    thread.join();
  }
  else {
    auto& messager = Msg::Messager::GetMessager();
    messager.Init(argc, argv, "SysMsg_Publisher");
    messager.CreatePublisherTopics({"topic_1", "topic_2", "topic_3"});
    std::thread thread1(Publisher_Thread, "topic_1", 100, 10);
    std::thread thread2(Publisher_Thread, "topic_2", 300, 20);
    std::thread thread3(Publisher_Thread, "topic_3", 400, 30);
    std::thread thread4(Publisher_Thread, "topic_1", 400, 40);
    thread1.join();
    thread2.join();
    thread3.join();
    thread4.join();
    messager.Publish("topic_1", BYE);
  }

  return EXIT_SUCCESS;
}
