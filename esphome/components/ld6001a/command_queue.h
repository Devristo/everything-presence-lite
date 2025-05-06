#include <queue>
#include <string>
#include <functional>

namespace esphome::ld6001a {
class Command {
 public:
  std::string data;
  std::function<void(const std::string &)> onResponse;

  Command(const std::string &cmd, std::function<void(const std::string &)> cb = nullptr)
      : data(cmd), onResponse(cb) {}
};

class CommandQueue {
    public:
        CommandQueue(std::function<void(const std::string&)> sender): sendFunction(sender) {};

        void enqueue(const Command& cmd) {
            queue.push(cmd);
            trySendNext();
        }
    
        void handleResponse(const std::string& response) {
            if (!waitingForAck || queue.empty()) return;
    
            Command& cmd = queue.front();
            if (cmd.onResponse) {
                cmd.onResponse(response);
            }
    
            queue.pop();
            waitingForAck = false;
            trySendNext();
        }
    
    private:
        std::queue<Command> queue;
        std::function<void(const std::string&)> sendFunction;
        bool waitingForAck = false;
    
        void trySendNext() {
            if (!waitingForAck && !queue.empty() && sendFunction) {
                const Command& cmd = queue.front();
                sendFunction(cmd.data);
                waitingForAck = true;
            }
        }
    };

}  // namespace esphome::ld6001a