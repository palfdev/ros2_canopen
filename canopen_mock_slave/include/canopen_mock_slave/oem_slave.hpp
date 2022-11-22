#ifndef OEM_SLAVE_HPP
#define OEM_SLAVE_HPP
#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/slave.hpp>
#include <lely/ev/co_task.hpp>

#include <atomic>
#include <thread>
#include <mutex>

#include "lifecycle_msgs/msg/state.hpp"
#include "canopen_mock_slave/base_slave.hpp"

using namespace lely;
using namespace std::chrono_literals;

namespace ros2_canopen
{

    class oemMockSlave : public canopen::BasicSlave
    {
    public:
        explicit oemMockSlave(io::TimerBase &timer, io::CanChannelBase &chan,
                                 const ::std::string &dcf_txt,
                                 const ::std::string &dcf_bin = "", uint8_t id = 0xff) : canopen::BasicSlave(timer, chan, dcf_txt, dcf_bin, id)
        {

        }
    };

    class oemSlave : public BaseSlave
    {
    public:
        explicit oemSlave(const std::string &node_name, bool intra_process_comms = false) : BaseSlave(node_name, intra_process_comms)
        {
        }

        void run() override
        {
            io::IoGuard io_guard;
            io::Context ctx;
            io::Poll poll(ctx);
            ev::Loop loop(poll.get_poll());
            auto exec = loop.get_executor();
            io::Timer timer(poll, exec, CLOCK_MONOTONIC);
            io::CanController ctrl(can_interface_name_.c_str());
            io::CanChannel chan(poll, exec);
            chan.open(ctrl);
            ros2_canopen::oemMockSlave slave(timer, chan, slave_config_.c_str(), "", node_id_);
            slave.Reset();

            RCLCPP_INFO(this->get_logger(), "Created OEM slave for node_id %i.", node_id_);

            loop.run();
            ctx.shutdown();
            RCLCPP_INFO(this->get_logger(), "Stopped CANopen Event Loop.");
        }
    };
}
#endif