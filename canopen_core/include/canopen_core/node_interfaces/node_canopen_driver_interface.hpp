#ifndef NODE_CANOPEN_DRIVER_INTERFACE_HPP_
#define NODE_CANOPEN_DRIVER_INTERFACE_HPP_

#include <lely/coapp/master.hpp>
#include <lely/ev/exec.hpp>

namespace ros2_canopen
{
    namespace node_interfaces
    {
        /**
         * @brief Node Canopen Driver Interface
         * 
         * This node provides the interface for NodeCanopenDriver classes
         * that provide ROS node independent CANopen functionality.
         * 
         */
        class NodeCanopenDriverInterface
        {
            public:
            NodeCanopenDriverInterface(){}
            /**
             * @brief Set Master
             * 
             * Sets the Lely Canopen Master Objects needed by the driver
             * to add itself to the master.
             * 
             */
            virtual void set_master(
                std::shared_ptr<lely::ev::Executor> exec,
                std::shared_ptr<lely::canopen::AsyncMaster> master) = 0;

            /**
             * @brief Demand set Master
             * 
             * This function should ask the drivers parent container to
             * set the master objects. This should result in a call to 
             * set_master function.
             * 
             */
            virtual void demand_set_master() = 0;
            

            /**
             * @brief Initialise the driver
             * 
             * In this function the ROS interface should be created and
             * potentially necessary callback groups.
             * 
             */
            virtual void init() = 0;

            /**
             * @brief Configure the driver
             * 
             * This function should contain the configuration related things,
             * such as reading parameter data or configuration data from files.
             * 
             */
            virtual void configure() = 0;

            /**
             * @brief Activate the driver
             * 
             * This function should activate the driver, consequently it needs to start all timers and threads necessary
             * for the operation of the driver.
             * 
             */
            virtual void activate() = 0;

            /**
             * @brief Deactivate the driver
             * 
             * This function should deactivate the driver, consequently it needs to stop all timers and threads that
             * are related to the operation of the diver.
             * 
             */
            virtual void deactivate() = 0;

            /**
             * @brief Cleanup the driver
             * 
             * This function needs to clean the internal state of the driver. This means
             * all data should be deleted.
             * 
             */
            virtual void cleanup() = 0;

            /**
             * @brief Shutdown the driver
             * 
             * This function should shutdown the driver.
             * 
             */
            virtual void shutdown() = 0;

            /**
             * @brief Add the driver to master
             * 
             */
            virtual void add_to_master() = 0;

            /**
             * @brief Remove the driver from master
             * 
             */
            virtual void remove_from_master() = 0; 
        };
    }
}

#endif
