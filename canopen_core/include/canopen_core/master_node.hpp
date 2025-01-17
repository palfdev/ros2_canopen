#ifndef MASTER_NODE_HPP_
#define MASTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "canopen_core/node_interfaces/node_canopen_master.hpp"

namespace ros2_canopen
{
    /**
     * @brief Canopen Master Interface
     * 
     * This class provides the interface that all master's that are loaded
     * by the device container need to implement.
     */
    class CanopenMasterInterface
    {
    public:
        /**
         * @brief Initialise the master
         * 
         * This function should intialise the master's functionality.
         * It will be called by the device container after the node was
         * added to the ros executor.
         */
        virtual void init() = 0;
        /**
         * @brief Shutdown the driver
         * 
         * This function should shutdown all functionality and make sure that
         * the master will exit cleanly. It will be called by the device container
         * before exiting.
         * 
         */
        virtual void shutdown() = 0;
        /**
         * @brief Get the master object
         * 
         * This function should return the lely master object. It should
         * throw a ros2_canopen::MasterException if the master object
         * was not yet set.
         * 
         * @return std::shared_ptr<lely::canopen::AsyncMaster> 
         */
        virtual std::shared_ptr<lely::canopen::AsyncMaster> get_master() = 0;
        /**
         * @brief Get the executor object
         * 
         * This function gets the lely executor object. It should throw
         * a ros2_canopen::MasterException if the executor object is not
         * yet instantiated.
         * 
         * @return std::shared_ptr<lely::ev::Executor> 
         */
        virtual std::shared_ptr<lely::ev::Executor> get_executor() = 0;

        /**
         * @brief Get the node base interface object
         * 
         * This function should return the NodeBaseInterface of the associated
         * ROS node. This is used by device container to add the master to the
         * ros executor.
         * 
         * @return rclcpp::node_interfaces::NodeBaseInterface::SharedPtr 
         */
        virtual rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() = 0;

        /**
         * @brief Check whether the node is a lifecycle node
         * 
         * @return true 
         * @return false 
         */
        virtual bool is_lifecycle() = 0;
    };

    /**
     * @brief Canopen Master
     * 
     * This class implements the Canopen Master Interface and is derived
     * from rclcpp::Node. All unmanaged master nodes should inherit from
     * this class.
     * 
     */
    class CanopenMaster : public CanopenMasterInterface, public rclcpp::Node 
    {
    public:
        std::shared_ptr<node_interfaces::NodeCanopenMasterInterface> node_canopen_master_;
        explicit CanopenMaster(
            const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
            : rclcpp::Node("canopen_master", node_options)
        {
            //node_canopen_master_ = std::make_shared<node_interfaces::NodeCanopenMaster<rclcpp::Node>>(this);
        }
        /**
         * @brief Initialises the driver
         * 
         * This function intialises the driver. It is called by the 
         * device container after adding the node to the ros executor.
         * This function uses the NodeCanopenMasterInterface. It will 
         * call init(), configure() and activate(). If this function
         * does not throw, the dirver is successfully initialised and
         * ready to be used.
         * 
         */
        virtual void init() override;

        /**
         * @brief Shuts the master down
         * 
         * This function is called by the device container before exiting,
         * it should enable a clean exit of the master and especially join
         * all threads. It will call the shutdown() function of the
         * NodeCanopenMasterInterface object associated with this class.
         */
        virtual void shutdown() override;

        /**
         * @brief Get the master object
         * 
         * This function should return the lely master object. It should
         * throw a ros2_canopen::MasterException if the master object
         * was not yet set.
         * 
         * @return std::shared_ptr<lely::canopen::AsyncMaster> 
         */
        virtual std::shared_ptr<lely::canopen::AsyncMaster> get_master() override;
        /**
         * @brief Get the executor object
         * 
         * This function gets the lely executor object. It should throw
         * a ros2_canopen::MasterException if the executor object is not
         * yet instantiated.
         * 
         * @return std::shared_ptr<lely::ev::Executor> 
         */
        virtual std::shared_ptr<lely::ev::Executor> get_executor() override;
        /**
         * @brief Get the node base interface object
         * 
         * This function should return the NodeBaseInterface of the associated
         * ROS node. This is used by device container to add the master to the
         * ros executor.
         * 
         * @return rclcpp::node_interfaces::NodeBaseInterface::SharedPtr 
         */
        virtual rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() override
        {
            return rclcpp::Node::get_node_base_interface();
        }
        /**
         * @brief Check whether the node is a lifecycle node
         * 
         * @return false 
         */
        virtual bool is_lifecycle()
        {
            return false;
        }
    };

    class LifecycleCanopenMaster : public CanopenMasterInterface, public rclcpp_lifecycle::LifecycleNode
    {
    protected:
        std::shared_ptr<node_interfaces::NodeCanopenMasterInterface> node_canopen_master_;

    public:
        LifecycleCanopenMaster(
            const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
            : rclcpp_lifecycle::LifecycleNode("lifecycle_canopen_master", node_options)
        {
            node_canopen_master_ = std::make_shared<node_interfaces::NodeCanopenMaster<rclcpp_lifecycle::LifecycleNode>>(this);
        }
        /**
         * @brief Initialises the driver
         * 
         * This function intialises the driver. It is called by the 
         * device container after adding the node to the ros executor.
         * This function uses the NodeCanopenMasterInterface. It will 
         * call init(). If this function
         * does not throw, the dirver is successfully initialised and
         * ready to switch through the lifecycle.
         * 
         */
        virtual void init() override;

        /**
         * @brief Shuts the master down
         * 
         * This function is called by the device container before exiting,
         * it should enable a clean exit of the master and especially join
         * all threads. It will call the shutdown() function of the
         * NodeCanopenMasterInterface object associated with this class.
         */
        virtual void shutdown() override;
        /**
         * @brief Get the master object
         * 
         * This function should return the lely master object. It should
         * throw a ros2_canopen::MasterException if the master object
         * was not yet set.
         * 
         * @return std::shared_ptr<lely::canopen::AsyncMaster> 
         */
        virtual std::shared_ptr<lely::canopen::AsyncMaster> get_master() override;
        /**
         * @brief Get the executor object
         * 
         * This function gets the lely executor object. It should throw
         * a ros2_canopen::MasterException if the executor object is not
         * yet instantiated.
         * 
         * @return std::shared_ptr<lely::ev::Executor> 
         */
        virtual std::shared_ptr<lely::ev::Executor> get_executor() override;
        /**
         * @brief Configure Transition Callback
         * 
         * This function will call the configure() function of the 
         * NodeCanopenMasterInterface object associated with this class.
         * 
         * @param state 
         * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &state);
        /**
         * @brief Activate Transition Callback
         * 
         * This function will call the active() function of the 
         * NodeCanopenMasterInterface object associated with this class.
         * 
         * @param state 
         * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &state);
        /**
         * @brief Deactivet Transition Callback
         * 
         * This function will call the deactive() function of the 
         * NodeCanopenMasterInterface object associated with this class.
         * 
         * @param state 
         * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &state);
        /**
         * @brief Cleanup Transition Callback
         * 
         * This function will call the cleanup() function of the 
         * NodeCanopenMasterInterface object associated with this class.
         * 
         * @param state 
         * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &state);
        /**
         * @brief Shutdown Transition Callback
         * 
         * This function will call the shutdown() function of the 
         * NodeCanopenMasterInterface object associated with this class.
         * 
         * @param state 
         * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State &state);
        /**
         * @brief Get the node base interface object
         * 
         * This function should return the NodeBaseInterface of the associated
         * ROS node. This is used by device container to add the master to the
         * ros executor.
         * 
         * @return rclcpp::node_interfaces::NodeBaseInterface::SharedPtr 
         */
        virtual rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() override
        {
            return rclcpp_lifecycle::LifecycleNode::get_node_base_interface();
        }
        /**
         * @brief Check whether the node is a lifecycle node
         * 
         * @return true 
         */
        virtual bool is_lifecycle()
        {
            return true;
        }
    };

}

#endif