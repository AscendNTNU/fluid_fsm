
/**
 * @mainpage
 *
 * Finite state machine built around the MAVROS protocol. It's built around clients requesting the FSM (the server)
 * to do operations through ROS services.
 */

/**
 * @file fluid.h
 */

#ifndef FLUID_H
#define FLUID_H

#include <fluid/Explore.h>
#include <fluid/Interact.h>
#include <fluid/Land.h>
#include <fluid/OperationCompletion.h>
#include <fluid/TakeOff.h>
#include <fluid/Travel.h>
#include <geometry_msgs/Point32.h>
#include <ros/ros.h>

#include <map>
#include <memory>

#include "operation.h"
#include "status_publisher.h"

/**
 * @brief Defines all the parameters for fluid.
 */
struct FluidConfiguration {
    /**
     * @brief The unified refresh rate across the operation machine.
     */
    const int refresh_rate;

    /**
     * @brief Whether the drone will arm automatically.
     */
    const bool should_auto_arm;

    /**
     * @brief Whether the drone will go into offboard mode automatically.
     */
    const bool should_auto_offboard;

    /**
     * @brief Specifies the radius for within we can say that the drone is at a given position.
     */
    const float distance_completion_threshold;

    /**
     * @brief Specifies how low the velocity has to be before we issue that a given operation has completed. This
     *        serves the purpose to prevent osciallations in the case when new operations are fired the moment the
     *        previous completes. With this threshold, we have to wait for the drone to be "steady" at the current
     *        position before moving on.
     */
    const float velocity_completion_threshold;

    /**
     * @brief Height used when e.g. 0 was given for a setpoint.
     */
    const float default_height;

    /**
     * @brief Kpx, Kpy, Kvx, Kvy are the gains used by the LQR.
     *        Respectively the gain in position for both axes and in velocity for both axes.
     */
    const float* LQR_gains; 

    /**
     * @brief Show some debugging prints during the interact operation
     */
    const bool interaction_show_prints;
    
    /**
     * @brief Use ground_truth data for interact operation.
     */
    const bool interaction_ground_truth;  

    /**
     * @brief Show some debugging prints during the interact operation
     */
    const float interact_max_vel;
    
    /**
     * @brief Use ground_truth data for interact operation.
     */
    const float interact_max_acc;  
};

/**
 * @brief The main class for the FSM.
 */
class Fluid {
   private:
    /**
     * @brief Only instance to this class.
     */
    static std::shared_ptr<Fluid> instance_ptr;

    /**
     * @brief Interface for publishing status messages.
     */
    std::shared_ptr<StatusPublisher> status_publisher_ptr;

    /**
     * @brief Sets up the service servers and clients.
     */
    Fluid(const FluidConfiguration configuration) : configuration(configuration) {
        take_off_server = node_handle.advertiseService("fluid/take_off", &Fluid::take_off, this);
        travel_server = node_handle.advertiseService("fluid/travel", &Fluid::travel, this);
        explore_server = node_handle.advertiseService("fluid/explore", &Fluid::explore, this);
        interact_server = node_handle.advertiseService("fluid/interact", &Fluid::interact, this);
        land_server = node_handle.advertiseService("fluid/land", &Fluid::land, this);
        operation_completion_client =
            node_handle.serviceClient<fluid::OperationCompletion>("fluid/operation_completion");
        status_publisher_ptr = std::make_shared<StatusPublisher>();
    }

    /**
     * @brief Kept hidden since this class is a singleton.
     */
    //Fluid& operator=(Fluid const&){}; //commented because it builds without and it created warnings

    /**
     * @brief Mapping for responses in service calls.
     */
    struct Response {
        /**
         * @brief Whether the service call was successful.
         */
        bool success;

        /**
         * @brief Error messsage (if any)
         */
        std::string message;
    };

    /**
     * @brief The current operation being executed
     */
    std::shared_ptr<Operation> current_operation_ptr;

    /**
     * @brief The current operation being executed, essentially the current item in the list of operations
     *        in #operation_execution_queue.
     */
    std::string current_operation;

    /**
     * @brief The list of operations which shall be executed.
     */
    std::list<std::shared_ptr<Operation>> operation_execution_queue;

    /**
     * @brief Flag for checking if one of the service handlers were called and that the FSM should transition to
     *        another operation.
     */
    bool got_new_operation = false;

    /**
     * @brief Used to initialize the service servers.
     */
    ros::NodeHandle node_handle;

    /**
     * @brief The servers which advertise the operations.
     */
    ros::ServiceServer take_off_server, travel_server, explore_server, interact_server, land_server;

    /**
     * @brief Used to give completion calls of operations.
     */
    ros::ServiceClient operation_completion_client;

    /**
     * @brief Service handler for the take off service.
     *
     * @param request The take off request.
     * @param response The take off response.
     *
     * @return true When the service call has been handled.
     */
    bool take_off(fluid::TakeOff::Request& request, fluid::TakeOff::Response& response);

    /**
     * @brief Service handler for the travel service.
     *
     * @param request The travel request.
     * @param response The travel response.
     *
     * @return true When the service call has been handled.
     */
    bool travel(fluid::Travel::Request& request, fluid::Travel::Response& response);

    /**
     * @brief Service handler for the explore service.
     *
     * @param request The explore request.
     * @param response The explore response.
     *
     * @return true When the service call has been handled.
     */
    bool explore(fluid::Explore::Request& request, fluid::Explore::Response& response);

    /**
     * @brief Service handler for the interact service.
     *
     * @param request The interact request.
     * @param response The interact response.
     *
     * @return true When the service call has been handled.
     */
    bool interact(fluid::Interact::Request& request, fluid::Interact::Response& response);

    /**
     * @brief Service handler for the land service.
     *
     * @param request The land request.
     * @param response The land response.
     *
     * @return true When the service call has been handled.
     */
    bool land(fluid::Land::Request& request, fluid::Land::Response& response);

    /**
     * @brief Will check if the operation to @p target_operation_identifier is valid and update the
     * #operation_execution_queue and #current_operation if it is.
     *
     * @param target_operation_identifier The target operation.
     * @param execution_queue The operation execution queue for the operation.
     *
     * @return Response based on the result of the attempt.
     */
    Response attemptToCreateOperation(const OperationIdentifier& target_operation_identifier,
                                      const std::list<std::shared_ptr<Operation>>& execution_queue);
    /**
     * @brief Retrieves the operation identifier from @p operation_ptr
     *
     * @param operation_ptr The operation.
     *
     * @return operation identifier if operation_ptr is not nullptr, #OperationIdentifier::UNDEFINED if else.
     */
    OperationIdentifier getOperationIdentifierForOperation(std::shared_ptr<Operation> operation_ptr);

    /**
     * @brief Validates if a given operation to @p target_operation_identifier is valid from @p
     * current_operation_identifier.
     *
     * @param current_operation_identifier The current operation identifier.
     * @param target_operation_identifier The target operation identifier of the operation.
     *
     * @return true if the operation is valid.
     */
    bool isValidOperation(const OperationIdentifier& current_operation_identifier,
                          const OperationIdentifier& target_operation_identifier) const;

    /**
     * @brief Performs operation transition between @p current_operation_ptr and @p target_operation_ptr.
     *
     * @param current_operation_ptr The current operation.
     * @param target_operation_ptr The target operation.
     *
     * @return The new operation (@p target_operation_ptr).
     */
    std::shared_ptr<Operation> performOperationTransition(std::shared_ptr<Operation> current_operation_ptr,
                                                          std::shared_ptr<Operation> target_operation_ptr);

   public:
    /**
     * @brief The configuration of the fluid singleton.
     */
    const FluidConfiguration configuration;

    /**
     * @brief Initializes the Fluid singleton with a @p configuration.
     *
     * @param configuration The configuration of the Fluid singleton.
     *
     * @note Will only actually initialize if #instance_ptr is not nullptr.
     */
    static void initialize(const FluidConfiguration configuration);

    /**
     * @return The Fluid singleton instance.
     */
    static Fluid& getInstance();

    /**
     * @return The status publisher.
     */
    std::shared_ptr<StatusPublisher> getStatusPublisherPtr();

    /**
     * @brief Runs the operation macine.
     */
    void run();
};

#endif