/**
 * @file operation.h
 */

#ifndef OPERATION_H
#define OPERATION_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>

#include "operation_identifier.h"
#include "type_mask.h"

/**
 * @brief Interface for operations within the finite state machine.
 */
class Operation {
   private:
    /**
     * @brief Gets the current pose.
     */
    ros::Subscriber pose_subscriber;

    /**
     * @brief Current pose.
     */
    geometry_msgs::PoseStamped current_pose;

    /**
     * @brief Callback for current pose.
     *
     * @param pose Pose retrieved from the callback.
     */
    void poseCallback(const geometry_msgs::PoseStampedConstPtr pose);

    /**
     * @brief Gets the current twist.
     */
    ros::Subscriber twist_subscriber;

    /**
     * @brief Current twist.
     */
    geometry_msgs::TwistStamped current_twist;

    /**
     * @brief Callback for current twist.
     *
     * @param twist Twist retrieved from the callback.
     */
    void twistCallback(const geometry_msgs::TwistStampedConstPtr twist);

    
    /**
     * @brief Current acceleration.
     */
    geometry_msgs::Vector3 current_accel;

    /**
     * @brief Determines whether this operation is a operation we can be at for longer periods of time. E.g. hold or
     * land.
     */
    const bool steady;

    
    /**
     * @brief Allow publishing positionTarget setpoints.
     */
    bool autoPublish;


   protected:

   /**
    * @brief Total operation count
    * 
    */
   static unsigned long int operation_total;

   /**
    * @brief Operation ID
    * 
    */
    
   const unsigned long int operation_tag = ++operation_total;

    /**
     * @brief Rate at which the operation is run
     *
     */
    int rate_int;

    /**
     * @brief Publishes setpoints.
     *
     */
    ros::Publisher setpoint_publisher;

    /**
     * @brief Used to construct the subscribers.
     */
    ros::NodeHandle node_handle;

    /**
     * @brief The setpoint.
     */
    mavros_msgs::PositionTarget setpoint;

    /**
     * @brief Publishes the setpoint.
     */
    void publishSetpoint();

    /**
     * @return true if the operation has finished its necessary tasks.
     */
    virtual bool hasFinishedExecution() const = 0;

    /**
     * @brief Initializes the operation.
     */
    virtual void initialize() {}

    /**
     * @brief Updates the operation logic.
     */
    virtual void tick() {}

    /**
     * @return The current pose.
     */
    geometry_msgs::PoseStamped getCurrentPose() const;

    /**
     * @return The current twist.
     */
    geometry_msgs::TwistStamped getCurrentTwist() const;

    /**
     * @return The current twist.
     */
    geometry_msgs::Vector3 getCurrentAccel() const;

    /**
     * @return The current yaw.
     */
    float getCurrentYaw() const;


    /**
     * @brief Estimate the acceleration of the drone from its orientation.
     * 
     * @param orientation The orientation of the drone as from poseCallback.
     * 
     * @return The estimation of the drone acceleration.
     */
    geometry_msgs::Vector3 orientation_to_acceleration(geometry_msgs::Quaternion orientation);

   public:
    /**
     * @brief The identifier for this operation.
     */
    const OperationIdentifier identifier;

    /**
     * @brief Get operation tag
     * 
     */
    const unsigned long int& getOperationTag(){return operation_tag;};

    /**
     * @brief Get total operation count
     * 
     */
    unsigned long int* getOperationTotalPtr(){return &operation_total;};

    /**
     * @brief Constructs a new operation.
     *
     * @param identifier The identifier of the operation.
     * @param steady Whether the operation is steady, it can be executed for longer periods of time without
     * consequences.
     * @param should_publish_setpoints Allow to prevent the operation publishing position setpoins
     */
    Operation(const OperationIdentifier& identifier, const bool& steady, const bool& autoPublish);

    /**
     * @brief Performs the loop for executing logic within this operation.
     *
     * @param should_tick               Called each tick, makes it possible to abort operations in the midst of an
     *                                  execution.
     * @param should_halt_if_steady     Will halt at this operation if it's steady, is useful
     *                                  if we want to keep at a certain operation for some time, e.g. #LandOperation
     *                                  or #HoldOperation.
     */
    virtual void perform(std::function<bool(void)> should_tick, bool should_halt_if_steady);

    /**
     * The #Fluid class has to be able to e.g. set the current pose if we transition to a operation which
     * requires to initially know where we are, e. g. land or take off. In that case we can execute the operation from
     * the current pose, and we don't have to wait for the pose callback and thus halt the system.
     */
    friend class Fluid;
};
#endif
