/**
 * @file move_state.h
 */

#ifndef MOVE_STATE_H
#define MOVE_STATE_H

#include "state.h"

/**
 * @brief Serves as the base for move states such as #ExploreState and #TravelState.
 */
class MoveState : public State {
   private:
    /**
     * @brief The threshold for when the drone is within a given setpoint.
     */
    const double position_threshold;

    /**
     * @brief The highest velocity the drone can have in order to signal that it can move to the next setpoint in the 
     *        #path.
     */
    const double velocity_threshold;

    /**
     * @brief The speed the drone should move at.
     */
    const double speed;

    /**
     * @brief Convenicene variable representing that the drone has been through all the setpoints in the #path.
     */
    bool been_to_all_points = false;

   protected:
    /**
     * @brief Flag for forcing the state to update the setpoint even the drone hasn't reached the setpoint.
     */
    bool update_setpoint = false;

    /**
     * @brief The current setpoint.
     */
    std::vector<geometry_msgs::Point>::iterator current_setpoint_iterator;

    /**
     * @brief List of the setpoints.
     */
    std::vector<geometry_msgs::Point> path;

    /**
     * @brief Sets up the move state.
     * 
     * @param state_identifier The state identifier. 
     * @param path The path of the state.
     * @param speed The speed at which to move. 
     * @param position_threshold The setpoint distance threshold. 
     * @param velocity_threshold The velocity threshold. 
     */
    explicit MoveState(const StateIdentifier& state_identifier,
                       const std::vector<geometry_msgs::Point>& path,
                       const double& speed,
                       const double& position_threshold,
                       const double& velocity_threshold) : State(state_identifier, false, true),
                                                           path(path),
                                                           speed(speed),
                                                           position_threshold(position_threshold),
                                                           velocity_threshold(velocity_threshold) {}

   public:
    /**
     * @return true When the drone has been through the whole #path.
     */
    bool hasFinishedExecution() const override;

    /**
     * @brief Checks where the drone is at a given point and updates the #current_setpoint_iterator if
     *        the drone has reached a setpoint.
     */
    virtual void tick() override;

    /**
     * @brief Sets up the #current_setpoint_iterator and sets the #speed at which to move.
     */
    virtual void initialize() override;
};

#endif
