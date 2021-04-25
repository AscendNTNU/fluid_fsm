/**
 * @file move_operation_global.h
 */

#ifndef MOVE_OPERATION_GLOBAL_H
#define MOVE_OPERATION_GLOBAL_H

#include "operation.h"


class MoveOperationGlobal : public Operation{
    public:
    explicit MoveOperationGlobal(const OperationIdentifier& operation_identifier,
                        const std::vector<mavros_msgs::GlobalPositionTarget>& global_path, const double& speed,
                        const double& position_threshold, const double& velocity_threshold,
                        const double& max_angle);

    bool hasFinishedExecution() const override;

    virtual void tick() override;
    virtual void initialize() override;


    protected:
    std::vector<mavros_msgs::GlobalPositionTarget>::iterator current_global_setpoint_iterator;
    std::vector<mavros_msgs::GlobalPositionTarget> global_path;


    

    private:

    const double position_threshold;
    const double velocity_threshold;
    const double speed;
    const double max_angle;
    bool been_to_all_points = false;
    bool update_setpoint = false;




};
//"as-the-crow-flies" distance
double globalDistance(double lat1, double long1, double lat2, double long2);
#endif
