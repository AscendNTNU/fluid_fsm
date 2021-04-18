/**
 * @file hold_operation.h
 */

#ifndef HOLD_OPERATION_H
#define HOLD_OPERATION_H

#include "operation.h"
#include "util.h"

/**
 * @brief Operation representing the drone hovering.
 */
class HoldOperation : public Operation {
   public:
    /**
     * @brief Sets up the hold operation.
     */
    explicit HoldOperation();

    /**
     * @return true When the drone is hovering still at a given position.
     */
    bool hasFinishedExecution() const override;

    /**
     * @brief Sets up the setpoint to the current position.    
     */
    void initialize() override;
};

#endif
