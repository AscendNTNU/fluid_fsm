/**
 * @file operation_identifier.h
 */

#ifndef OPERATION_IDENTIFIER_H
#define OPERATION_IDENTIFIER_H

#include <map>
#include <string>

const std::string ARDUPILOT_MODE_GUIDED = "GUIDED";
const std::string ARDUPILOT_MODE_LAND = "LAND"; //cahnged from AUTO.LAND to LAND for ardupilot  

/**
 * @brief Represents the different operations.
 */
enum class OperationIdentifier {
    // Null is here to make the bredth first search possible in operation graph
    TAKE_OFF,
    HOLD,
    EXPLORE,
    TRAVEL,
    LAND,
    INTERACT,
    UNDEFINED
};

/**
 * @brief Retrieves the Ardupilot mode for a given @p operation_identifier.
 *
 * @param operation_identifier The operation identifier to get the Ardupilot mode for.
 *
 * @return The Ardupilot mode.
 */
std::string getArdupilotModeForOperationIdentifier(const OperationIdentifier& operation_identifier);

/**
 * @brief Get a string from the @p operation_identifier.
 *
 * @param operation_identifier The operation identifier.
 *
 * @return The operation identifier represented in a string.
 */
std::string getStringFromOperationIdentifier(const OperationIdentifier& operation_identifier);

#endif
