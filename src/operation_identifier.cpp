#include "operation_identifier.h"

std::string getArdupilotModeForOperationIdentifier(const OperationIdentifier& operation_identifier) {
    switch (operation_identifier) {
        case OperationIdentifier::LAND:
            return ARDUPILOT_MODE_LAND;
        default:
            return ARDUPILOT_MODE_GUIDED;
    }
}

std::string getStringFromOperationIdentifier(const OperationIdentifier& operation_identifier) {
    switch (operation_identifier) {
        case OperationIdentifier::TAKE_OFF:
            return "TAKE_OFF";
        case OperationIdentifier::HOLD:
            return "HOLD";
        case OperationIdentifier::EXPLORE:
            return "EXPLORE";
        case OperationIdentifier::TRAVEL:
            return "TRAVEL";
        case OperationIdentifier::LAND:
            return "LAND";
        case OperationIdentifier::INTERACT:
            return "INTERACT";
        case OperationIdentifier::UNDEFINED:
            return "UNDEFINED";
    }
    return ""; //to avoid warning
}
