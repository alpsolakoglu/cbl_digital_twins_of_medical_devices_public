#ifndef CONTROLLER_STATES_H
#define CONTROLLER_STATES_H

namespace DT
{
    enum class ControllerState
    {
        START,
        CONFIGURE,
        AWAITING_COMMAND,
        EXECUTING_COMMAND,
        ERROR
    };
}

#endif