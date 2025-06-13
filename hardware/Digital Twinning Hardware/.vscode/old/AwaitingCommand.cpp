// #include "controllers/states/AwaitingCommand.h"

// #include <Arduino.h>

// namespace DT
// {
//     AwaitingCommand::AwaitingCommand(AxisController &axisController) : IState(axisController) {}

//     void AwaitingCommand::onEnter()
//     {
//         m_axisController.set
//         return true; // Successfully started the idle state
//     }

//     void AwaitingCommand::onUpdate()
//     {
//         // In idle state, we can perform periodic checks or updates if needed
//         Serial.println("Idle state is active. Waiting for commands...");
//     }
// }