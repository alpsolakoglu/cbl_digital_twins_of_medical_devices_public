
// #include "interfaces/IState.h"
// #include "interfaces/IPositionable.h"
// #include "controllers/states/AwaitingStart.h"

// namespace DT
// {
//     ServoController::AxisController(IPositionable &positionable)
//         : m_positionable(positionable)
//     {
//         changeState(new AwaitingStart(this)); // Initialize with the AwaitingStart state
//     }

//     void AxisController::changeState(IState *newState)
//     {
//         if (newState == nullptr)
//         {
//             return; // Do not change state if the new state is null
//         }

//         if (m_currentState != nullptr)
//         {
//             m_currentState->onExit(); // Call exit method of the current state
//             delete m_currentState;    // Clean up the current state
//         }

//         m_currentState = newState; // Set the new state
//         m_currentState->onEnter(); // Call enter method of the new state
//     }

//     void AxisController::update()
//     {
//         m_currentState->onUpdate(); // Call update method of the current state
//     }

//     AxisController::~AxisController()
//     {
//         delete m_currentState; // Clean up the current state
//     }
// }
