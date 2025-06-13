// #ifndef AXIS_CONTROLLER_H
// #define AXIS_CONTROLLER_H

// #include "interfaces/IState.h"
// #include "interfaces/IPositionable.h"

// #include <queue>

// namespace DT
// {
//     class ServoController
//     {
//     private:
//         IState* m_currentState; // Reference to the current state of the axis controller
//         std::queue<Angle> m_angleQueue; // Queue to hold angles for the axis

//         IPositionable& m_positionable;  // Reference to the positionable interface for the axis
//     public:
//         AxisController(IPositionable& positionable);

//         void changeState(IState* newState);

//         void update();
//     };

    
// }

// #endif