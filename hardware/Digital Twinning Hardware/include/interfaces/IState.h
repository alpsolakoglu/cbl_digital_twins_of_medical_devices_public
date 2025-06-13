// #ifndef STATE_H
// #define STATE_H

// #include "controllers/AxisController.h"

// namespace DT {
//     class IState
//     {
//     private:
//         AxisController& m_axisController; // Reference to the AxisController managing this state
//     public:    
//         IState(AxisController& axisController) : m_axisController(axisController) {};

//         /// Called when the controller enters this state
//         virtual void onEnter() = 0;

//         /// Called periodically (e.g. in a run loop) while in this state
//         virtual void onUpdate() = 0;

//         /// Called when the controller is leaving this state
//         virtual void onExit() = 0;
//     };
// }

// #endif