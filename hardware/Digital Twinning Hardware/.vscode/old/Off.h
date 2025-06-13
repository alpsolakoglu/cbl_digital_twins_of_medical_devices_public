#include "interfaces/IState.h"

namespace DT
{
    class Off : public IState
    {
    public:
        Off(AxisController& axisController);
        
        /// Called when the controller enters this state
        void onEnter() override;

        /// Called periodically (e.g. in a run loop) while in this state
        void onUpdate() override;

        /// Called when the controller is leaving this state
        void onExit() override;
    };
}
