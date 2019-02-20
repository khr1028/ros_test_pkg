#ifndef OROCOS_TEST_COMPONENT_COMPONENT_HPP
#define OROCOS_TEST_COMPONENT_COMPONENT_HPP

#include <rtt/RTT.hpp>
namespace tway {
class TestComponent : public RTT::TaskContext{
    public:
        TestComponent(std::string const& name);
        virtual void ControlLoop() = 0;
        bool configureHook();
        bool startHook();
        void updateHook();
        void stopHook();
        void cleanupHook();

        bool setValue(double);
    private:
};
}
#endif
