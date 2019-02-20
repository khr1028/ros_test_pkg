#include "test_component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
namespace tway {
TestComponent::TestComponent(std::string const& name) : TaskContext(name)
{
    
}

bool TestComponent::configureHook() {

    std::cout << "TestComponent configured !" <<std::endl;
    return true;
}

bool TestComponent::startHook() {
    std::cout << "TestComponent started !" <<std::endl;
    return true;
}

void TestComponent::updateHook() {
    std::cout << "TestComponent executes updateHook !" <<std::endl;
}

void TestComponent::stopHook() {
    std::cout << "TestComponent executes stopping !" <<std::endl;
}

void TestComponent::cleanupHook() {
    std::cout << "TestComponent cleaning up !" <<std::endl;
}

bool TestComponent::setValue(double value)
{
   }

}
/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(TestComponent)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */


//ORO_CREATE_COMPONENT(tway::TestComponent)
ORO_CREATE_COMPONENT_TYPE()
ORO_LIST_COMPONENT_TYPE(tway::TestComponent)