
/* make with: catkin_make run_tests_test_component
* or just cmake:
 mkdir buid && cd build 
 cmake .. 
 make all
 make run_tests OR
 make run_tests_test_component
*/

#include "../src/test_component.hpp"

#include <ocl/DeploymentComponent.hpp>

#include <rtt/os/startstop.h>
#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <rtt/deployment/ComponentLoader.hpp>
#include <rtt/scripting/Scripting.hpp>

#include <gtest/gtest.h>


boost::shared_ptr<OCL::DeploymentComponent> deployer;
boost::shared_ptr<RTT::Scripting> scripting_service;


TEST(BasicTest, basicHooks)
{
    
    TestComponent component("test_comp");
    ASSERT_TRUE(component.configure());
    ASSERT_TRUE(component.start());
    ASSERT_TRUE(component.stop());
    ASSERT_TRUE(component.cleanup());

    ASSERT_TRUE(component.configure());

}


TEST(BasicTest, import) 
{
    // Import the component
    boost::shared_ptr<RTT::ComponentLoader> comp_loader = 
        RTT::ComponentLoader::Instance();
    ASSERT_TRUE(comp_loader->import("test_component","" ));
    RTT::TaskContext * test_comp = comp_loader->loadComponent(
                                    "test","TestComponent" );

    RTT::base::PortInterface * port = test_comp->getPort("double_outport");
}



// Declare another test
TEST(SpecificTest, operations)
{
// <test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    // Initialize Orocos
    __os_init(argc, argv);

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}