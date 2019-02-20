
#include "../include/real_time_task.h"
#include <rtt/Component.hpp>
namespace tway
{
    RealTimeTask::RealTimeTask(std::string const& name)
	: TaskContext(name)
    {

    }
    
}
//ORO_CREATE_COMPONENT(tway::RealTimeTask)
//ORO_CREATE_COMPONENT_TYPE()
//ORO_LIST_COMPONENT_TYPE(tway::RealTimeTask)