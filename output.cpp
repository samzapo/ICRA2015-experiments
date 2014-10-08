/*****************************************************************************
 * Controller for LINKS robot
 ****************************************************************************/
#include <Moby/EventDrivenSimulator.h>
#include <Moby/RCArticulatedBody.h>
#include <Ravelin/Pose3d.h>
#include <Ravelin/Vector3d.h>
#include <Ravelin/VectorNd.h>

 boost::shared_ptr<Moby::EventDrivenSimulator> sim;
 Moby::RigidBodyPtr part;

void controller_callback(Moby::DynamicBodyPtr dbp, double t, void*)
{
  Ravelin::VectorNd x,xd;
  part->get_generalized_coordinates( Moby::DynamicBody::eSpatial,x);
  part->get_generalized_velocity( Moby::DynamicBody::eSpatial,xd);

  std::cout << "x = " << x << std::endl;
  std::cout << "v = " << xd << std::endl;

  assert(x[0] < 1.0);
}

// ============================================================================
// ================================ CALLBACKS =================================

/// plugin must be "extern C"
extern "C" {

void init(void* separator, const std::map<std::string, Moby::BasePtr>& read_map, double time)
{
  // If use robot is active also init dynamixel controllers
  // get a reference to the EventDrivenSimulator instance
  for (std::map<std::string, Moby::BasePtr>::const_iterator i = read_map.begin();
       i !=read_map.end(); i++)
  {
    // Find the simulator reference
    if (!sim)
      sim = boost::dynamic_pointer_cast<Moby::EventDrivenSimulator>(i->second);

    // find the robot reference
    if (!part)
      part = boost::dynamic_pointer_cast<Moby::RigidBody>(i->second);
    else if(part->id.compare("PART") != 0)
        part = boost::dynamic_pointer_cast<Moby::RigidBody>(i->second);
  }

  part->controller                     = &controller_callback;

}
} // end extern C
