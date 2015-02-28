/*****************************************************************************
 * "Controller" for sliding/rolling sphere example 
 ****************************************************************************/
#include <Moby/EventDrivenSimulator.h>
#include <Moby/RCArticulatedBody.h>
#include <Moby/GravityForce.h>
#include <Ravelin/Pose3d.h>
#include <Ravelin/Vector3d.h>
#include <Ravelin/VectorNd.h>
#include <fstream>
#include <stdlib.h>
#include <boost/algorithm/string.hpp>

using boost::shared_ptr;
using namespace Ravelin;
using namespace Moby;

std::map<std::string, Moby::RigidBodyPtr> objects;
boost::shared_ptr<EventDrivenSimulator> sim;
boost::shared_ptr<GravityForce> grav;

// setup simulator callback
void post_step_callback(Simulator* sim)
{
}

/// plugin must be "extern C"
extern "C" {
void init(void* separator, const std::map<std::string, Moby::BasePtr>& read_map, double time)
{
  // get a reference to the EventDrivenSimulator instance
  for (std::map<std::string, Moby::BasePtr>::const_iterator i = read_map.begin();
       i !=read_map.end(); i++)
  {
    std::vector<std::string> model_name;
    boost::split(model_name, i->first, boost::is_any_of("::"));
    // Find the simulator reference
    if (!sim)
      sim = boost::dynamic_pointer_cast<EventDrivenSimulator>(i->second);
    if (model_name.front().compare("sdf") == 0)
      objects[i->first] = boost::dynamic_pointer_cast<RigidBody>(i->second);
    if (!grav)
      grav = boost::dynamic_pointer_cast<GravityForce>(i->second);
  }

  sim->post_step_callback_fn = &post_step_callback;


  objects.at("sdf::1::book")->set_pose(Ravelin::Pose3d(
          Ravelin::Quatd::identity(),Ravelin::Origin3d(0.1,0.1,1), Moby::GLOBAL
        )
      );
  objects.at("sdf::2::book")->set_pose(Ravelin::Pose3d(
          Ravelin::Quatd::identity(),Ravelin::Origin3d(0.1,0.1,1.03), Moby::GLOBAL
        )
      );
  objects.at("sdf::3::book")->set_pose(Ravelin::Pose3d(
          Ravelin::Quatd::identity(),Ravelin::Origin3d(0.1,0.1,1.06), Moby::GLOBAL
        )
      );
  objects.at("sdf::4::book")->set_pose(Ravelin::Pose3d(
          Ravelin::Quatd::identity(),Ravelin::Origin3d(0.1,0.1,1.09), Moby::GLOBAL
        )
      );
  objects.at("sdf::5::book")->set_pose(Ravelin::Pose3d(
          Ravelin::Quatd::identity(),Ravelin::Origin3d(0.1,0.1,1.12), Moby::GLOBAL
        )
      );
  objects.at("sdf::6::book")->set_pose(Ravelin::Pose3d(
          Ravelin::Quatd::identity(),Ravelin::Origin3d(0.1,0.1,1.15), Moby::GLOBAL
        )
      );
  
}
} // end extern C
/*****************************************************************************
 * Controller for LINKS robot
 ****************************************************************************/

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <math.h>

 namespace gazebo
 {
   class ControllerPlugin : public ModelPlugin
   {
     private: physics::WorldPtr world;
     private: std::vector<sensors::ContactSensorPtr> contacts;

     // Pointer to the model
     private: physics::ModelPtr model;
     // Pointer to the update event connection
     private: std::vector<event::ConnectionPtr> connections;

     gazebo::transport::NodePtr receiver;
     std::vector<gazebo::transport::SubscriberPtr> subs;

     public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
     {
       std::cerr << ">> start Plugin: Load(.)" << std::endl;
       // Store the pointer to the model
       this->model = _parent;

       // store the pointer to the world
       world = model->GetWorld();
       // SET UP INITIAL CONDITIONS OF MODEL
      

       std::cerr << "<< end Plugin: Load(.)" << std::endl;
     }

   public: void Update()
   {
   }
 };

   // Register this plugin with the simulator
   GZ_REGISTER_MODEL_PLUGIN(ControllerPlugin)
 }
