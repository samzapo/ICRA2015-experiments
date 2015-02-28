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

using boost::shared_ptr;
using namespace Ravelin;
using namespace Moby;

Moby::RigidBodyPtr sphere;
boost::shared_ptr<EventDrivenSimulator> sim;
boost::shared_ptr<GravityForce> grav;

// Test Parameters
double trial_time = 5.0;
double theta_res = 0.5;
double r_res = 1.0;

// Trial variables
int trial_num = -1;
double r = 5, theta = 0;
double start_time = 0;

bool get_new_state(double t, double& x, double& y){
  static bool inited = false;
#ifdef NOT_THIS
  // Start a new sample
  if(t-start_time > trial_time || !inited){
    inited = true;
    // Increment initial state
    if(theta >= 2.0*M_PI){
      theta = 0;
      r += r_res;
  } else {
    theta += M_PI_2*theta_res;
  }

  // End once we've sampled enough end
  if(r > 5.0);
    throw std::runtime_error("radial vel exceeded 5 m/2");
#endif
  // Start a new sample
  if(t-start_time > trial_time){
    // Increment initial state
    theta += M_PI_2*theta_res;
    if(theta >= 2.0*M_PI)
      throw std::runtime_error("Finished all directions of contact vel");
    
    start_time = t;
    trial_num++;
    x = r*cos(theta);
    y = r*sin(theta);
    return true;
  }

  x = r*cos(theta);
  y = r*sin(theta);

  if(!inited){
    inited = true;
    return true;
  }
  return false;
}

// setup simulator callback
void post_step_callback(Simulator* sim)
{
  const unsigned X = 0, Y = 1, Z = 2;

  // setup the sphere radius
  const double R = 1.0;

  ///////////////////////////////////////////////
  //
  //

       
       // Control print ball state
       static double last_time = -0.001;
       double t = sim->current_time;
       double dt = t - last_time;
       last_time = t;

       double x,y;
       if(get_new_state(t,x,y)){
         // Reset world state
         Ravelin::Pose3d p(Ravelin::Quatd::identity(),Ravelin::Origin3d(0,0,1),Moby::GLOBAL);
         sphere->set_pose(p);;

         std::cerr << "r = " <<  r << " ; theta = " << theta << std::endl;
         std::cerr << "x = " <<  x << " ; y = " << y << std::endl;
         // Set new initial velocity
         Ravelin::Vector3d avel(0,0,0),
                         lvel(x,y,0);

         Ravelin::SVelocityd vel;

         vel.set_angular(avel);
         vel.set_linear(lvel);

         sphere->set_velocity(vel);

       }
         
       /*
       Ravelin::Vector3d avel,lvel,cvel;

       Ravelin::SVelocityd vel;
       vel = sphere->get_velocity();
       Ravelin::SVelocityd gvel 
         = Ravelin::Pose3d::transform(Moby::GLOBAL,vel);
       avel = gvel.get_angular();
       lvel = gvel.get_linear();

       Ravelin::Vector3d k(0,0,1);

       cvel = lvel + Ravelin::Vector3d::cross(avel,-k);

       std::cout
           << trial_num << " "
           << t-start_time << " "
           << r       << " "
           << theta   << " "
           << cvel[X] << " "
           << cvel[Y] << std::endl;
  
           */
      

  //
  //
  ///////////////////////////////////////////////

  // get the bottom of the sphere
  Transform3d wTs = Pose3d::calc_relative_pose(sphere->get_pose(), GLOBAL);

  shared_ptr<Pose3d> Pbot(new Pose3d);  
  Pbot->rpose = GLOBAL;
  Pbot->x = wTs.x;
  Pbot->x[Z] -= R;

  // get the velocity of the sphere at the contact point
  SVelocityd v = sphere->get_velocity();
  Transform3d botTv = Pose3d::calc_relative_pose(v.pose, Pbot);
  SVelocityd xd = botTv.transform(v);
  Vector3d linear = xd.get_linear();

       std::cout
           << trial_num << " "
           << t-start_time << " "
           << r       << " "
           << theta   << " "
           << linear[X] << " "
           << linear[Y] << std::endl;
/*
  SVelocityd v = sphere->get_velocity();
  Origin3d xd(v.get_linear());
  Origin3d omega(v.get_angular());
  Origin3d s(1.0, 0.0, 0.0);
  Origin3d t(0.0, 1.0, 0.0);
  Origin3d crosss = Origin3d::cross(-wTs.x, s);
  Origin3d crosst = Origin3d::cross(-wTs.x, t);
*/

/*
  // output the sliding velocity at the contact 
  std::ofstream out("contactv.dat", std::ostream::app);
  out << sim->current_time << " " << linear[X] << " " << linear[Y] << std::endl;
//  out << sim->current_time << " " << (s.dot(xd) + crosss.dot(omega)) << " " << (t.dot(xd) + crosst.dot(omega)) << std::endl; 
//  out << sim->current_time << " " << v[3] << " " << v[4] << " " << v[5] << " " << v[0] << " " << v[1] << " " << v[2] << std::endl;
  out.close();

  out.open("ke.dat", std::ostream::app);
  out << sim->current_time << " " << sphere->calc_kinetic_energy() << std::endl;
  out.close();
*/
}

/// plugin must be "extern C"
extern "C" {

void init(void* separator, const std::map<std::string, Moby::BasePtr>& read_map, double time)
{
  const unsigned Z = 2;

  // wipe out contactv
  std::ofstream out("contactv.dat");
  out.close();
  out.open("ke.dat");
  out.close();

  // get a reference to the EventDrivenSimulator instance
  for (std::map<std::string, Moby::BasePtr>::const_iterator i = read_map.begin();
       i !=read_map.end(); i++)
  {
    // Find the simulator reference
    if (!sim)
      sim = boost::dynamic_pointer_cast<EventDrivenSimulator>(i->second);
    if (i->first == "::sphere")
      sphere = boost::dynamic_pointer_cast<RigidBody>(i->second);
    if (!grav)
      grav = boost::dynamic_pointer_cast<GravityForce>(i->second);
  }

  sim->post_step_callback_fn = &post_step_callback;
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

       // get the sensor
         std::string sensor_name("contact");
         std::cerr << "initing sensor : " << sensor_name << std::endl;
         contacts.push_back(
               boost::dynamic_pointer_cast<sensors::ContactSensor>(
                 sensors::SensorManager::Instance()->GetSensor(sensor_name)
               ));
         contacts[0]->SetActive(true);

       this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
           boost::bind(&ControllerPlugin::Update, this)));

       // SET UP INITIAL CONDITIONS OF MODEL

       std::cerr << "<< end Plugin: Load(.)" << std::endl;
     }

   public: void Update()
   {
       // Control print ball state
       static double last_time = -0.001;
       double t = world->GetSimTime().Double();
       double dt = t - last_time;
       last_time = t;

       double x,y;
       if(get_new_state(t,x,y)){
         std::cerr << "r = " <<  r << " ; theta = " << theta << std::endl;
         // Reset world state
         math::Pose p(0,0,1,0,0,0);
         model->SetWorldPose(p);
         model->SetLinkWorldPose(p,"sphere::link");

         // Set new initial velocity
         math::Vector3 avel(0,0,0),
                       lvel(x,y,0);

         model->SetAngularVel(avel);
         model->SetLinearVel(lvel);
       }

       math::Vector3 avel,lvel,cvel;

       avel = model->GetWorldAngularVel();
       lvel = model->GetWorldLinearVel();

       math::Vector3 k(0,0,1);

       cvel = lvel + avel.Cross(-k);

       std::cout
           << trial_num << " "
           << t-start_time << " "
           << r       << " "
           << theta   << " "
           << cvel[0] << " "
           << cvel[1] << std::endl;

     }
     };

   // Register this plugin with the simulator
   GZ_REGISTER_MODEL_PLUGIN(ControllerPlugin)
 }
