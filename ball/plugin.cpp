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
       // Trial variables
       static int trial_num = -1;
       static double r = 0, theta = 2.0*M_PI;

       // Control print ball state
       std::cerr << ">> start Plugin: Update(.)" << std::endl;
       static double last_time = -0.001;
       double t = world->GetSimTime().Double();
       double dt = t - last_time;
       last_time = t;

       // Start a new sample
       static double start_time = t;
       static bool inited = false;
       if(t-start_time > 2.0 || !inited){
         inited = true;
         // Increment initial state
         if(theta >= 2.0*M_PI){
           theta = 0;
           r += 1.0;
         } else {
           theta += M_PI_2*0.1;
         }

         // End once we've sampled enough end
         assert(r <= 5.0);

         start_time = t;
         trial_num++;

         // Reset world state
         math::Pose p(0,0,1,0,0,0);
         model->SetWorldPose(p);
         model->SetLinkWorldPose(p,"sphere::link");

         // Set new initial velocity
         math::Vector3 avel(0,0,0),
                       lvel(r*cos(theta),r*sin(theta),0);

         model->SetAngularVel(avel);
         model->SetLinearVel(lvel);
       }

       /*
       std::cerr << "**Starting Sensor**" << std::endl;

         std::cerr << "Contact Sensor ID: " << contacts[0]->GetId() << std::endl;
         std::cerr << "Contact Sensor Collision Name: " << contacts[0]->GetCollisionName(0) << std::endl;
         std::cerr << "Contact Sensor Topic: " << contacts[0]->GetTopic() << std::endl;

         const msgs::Contacts& c = contacts[0]->GetContacts();

         if(c.contact_size() > 0){
           for (unsigned i = 0; i < c.contact_size(); i++)
           {
             std::string c1 = c.contact(i).collision1();
             std::string c2 = c.contact(i).collision2();

             std::cerr << "Collision between[" << c1
                       << "] and [" << c2 << "]" << std::endl;

             for (unsigned j = 0; j < c.contact(i).position_size(); ++j)
             {
//               std::cerr << j << "   Position: ["
//                         << c.contact(i).position(j).x() << " "
//                         << c.contact(i).position(j).y() << " "
//                         << c.contact(i).position(j).z() << "]\n";
//               std::cerr << "   Normal: ["
//                         << c.contact(i).normal(j).x() << " "
//                         << c.contact(i).normal(j).y() << " "
//                         << c.contact(i).normal(j).z() << "]\n";
//               std::cerr << "   Depth:" << c.contact(i).depth(j) << "\n";


                // Add contact point to this end effector
                std::cerr << "contact_force = [ "
                          << c.contact(i).wrench(j).body_1_wrench().force().x() << " , "
                          << c.contact(i).wrench(j).body_1_wrench().force().y() << " , "
                          << c.contact(i).wrench(j).body_1_wrench().force().z() << " ]';" << std::endl;
              }
           }
         }

         */

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

       std::cerr << "<< end Plugin: Update(.)" << std::endl;
     }
     };

   // Register this plugin with the simulator
   GZ_REGISTER_MODEL_PLUGIN(ControllerPlugin)
 }
