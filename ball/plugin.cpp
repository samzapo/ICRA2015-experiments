/*****************************************************************************
 * Controller for LINKS robot
 ****************************************************************************/

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>


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
       std::cerr << ">> start Plugin: Load(.)";
       // Store the pointer to the model
       this->model = _parent;

       // store the pointer to the world
       world = model->GetWorld();

       // get the sensor
         std::string sensor_name("contact");
         std::cerr << "initing sensor : " << sensor_name;
         contacts.push_back(
               boost::dynamic_pointer_cast<sensors::ContactSensor>(
                 sensors::SensorManager::Instance()->GetSensor(sensor_name)
               ));
         contacts[0]->SetActive(true);

       this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
           boost::bind(&ControllerPlugin::Update, this)));

       // SET UP INITIAL CONDITIONS OF MODEL

       //math::Pose p(0,0,0.5,0,0,0);
       //model->SetWorldPose(p);
       //model->SetLinkWorldPose(p,"sphere::link");

       std::cerr << "<< end Plugin: Load(.)";
     }

   public: void Update()
   {
       // Control Robot
       std::cerr << ">> start Plugin: Update(.)";
       static double last_time = -0.001;
       double t = world->GetSimTime().Double();
       double dt = t - last_time;
       last_time = t;

       std::cerr << "**Starting Sensor**";

         std::cerr << "Contact Sensor ID: " << contacts[0]->GetId();
         std::cerr << "Contact Sensor Collision Name: " << contacts[0]->GetCollisionName(0);
         std::cerr << "Contact Sensor Topic: " << contacts[0]->GetTopic();

         const msgs::Contacts& c = contacts[0]->GetContacts();

         if(c.contact_size() > 0){
           for (unsigned i = 0; i < c.contact_size(); i++)
           {
             std::string c1 = c.contact(i).collision1();
             std::string c2 = c.contact(i).collision2();

             std::cerr << "Collision between[" << c1
                       << "] and [" << c2 << "]\n";

             for (unsigned j = 0; j < c.contact(i).position_size(); ++j)
             {
               std::cerr << j << "   Position: ["
                         << c.contact(i).position(j).x() << " "
                         << c.contact(i).position(j).y() << " "
                         << c.contact(i).position(j).z() << "]\n";
               std::cerr << "   Normal: ["
                         << c.contact(i).normal(j).x() << " "
                         << c.contact(i).normal(j).y() << " "
                         << c.contact(i).normal(j).z() << "]\n";
               std::cerr << "   Depth:" << c.contact(i).depth(j) << "\n";

                // Add contact point to this end effector
                //c.contact(i).wrench(j).body_1_wrench().force().x();
              }
           }
         }

       // Apply a small linear velocity to the model.
       std::cerr << "<< end Plugin: Update(.)";
     }
     };

   // Register this plugin with the simulator
   GZ_REGISTER_MODEL_PLUGIN(ControllerPlugin)
 }
