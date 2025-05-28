#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>



// Register the plugin
namespace my_gazebo_plugin
{
class MotorPlugin : public gz::sim::System,
public gz::sim::ISystemConfigure,
public gz::sim::ISystemUpdate
{
public:
MotorPlugin(){};
~MotorPlugin(){};
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &/*_eventMgr*/) override
                 {
    model_ = gz::sim::Model(_entity);
    
    joint_name_ = _sdf->Get<std::string>("joint_name", "j1").first;
    
    joint_entity_ = model_.JointByName(_ecm, joint_name_);
    if (joint_entity_ == gz::sim::kNullEntity)
    {
      std::cerr << "Joint [" << joint_name_ << "] not found in model [" << model_.Name(_ecm) << "]" << std::endl;
      return;
    }
    
    if (!_ecm.Component<gz::sim::components::JointVelocity>(joint_entity_))
    {
      _ecm.CreateComponent(joint_entity_, gz::sim::components::JointVelocity({0.0}));
    }
    
    if (!_ecm.Component<gz::sim::components::JointForceCmd>(joint_entity_))
    {
      _ecm.CreateComponent(joint_entity_, gz::sim::components::JointForceCmd({0.0}));
    }

    std::cout << "MotorPlugin loaded for joint [" << joint_name_ << "] in model [" << model_.Name(_ecm) << "]" << std::endl;
  }

  // Getting called at every simulation step
  void Update(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (joint_entity_ == gz::sim::kNullEntity)
      return;
      
    auto joint_vel_comp = _ecm.Component<gz::sim::components::JointVelocity>(joint_entity_);
    if (joint_vel_comp)
    {
      const auto &velocities = joint_vel_comp->Data();
      if (!velocities.empty())
      {
        
        double MaxCurrent = V_max / res + i_no_load; // Not getting used somewhere right now.

        double V = V_max * pwm; 
        double Current = (V - velocities[0] / Kv) / res; // from section 1.2        
        double Torque = 0; 
        
        // for turning motors.
        //////////////////////////////////////////////
        if (Current >= i_no_load)
          Torque = (Current - i_no_load) / Kv; 

        if (Current <= i_no_load)
          Torque = (Current + i_no_load) / Kv;
        //////////////////////////////////////////////


        double drag_coeff = 0.00001; // adding a virtual load.
        double loadTorque = drag_coeff * velocities[0] * velocities[0]; // Quadratic drag

        double netTorque = Torque - loadTorque;
        auto jfcComp = _ecm.Component<gz::sim::components::JointForceCmd>(joint_entity_);
        if (jfcComp)
        {
          auto &forceCmd = jfcComp->Data();
          if (!forceCmd.empty())
          {

            // converting rad/s to rpm
            double rpm = (velocities[0] * 60.0) / (2.0 * M_PI);
            forceCmd[0] = netTorque; // passing torque to the motor.
            std::cout << "velocity: " << velocities[0] << " rad/s " << std::endl;
            std::cout << "rpm: " << rpm << std::endl;
            std::cout << "torque: " << netTorque << std::endl;
          }

        }

      }
      else
      {
        std::cerr << "No velocity data available for joint [" << joint_name_ << "]" << std::endl;
      }
    }
    else
    {
      std::cerr << "JointVelocity component not found for joint [" << joint_name_ << "]" << std::endl;
    }
  }
  
  private:
  gz::sim::Model model_;
  gz::sim::Entity joint_entity_ = gz::sim::kNullEntity;
  std::string joint_name_;
  int Kv = 1000; // RPM/V
  float res = 0.090; // Ohms.
  float i_no_load = 0.5; //amps
  float V_max = 10; //volts
  float pwm = 0.1; // normalized pwm.

  
};
}

GZ_ADD_PLUGIN(
  my_gazebo_plugin::MotorPlugin,
  gz::sim::System,
  my_gazebo_plugin::MotorPlugin::ISystemConfigure,
  my_gazebo_plugin::MotorPlugin::ISystemUpdate
)


GZ_ADD_PLUGIN_ALIAS(my_gazebo_plugin::MotorPlugin, "MotorPlugin")