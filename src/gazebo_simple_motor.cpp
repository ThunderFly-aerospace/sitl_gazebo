#ifndef _SIMPLE_MOTOR_PLUGIN_HH_
#define _SIMPLE_MOTOR_PLUGIN_HH_

#include <stdio.h>

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <rotors_model/motor_model.hpp>
#include "CommandMotorSpeed.pb.h"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "MotorSpeed.pb.h"
#include "Float.pb.h"

#include "common.h"

using namespace ignition::math;

typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;

#define DEBUG_CONST 500

namespace gazebo
{

  class SimpleMotorPlugin : public ModelPlugin
  {

    public: 
	SimpleMotorPlugin() {}

	~SimpleMotorPlugin()
	{
		updateConnection->~Connection();
	}

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Just output a message for now
		gzdbg << " SimpleMotorPlugin is attach to model[" << _model->GetName() << "]\n";

		this->base_link = _model->GetChildLink(_sdf->GetElement("base_link")->Get<std::string>());
		this->force_dir = _sdf->Get<ignition::math::Vector3d>("force_dir");
        this->force_pos = _sdf->Get<ignition::math::Vector3d>("force_pos");

        this->coef = _sdf->GetElement("coef")->Get<double>();

        std::string commandTopic = _sdf->GetElement("commandSubTopic")->Get<std::string>();
        this->motor_number=_sdf->GetElement("motorNumber")->Get<int>();

		counter=0;

		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			std::bind(&SimpleMotorPlugin::OnUpdate, this));

        node_handle_ = transport::NodePtr(new transport::Node());
        node_handle_->Init("");

        command_sub_ = node_handle_->Subscribe<mav_msgs::msgs::CommandMotorSpeed>("~/" + _model->GetName() + commandTopic,
            &SimpleMotorPlugin::VelocityCallback, this);

	}

	virtual void OnUpdate()
	{
		//debuging loop
        counter++;
		if(counter>DEBUG_CONST)
		{
			counter=0;
		}

        Vector3d force=this->coef*this->motor_thortle*this->force_dir;
        base_link->AddLinkForce(force,force_pos);

		//debuging loop
		if(counter==DEBUG_CONST)
		{
            //gzdbg << "Thortle: " << force << "\n";
		}
	}

    void VelocityCallback(CommandMotorSpeedPtr &rot_velocities) 
    {
       this->motor_thortle = rot_velocities->motor_speed(motor_number);
    }
	
	private:
		event::ConnectionPtr updateConnection;
        transport::SubscriberPtr command_sub_;
        transport::NodePtr node_handle_;
		int counter; 
		physics::LinkPtr base_link;
        Vector3d force_pos;
        Vector3d force_dir;
        int motor_number;
        double motor_thortle;
        double coef;

  };

  GZ_REGISTER_MODEL_PLUGIN(	SimpleMotorPlugin )
}
#endif
