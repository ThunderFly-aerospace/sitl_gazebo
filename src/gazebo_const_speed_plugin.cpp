#ifndef _CONST_SPEED_PLUGIN_HH_
#define _CONST_SPEED_PLUGIN_HH_

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

  class ConstSpeedPlugin : public ModelPlugin
  {

    public: 
	ConstSpeedPlugin() {}

	~ConstSpeedPlugin()
	{
		updateConnection->~Connection();
	}

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Just output a message for now
		gzdbg << " ConstSpeedPlugin is attach to model[" << _model->GetName() << "]\n";

		this->base_link = _model->GetChildLink(_sdf->GetElement("base_link")->Get<std::string>());
		this->speed_vct = _sdf->Get<ignition::math::Vector3d>("speed_vct");

       // std::string commandTopic = _sdf->GetElement("commandSubTopic")->Get<std::string>();
       // this->motor_number=_sdf->GetElement("motorNumber")->Get<int>();

		counter=0;

		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			std::bind(&ConstSpeedPlugin::OnUpdate, this));

		//messaging
		this->node = transport::NodePtr(new transport::Node());
		this->node->Init(_model->GetWorld()->Name());
		// Create a topic name
		std::string topicName = "speed_plugin/speed_cmd";
		this->sub = this->node->Subscribe(topicName,
		   &ConstSpeedPlugin::OnMsg, this);

	}

	virtual void OnUpdate()
	{
		//debuging loop
        counter++;
		if(counter>DEBUG_CONST)
		{
			counter=0;
		}

		const Quaterniond baseRot=this->base_link->WorldPose().Rot();
        Vector3d speed=baseRot*this->speed_vct;

        base_link->SetAngularVel(Vector3d(0,0,0));
		base_link->SetLinearVel(speed);

		//debuging loop
		if(counter==DEBUG_CONST)
		{
            gzdbg << "Speed: " << this->speed_vct << "\n";
		}
	}

	void OnMsg(ConstVector3dPtr &_msg)
	{
	    this->speed_vct.X()=_msg->x();
	    this->speed_vct.Y()=_msg->y();
	    this->speed_vct.Z()=_msg->z();
	}
	
	private:
		event::ConnectionPtr updateConnection;
        transport::SubscriberPtr sub;
        transport::NodePtr node;
		int counter; 
		physics::LinkPtr base_link;
        Vector3d speed_vct;

  };

  GZ_REGISTER_MODEL_PLUGIN(	ConstSpeedPlugin )
}
#endif
