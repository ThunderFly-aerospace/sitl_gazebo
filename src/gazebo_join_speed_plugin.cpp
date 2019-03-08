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

#define DEBUG_CONST 500

namespace gazebo
{

  class JointSpeed : public ModelPlugin
  {

    public: 
	JointSpeed() {}

	~JointSpeed()
	{
		updateConnection->~Connection();
	}

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Just output a message for now
		gzdbg << " JointSpeed is attach to model[" << _model->GetName() << "]\n";

		this->joint = _model->GetJoint(_sdf->GetElement("joint")->Get<std::string>());
		
		counter=0;

		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			std::bind(&JointSpeed::OnUpdate, this));

	}

	virtual void OnUpdate()
	{
		//debuging loop
        counter++;
		if(counter>DEBUG_CONST)
		{
			counter=0;
		}
		
        double speed=this->joint->GetVelocity(0);

		//debuging loop
		if(counter==DEBUG_CONST)
		{
            gzdbg << "Joint speed: " << speed/(2*3.14159265) <<"Hz" << "  "<< speed/(2*3.14159265)*60 << "oto/min"<<"\n";
		}
	}
	
	private:
		event::ConnectionPtr updateConnection;
		int counter; 
		physics::JointPtr joint;
  };

  GZ_REGISTER_MODEL_PLUGIN(	JointSpeed )
}
#endif
