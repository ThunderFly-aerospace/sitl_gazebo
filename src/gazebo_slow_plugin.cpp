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

  class SlowPlugin : public ModelPlugin
  {

    public: 
	SlowPlugin() {}

	~SlowPlugin()
	{
		updateConnection->~Connection();
	}

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Just output a message for now
		gzdbg << " SlowPlugin is attach to model[" << _model->GetName() << "]\n";

		this->base_link = _model->GetChildLink(_sdf->GetElement("base_link")->Get<std::string>());
		this->coef = _sdf->GetElement("coef")->Get<double>();
		
		counter=0;

		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			std::bind(&SlowPlugin::OnUpdate, this));

	}

	virtual void OnUpdate()
	{
		//debuging loop
        counter++;
		if(counter>DEBUG_CONST)
		{
			counter=0;
		}

        const Vector3d& lspeed=this->base_link->WorldLinearVel();

		Vector3d force=-(this->coef)*lspeed;

        base_link->AddForceAtRelativePosition(force, Vector3d(0,0,0));
		
		//debuging loop
		if(counter==DEBUG_CONST)
		{
            gzdbg << "Braking force: " << force <<"\n";
		}
	}
	
	private:
		event::ConnectionPtr updateConnection;
		int counter; 
		physics::LinkPtr base_link;
		double coef;

  };

  GZ_REGISTER_MODEL_PLUGIN(	SlowPlugin )
}
#endif
