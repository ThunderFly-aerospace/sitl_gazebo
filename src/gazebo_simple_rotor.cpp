#ifndef _SIMPLE_ROTOR_PLUGIN_HH_
#define _SIMPLE_ROTOR_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <cmath>

#define SLOW_CONST 0.1
#define DEBUG_CONST 500

#define AIR_DENSITY 1.2041
#define AREA 0.5

using namespace ignition::math;

namespace gazebo
{

  class SimpleRotorPlugin : public ModelPlugin
  {

    public: 
	SimpleRotorPlugin() {}

	~SimpleRotorPlugin()
	{
		updateConnection->~Connection();
	}

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Just output a message for now
		gzdbg << " SimpleRotorPlugin is attach to model[" << _model->GetName() << "]\n";

		this->base_link = _model->GetChildLink(_sdf->GetElement("force_link")->Get<std::string>());
		this->rotor_link = _model->GetChildLink(_sdf->GetElement("rotor_link")->Get<std::string>());
        this->force_pos = _sdf->Get<ignition::math::Vector3d>("force_pos");

		counter=0;

		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			std::bind(&SimpleRotorPlugin::OnUpdate, this));

	}

	virtual void OnUpdate()
	{
		//debuging loop
        counter++;
		if(counter>DEBUG_CONST)
		{
			counter=0;
		}

		const Vector3d & linVel=rotor_link->WorldLinearVel();
        if(linVel.Length()< SLOW_CONST)
        {
            if(counter==DEBUG_CONST)
                gzdbg << "Too slow \n";
            return;
        }

		const Pose3d &  pose=rotor_link->WorldPose();

		const Quaterniond rot=pose.Rot();

		Vector3d forward=rot*Vector3d(1,0,0);
		Vector3d upward=rot*Vector3d(0,0,1);
        Vector3d wingCut=rot*Vector3d(0,1,0);

		Vector2d wingVel= Vector2d(linVel.Dot(forward),linVel.Dot(upward));
        
        double wingVelocity=wingVel.Length();
        if(wingVelocity< SLOW_CONST)
        {
            if(counter==DEBUG_CONST)
                gzdbg << "Wing Too Slow \n";
            return;
        }

		double alpha=std::atan2(wingVel.Y(),wingVel.X());
        double q=0.5*AIR_DENSITY*wingVelocity*wingVelocity*AREA;
        double L=q*getCL(-alpha);
        double D=q*getCD(-alpha);

        Vector3d liftDirection = linVel.Cross(wingCut).Normalize();
        Vector3d liftForce=L*liftDirection;
        Vector3d dragDirection= -(linVel-linVel.Dot(wingCut)).Normalize();
        Vector3d dragForce=D*dragDirection;

        this->base_link->AddForceAtRelativePosition(liftForce,this->force_pos);
        this->base_link->AddForceAtRelativePosition(dragForce,this->force_pos);
                

		//debuging loop
		if(counter==DEBUG_CONST)
		{
            gzdbg << "Alpha: " << alpha*360.0/(2.0*3.141592) << "\n";
			gzdbg << "L: "<< liftForce <<"\n";
            gzdbg << "D: "<< dragForce <<"\n";
		}

	}
	
	private:
		event::ConnectionPtr updateConnection;
		int counter; 
		physics::LinkPtr base_link;
		physics::LinkPtr rotor_link;
        Vector3d force_pos;

        double getCL(double alpha)
        {
            double alpha_degree=alpha*360.0/(2.0*3.141592);
            double CL=0.0;
            if(alpha_degree > 10  && alpha_degree <= 35)
                CL=(alpha_degree-10)*0.004+0.7;
            if(alpha_degree > 35 && alpha_degree < 90)
                CL=(alpha_degree-35)*-0.014+0.8;
        };

        double getCD(double alpha)
        {
            double alpha_degree=alpha*360.0/(2.0*3.141592);
            double CD=0.0;
            if(alpha_degree > -180  && alpha_degree <= -90)
                CD=(alpha_degree+180)/90*(-1.2);
            if(alpha_degree > -90  && alpha_degree <= 90)
                CD=alpha_degree/90*1.2;
            if(alpha_degree > 90 && alpha_degree <= 180)
                CD=(alpha_degree-90)/90*-1.2+1.2;
            return CD;
        };

  };

  GZ_REGISTER_MODEL_PLUGIN(	SimpleRotorPlugin )
}
#endif
