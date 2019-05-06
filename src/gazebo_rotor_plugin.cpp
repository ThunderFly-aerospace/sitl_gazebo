#ifndef _ROTOR_PLUGIN_HH_
#define _ROTOR_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include <cmath>

#define SLOW_CONST 0.1
#define DEBUG_CONST 500
#define PI 3.141592654

using namespace ignition::math;
using namespace ignition::math::v4;
using namespace gazebo::physics;

namespace gazebo
{

  class RotorPlugin : public ModelPlugin
	  {

    public: 
	RotorPlugin() {}

	~RotorPlugin()
	{
		updateConnection->~Connection(); //??
	}

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Just output a message for now
		gzdbg << " RotorPlugin is attach to model[" << _model->GetName() << "]\n";

		this->base_link = _model->GetChildLink(_sdf->GetElement("base_link")->Get<std::string>());
        this->rotor_pos = _sdf->Get<Vector3d>("rotor_pos");
		this->blade_length=_sdf->GetElement("blade_length")->Get<double>();
		 
		base_link->VisualId(_sdf->GetElement("left_blade_visual")->Get<std::string>(), this->leftBladeVisualID);
		base_link->VisualId(_sdf->GetElement("right_blade_visual")->Get<std::string>(), this->rightBladeVisualID);
		
		this->world=_model->GetWorld();

		counter=0;
		rotorAngle=0.0;
		rotorOmega=2*PI;
		lastUpdateTime =0.0;

	 	lbladeDefaultPos=Vector3d(0.0, -blade_length/2,0.0);
		rbladeDefaultPos=Vector3d(0.0, blade_length/2,0.0);

		UpdateRotorVisualPos();

		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			std::bind(&RotorPlugin::OnUpdate, this));

        //messaging
     /*   node_handle_ = transport::NodePtr(new transport::Node());
        node_handle_->Init("");
        rotorfreq_pub_ = node_handle_->Advertise<gazebo::msgs::Vector3d>("~/" + _model->GetName() + "/RotorFreq", 1);*/

	}

	virtual void OnUpdate()
	{
		//debuging loop
        counter++;
		if(counter>DEBUG_CONST)
		{
			counter=0;
		}

		common::Time currentTime=this->world->SimTime();
		double timeStep=currentTime.Double() - this->lastUpdateTime.Double();
		this->lastUpdateTime=currentTime;
			
		rotorAngle+=timeStep*rotorOmega; //Euler integration of rotor angle

		UpdateRotorVisualPos();
		/*const Vector3d & linVel=rotor_link->WorldLinearVel();
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

		double alpha=-std::atan2(wingVel.Y(),wingVel.X());
        double q=0.5*AIR_DENSITY*wingVelocity*wingVelocity*AREA;
        double L=q*getCL(alpha);
        double D=q*getCD(alpha);

        Vector3d liftDirection = linVel.Cross(wingCut).Normalize();
        Vector3d liftForce=L*liftDirection;
        Vector3d dragDirection= -(linVel-linVel.Dot(wingCut)).Normalize();
        Vector3d dragForce=D*dragDirection;

        this->base_link->AddForceAtRelativePosition(liftForce,this->force_pos);
        this->base_link->AddForceAtRelativePosition(dragForce,this->force_pos);
        */

		//debuging loop
		if(counter==DEBUG_CONST)
		{
           /* const Quaterniond baseRot=this->base_link->WorldPose().Rot();
            
	            gzdbg << "Alpha: " << alpha*360.0/(2.0*3.141592) << "\n";
			gzdbg << "L: "<< baseRot.Inverse()*liftForce <<"\n";
            gzdbg << "D: "<< baseRot.Inverse()*dragForce <<"\n";


            gazebo::msgs::Vector3d msg;
        	gazebo::msgs::Set(&msg, liftForce);

        	rotorfreq_pub_->Publish(msg);*/

			gzdbg << timeStep <<std::endl;
		}

	}
	
	private:
		event::ConnectionPtr updateConnection;
		WorldPtr world;
		common::Time lastUpdateTime;

		int counter; 
		LinkPtr base_link;
        Vector3d rotor_pos;
		unsigned int leftBladeVisualID;
		unsigned int rightBladeVisualID;
		double blade_length;

		Vector3d lbladeDefaultPos;
		Vector3d rbladeDefaultPos;

		//rotor state		
		double rotorAngle; 
		double rotorOmega; //uhlova rychlost v radianech - zatím double 


		void UpdateRotorVisualPos()
		{
			Quaterniond rotorRot=Quaterniond(0.0, 0.0, rotorAngle);

			base_link->SetVisualPose(leftBladeVisualID, Pose3d(rotor_pos+rotorRot*lbladeDefaultPos,rotorRot));
			base_link->SetVisualPose(rightBladeVisualID, Pose3d(rotor_pos+rotorRot*rbladeDefaultPos,rotorRot));
		}


       /* double getCL(double alpha)
        {
            double alpha_degree=alpha*360.0/(2.0*3.141592);
            double CL=0.0;
            if(alpha_degree > 0  && alpha_degree <= 10)
                CL=(alpha_degree)*0.07;
            if(alpha_degree > 10  && alpha_degree <= 35)
                CL=(alpha_degree-10)*0.004+0.7;
            if(alpha_degree > 35 && alpha_degree < 90)
                CL=(alpha_degree-35)*-0.014+0.8;
            return CL;
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
            return fabs(CD);
        };

        transport::NodePtr node_handle_;
        transport::PublisherPtr rotorfreq_pub_;*/


  };

  GZ_REGISTER_MODEL_PLUGIN(	RotorPlugin )
}
#endif
