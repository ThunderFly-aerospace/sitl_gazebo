#ifndef _ROTOR_PLUGIN_HH_
#define _ROTOR_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include <cmath>

#define SLOW_CONST 0.1
#define DEBUG_CONST 300
#define PI 3.141592654
#define AIR_DENSITY 1.2041
#define ToDeg(a) ((a)/2.0/PI*360.0)
#define ToRad(a) ((a)/360.0*2.0*PI)

#define BLADE_COUNT 2

#define EPSILON 0.0001
#define FLAP_P 0.5
#define FLAP_ITER 20

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

    virtual void Load(physics::ModelPtr model, sdf::ElementPtr _sdf)
    {
      // Just output a message for now
		gzdbg << " RotorPlugin is attach to model[" << model->GetName() << "]\n";

		this->base_link = model->GetChildLink(_sdf->GetElement("base_link")->Get<std::string>());
        this->rotor_pos = _sdf->Get<Vector3d>("rotor_pos");
		this->blade_length=_sdf->GetElement("blade_length")->Get<double>();
        this->blade_width=_sdf->GetElement("blade_width")->Get<double>();
        this->blade_weight=_sdf->GetElement("blade_weight")->Get<double>();
        this->delta_angle=ToRad(_sdf->GetElement("blade_delta")->Get<double>());
        this->air_density=_sdf->Get<double>("air_density");
        this->number_of_elements=_sdf->Get<int>("element_count");;
        this->element_area=this->blade_width*this->blade_length/this->number_of_elements;

		 
		base_link->VisualId(_sdf->GetElement("left_blade_visual")->Get<std::string>(), this->leftBladeVisualID);
		base_link->VisualId(_sdf->GetElement("right_blade_visual")->Get<std::string>(), this->rightBladeVisualID);
		
		this->world=model->GetWorld();

		counter=0;
		rotorOmega=PI/4;
		lastUpdateTime =0.0;

        for(int b=0;b<BLADE_COUNT;b++)
        {
            rotorBladeAngle[b]=b*2.0*PI/BLADE_COUNT;
            flaping_angle[b]=ToRad(0);
        }
            

        pitch=0.0;
        roll=0.0;

	 	lbladeDefaultPos=Vector3d(0.0, blade_length/2,0.0);
		rbladeDefaultPos=Vector3d(0.0, -blade_length/2,0.0);

		Rdisc=Quaterniond(Vector3d(0,0,1),0);

		UpdateRotorVisualPos();

		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			std::bind(&RotorPlugin::OnUpdate, this));

        //messaging
		node_handle = transport::NodePtr(new transport::Node());
		node_handle->Init(model->GetWorld()->Name());

		std::string rollCmdTopic = "~/" + model->GetName() + "/rotor_roll_cmd";
		std::string pitchCmdTopic = "~/" + model->GetName() + "/rotor_pitch_cmd";

		// Subscribe to the topics, and register a callbacks
		control_roll_sub = node_handle->Subscribe(rollCmdTopic, &RotorPlugin::OnRollCmdMsg, this);
		control_pitch_sub = node_handle->Subscribe(pitchCmdTopic, &RotorPlugin::OnPitchCmdMsg, this);	

		/*rotorfreq_pub_ = node_handle_->Advertise<gazebo::msgs::Vector3d>("~/" + _model->GetName() + "/RotorFreq", 1);*/

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
			


        const Vector3d & rotorLinVel=base_link->WorldLinearVel(rotor_pos);
//        const Vector3d base_pose=base_link->WorldPose().Pos();
//        const Quaterniond base_rot=base_link->WorldPose().Rot();

      
       Vector3d omega=Rdisc*Vector3d(0,0,rotorOmega);


       for(int b=0;b<BLADE_COUNT;b++)
       {

            Quaterniond Rrot(Vector3d(0,0,1), rotorBladeAngle[b]);//otočení rotoru  
            Quaterniond Rblade=Rdisc*Rrot;//otočení rotoru na hlavě     

            Vector3d bladeAirMoment(0,0,0);
            Vector3d bladeAirForce(0,0,0);
            for(int i=0;i<FLAP_ITER;i++)//několik iterací na nalezení flapping uhlu
            {
                //TADY CHYBY přidat úhel mezi osou rotace a osou hlavy později 
                Quaterniond RbladeTotal=Rblade*Quaterniond(Vector3d(0,1,0),getFlapCorectionAngle(flaping_angle[b])); //otočení listu včetně flaping úhlu

		        Vector3d forward=RbladeTotal*Vector3d(-1,0,0);//musí být pravotočivá soustava, asi
		        Vector3d upward=RbladeTotal*Vector3d(0,0,1);
                Vector3d wingCut=RbladeTotal*Vector3d(0,-1,0);

                bladeAirMoment.Set(0,0,0);
                bladeAirForce.Set(0,0,0);

                for(int e=0;e<number_of_elements;e++)
                {
                    double d=(e+0.5)/number_of_elements*blade_length;

                    Vector3d vel=omega.Cross((Rblade*Vector3d(0,d,0)))+rotorLinVel;
                    Vector2d airfoilVel= Vector2d(vel.Dot(forward),vel.Dot(upward));

                    double airfoilVelocity=airfoilVel.Length();
                    /*if(airfoilVelocity< SLOW_CONST)
                    {
                        if(counter==DEBUG_CONST)
                            gzdbg << "Element Too Slow \n";
                        continue;
                    }*/

		            double alpha=std::atan2(airfoilVel.Y(),airfoilVel.X());
                    double q=0.5*this->air_density*this->element_area*airfoilVelocity*airfoilVelocity;
                    double L=q*getCL(alpha);
                    double D=q*getCD(alpha);

                    Vector3d liftDirection = vel.Cross(wingCut).Normalize();
                    Vector3d liftForce=L*liftDirection;//tohle je kolmo na vítr, ne na list
                    Vector3d dragDirection= -(vel-vel.Dot(wingCut)).Normalize();
                    Vector3d dragForce=D*dragDirection;//tohle je po větru

                    bladeAirMoment+=(Rblade*Vector3d(0,d,0)).Cross(liftForce);
                    bladeAirMoment+=(Rblade*Vector3d(0,d,0)).Cross(dragForce);
                    bladeAirForce+=liftForce;
                    bladeAirForce+=dragForce;
                }

                //rozklad momentů do flap a ax - do lokalních
                Vector3d bladeLocalMoment =Rblade.RotateVectorReverse(bladeAirMoment);//zatím je flap X složka bladeLocalMomentu, ale tady se pak projeví odklon osy od disku
                
                //výpočet flap_angle 
                double weightMoment=blade_length/2.0*blade_weight*9.81; //tohle je blbě, když bude rotor nakloněný, zahraje si jen čast váhy
                double odstrMomentMax=blade_weight*rotorOmega*rotorOmega*(blade_length/2.0)*(blade_length/2.0);//to je taky jen přibližné, zanedbávám cos u vzdalenosti od těžiště
                double zlo=(bladeLocalMoment.X()-weightMoment)/odstrMomentMax;

                double new_flap_angle=0;
                if(zlo>=-0.25 && zlo<=0.25)//jen od 15 do 15 stupňů
                {
                    new_flap_angle=std::asin(zlo);
                }
                else
                {
                    if(zlo>0.25)
                        new_flap_angle=ToRad(15);
                    if(zlo<-0.25)
                        new_flap_angle=ToRad(-15);
                }

                if(fabs(flaping_angle[b]-new_flap_angle)<EPSILON)
                {
                    if(counter==DEBUG_CONST)
		                    gzdbg<<"flap iterations:" << i <<std::endl;       
                    break;
                }
                else
                {
                    flaping_angle[b]+=FLAP_P*(new_flap_angle-flaping_angle[b]);
                    if(i==(FLAP_ITER-1))
                        gzdbg<<"Flapping angle not stabilized" <<std::endl;
                }
            }

            Vector3d bladeLocalMoment=Rblade.RotateVectorReverse(bladeAirMoment);
            if(counter==DEBUG_CONST)
		    {
                   gzdbg <<ToDeg(flaping_angle[b])<<std::endl;
                   gzdbg <<bladeLocalMoment<<std::endl;
                   gzdbg <<Rblade.RotateVectorReverse(bladeAirForce)<<std::endl;
            }
        }

        rotorOmega+=2*PI/50*rotadd;

        for(int b=0;b<BLADE_COUNT;b++)
            rotorBladeAngle[b]+=timeStep*rotorOmega; //Euler integration of rotor angle
		

		//update rotationAxR by input
		Rdisc=Quaterniond(Vector3d(1,0,0),roll)
    					*Quaterniond(Vector3d(0,1,0),pitch);
						

		UpdateRotorVisualPos();

		//debuging loop
		if(counter==DEBUG_CONST)
		{
           /* gazebo::msgs::Vector3d msg;
        	gazebo::msgs::Set(&msg, liftForce);

        	rotorfreq_pub_->Publish(msg);*/

			gzdbg << rotorOmega/2.0/PI*60 <<std::endl;
			gzdbg <<"R: "<< roll << std::endl;
			gzdbg <<"P: "<< pitch << std::endl;
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
        double blade_width;
        double blade_weight;
        double delta_angle;
        double air_density;
        int number_of_elements;
        double element_area;

		Vector3d lbladeDefaultPos;
		Vector3d rbladeDefaultPos;

		//rotor state		
		Quaterniond Rdisc;
		double rotorBladeAngle[BLADE_COUNT]; 
		double rotorOmega; //uhlova rychlost v radianech
        double flaping_angle[BLADE_COUNT];

		//input
		double roll;
		double pitch;

        //debug
        double rotadd;

		//messaging
		transport::NodePtr node_handle;
  		transport::SubscriberPtr control_roll_sub;
  		transport::SubscriberPtr control_pitch_sub;
		

		void UpdateRotorVisualPos()
		{

			Quaterniond rotorRot=Rdisc*Quaterniond(Vector3d(0,0,1), rotorBladeAngle[0]);

			base_link->SetVisualPose(leftBladeVisualID, Pose3d(rotor_pos+rotorRot.RotateVector(lbladeDefaultPos),rotorRot));
			base_link->SetVisualPose(rightBladeVisualID, Pose3d(rotor_pos+rotorRot.RotateVector(rbladeDefaultPos),rotorRot));
		}


		void OnRollCmdMsg(ConstAnyPtr &_msg)
		{
			roll = _msg->double_value();
		}

		void OnPitchCmdMsg(ConstAnyPtr &_msg)
		{
			//pitch = _msg->double_value();
            pitch=0.0;
            rotadd=_msg->double_value();
		}

        double getFlapCorectionAngle(double flapping_angle)
        {
            //aproximujeme (flapping_angle)_osa_kolma_na_list = (flaping_angle)_osa_s_delta_uhlem
            Vector3d rotated=Quaterniond(Vector3d(1,0,0),flapping_angle)*Vector3d(1,std::sin(this->delta_angle),0);
            return std::atan2(rotated.Z(),1);   
        }

        double getCL(double alpha)
        {
            double alpha_degree=alpha*360.0/(2.0*3.141592);
            double CL=0.0;
            if(alpha_degree > -10  && alpha_degree <= -5)
                CL=-0.5;
            if(alpha_degree > -5  && alpha_degree <= 7.5)
                CL=alpha_degree*0.136+0.18;
            if(alpha_degree > 7.5 && alpha_degree < 20)
                CL=1.2;
            return CL;
        };

        double getCD(double alpha)
        {
            
            double alpha_degree=alpha*360.0/(2.0*3.141592);
            double CD=0.0;
            if(alpha_degree > -10  && alpha_degree <= -5)
                CD=alpha_degree*(-0.016)-0.06;
            if(alpha_degree > -5  && alpha_degree <= 0)
                CD=alpha_degree*(-0.002)+0.01;
            if(alpha_degree > 0 && alpha_degree <= 7.5)
                CD=0.01;
            if(alpha_degree > 7.5 && alpha_degree <= 15)
                CD=alpha_degree*0.008-0.05;
            if(alpha_degree > 15 && alpha_degree <= 20)
                CD=alpha_degree*0.2-2.3;
            return CD;
        };

     /*   transport::NodePtr node_handle_;
        transport::PublisherPtr rotorfreq_pub_;*/


  };

  GZ_REGISTER_MODEL_PLUGIN(	RotorPlugin )
}
#endif
