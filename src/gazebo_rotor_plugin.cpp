#ifndef _ROTOR_PLUGIN_HH_
#define _ROTOR_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include <cmath>

#define SLOW_CONST 0.1
#define DEBUG_CONST 100
#define PI 3.141592654
#define AIR_DENSITY 1.2041
#define ToDeg(a) ((a)/2.0/PI*360.0)
#define ToRad(a) ((a)/360.0*2.0*PI)

#define BLADE_COUNT 2

#define UPDATE_PER_ROUND 100

using namespace ignition::math;
using namespace ignition::math::v4;
using namespace gazebo::physics;

namespace gazebo
{

  class RotorPlugin : public ModelPlugin
  {

    public: 
    RotorPlugin() 
    {
        blade_polar_angles=nullptr;
        blade_polar_CL=nullptr;
        blade_polar_CD=nullptr;
    }

    ~RotorPlugin()
    {
        updateConnection->~Connection(); //??
        if(blade_polar_angles)
            delete [] blade_polar_angles;
        if(blade_polar_CL)
            delete [] blade_polar_CL;
        if(blade_polar_CD)
            delete [] blade_polar_CD;      
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
        this->collective_angle=ToRad(_sdf->GetElement("blade_collective")->Get<double>());
        this->delta_angle=ToRad(_sdf->GetElement("blade_delta")->Get<double>());
        this->air_density=_sdf->Get<double>("air_density");
        this->number_of_elements=_sdf->Get<int>("element_count");;
        this->element_area=this->blade_width*this->blade_length/this->number_of_elements;

        base_link->VisualId(_sdf->GetElement("right_blade_visual")->Get<std::string>(), bladeVisualID[0]);         
        base_link->VisualId(_sdf->GetElement("left_blade_visual")->Get<std::string>(), bladeVisualID[1]);


        //load polar
        sdf::ElementPtr polar=_sdf->GetElement("blade_polar");
        sdf::ElementPtr polar_point=polar->GetFirstElement();
        blade_polar_point_count=0;
        while(polar_point!= sdf::ElementPtr(nullptr))
        {
            blade_polar_point_count++;
            polar_point=polar_point->GetNextElement();
        }

        gzdbg << "Loading polar approx with " << blade_polar_point_count << "points." <<std::endl;

        this->blade_polar_angles =new double[blade_polar_point_count];
        this->blade_polar_CL=new double[blade_polar_point_count];;
        this->blade_polar_CD=new double[blade_polar_point_count];;



        int p=0;
        polar_point=polar->GetFirstElement();
        while(polar_point!= sdf::ElementPtr(nullptr))
        {
            //TODO: tahle podmínka je blbě, pokud atribut neexistuje tak to hodí SEGFAULT (SIGSEGV)
            if(!polar_point->GetAttribute("angle")->Get(blade_polar_angles[p])
                || !polar_point->GetAttribute("CL")->Get(blade_polar_CL[p])
                || !polar_point->GetAttribute("CD")->Get(blade_polar_CD[p])
              )
            {
                gzdbg << "Error loading Polar point: " << p  << std::endl;
            }

            polar_point=polar_point->GetNextElement();
            p++;
        }

        gzdbg << "Polar loaded." <<std::endl;

       // for(int i=0;i<blade_polar_point_count;i++)
       //     gzdbg << blade_polar_angles[i] << " " << blade_polar_CL[i] << " " << blade_polar_CD[i] << std::endl; 

        
        this->world=model->GetWorld();

        counter=0;
        rotorOmega=10*PI;
        lastUpdateTime =0.0;

        for(int b=0;b<BLADE_COUNT;b++)
        {
            rotorBladeAngle[b]=b*2.0*PI/BLADE_COUNT;
            bladeFlapAngle[b]=ToRad(15);
            bladeOmega[b]=base_link->WorldPose().Rot()*(rotorOmega*Vector3d(0,0,1));
        }
        bladeDefaultPos=Vector3d(0.0, -blade_length/2,0.0);
        flapAxDefault=Vector3d(std::cos(delta_angle),std::sin(delta_angle),0);          

        pitch=ToRad(-7);
        roll=0.0;
       
        omegaStabil=0;
        lastRotorOmega=rotorOmega;
        up=true;
        

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

		debug_pub = node_handle->Advertise<gazebo::msgs::Vector3d>("rotor_plugin/debug_msg");
		//debug_pub->WaitForConnection();

        /*rotorfreq_pub_ = node_handle_->Advertise<gazebo::msgs::Vector3d>("~/" + _model->GetName() + "/RotorFreq", 1);*/

        begin_angle=-1;
        last_debug=this->world->SimTime();

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

      // gzdbg<<currentTime.Double() <<std::endl;
      // gzdbg<<last_debug.Double() <<std::endl;

       if(begin_angle<0 && (currentTime.Double()-last_debug.Double())>1)
       {
           // gzdbg<<"Debug one round "<<std::endl;
            begin_angle=rotorBladeAngle[0];
       }
            
       if(begin_angle>0 && (rotorBladeAngle[0]-begin_angle)>2*PI)
       {
			sendDebugMsg(-2,0,0);//data send, render now
            begin_angle=-1.0;
            last_debug=currentTime;
       }


        const Vector3d & rotorLinVel=base_link->WorldLinearVel(rotor_pos);
//        const Vector3d base_pose=base_link->WorldPose().Pos();
        const Quaterniond base_rot=base_link->WorldPose().Rot();

      
       double rotorFrequency=std::fabs(rotorOmega/2.0/PI);       
       int outerUpdatesPerRound=std::round(1.0/timeStep/rotorFrequency);
       int innerUpdates=(UPDATE_PER_ROUND/outerUpdatesPerRound)+1;
       if(innerUpdates<1)
            innerUpdates=1;

      /* if(counter==DEBUG_CONST)
       {
            //gzdbg <<"rot Frequency:" <<rotorFrequency<<std::endl;
            //gzdbg <<"outer update:" <<outerUpdatesPerRound<<std::endl;
            gzdbg <<"innerUpdates:" <<innerUpdates<<std::endl;
            //gzdbg <<"real updates:" << outerUpdatesPerRound*innerUpdates <<std::endl;
            //gzdbg <<"expected updates:" <<UPDATE_PER_ROUND<<std::endl;
       }*/

	   if(begin_angle>0)
       { 
            int anglesCount=outerUpdatesPerRound*innerUpdates;
            sendDebugMsg(-1,anglesCount,number_of_elements);
	   }

       double innerTimeStep=timeStep/innerUpdates;
	   Vector3d rotorTimeStepForce(0,0,0);
       for(int ts=0;ts<innerUpdates;ts++)
       {
           Vector3d rotorTotalForce(0,0,0);
           Vector3d rotorTotalForceMoment(0,0,0);
           for(int b=0;b<BLADE_COUNT;b++)
           {
    
                //vypočet vztlaku a dragu
                Quaterniond Rblade=base_link->WorldPose().Rot()
                    *Quaterniond(Vector3d(1,0,0),roll)
                     *Quaterniond(Vector3d(0,1,0),pitch)
                     *Quaterniond(Vector3d(0,0,1),rotorBladeAngle[b])
                     *Quaterniond(flapAxDefault,bladeFlapAngle[b])
                     *Quaterniond(bladeDefaultPos,collective_angle);

                Vector3d forward=Rblade*Vector3d(1,0,0);//musí být pravotočivá soustava, asi
                Vector3d upward=Rblade*Vector3d(0,0,1);
                Vector3d wingCut=Rblade*Vector3d(0,1,0);
                    
                Vector3d bladeAirMoment(0,0,0);
                Vector3d bladeAirForce(0,0,0);

                for(int e=0;e<number_of_elements;e++)
                {
					double debugValue;
                    //int e=number_of_elements-1;
                    double d=(e+0.5)/number_of_elements*blade_length;
                    Vector3d r=Rblade*Vector3d(0,-d,0);
                    Vector3d vel=bladeOmega[b].Cross(r)+rotorLinVel;
                    Vector2d airfoilVel= Vector2d(vel.Dot(forward),vel.Dot(upward));
              
                    double airfoilVelocity=airfoilVel.Length();
                    /*if(airfoilVelocity< SLOW_CONST)
                    {
                        if(counter==DEBUG_CONST)
                            gzdbg << "Element Too Slow \n";
                        continue;
                    }*/

                    double alpha=-std::atan2(airfoilVel.Y(),airfoilVel.X());
                    double q=0.5*this->air_density*this->element_area*airfoilVelocity*airfoilVelocity;
                    double L=q*getCL(alpha);
					if(e==number_of_elements-1)
						L=0;
                    double D=q*getCD(alpha);

                    Vector3d liftDirection = vel.Cross(wingCut).Normalize();
                    Vector3d liftForce=L*liftDirection;//tohle je kolmo na vítr, ne na list
                    Vector3d dragDirection= -(vel-vel.Dot(wingCut)).Normalize();
                    Vector3d dragForce=D*dragDirection;//tohle je po větru

                    bladeAirMoment+=r.Cross(liftForce);
                    bladeAirMoment+=r.Cross(dragForce);
                    bladeAirForce+=liftForce;
                    bladeAirForce+=dragForce;

					debugValue=ToDeg(alpha);

					if(begin_angle>0 && b==0)
					{
						sendDebugMsg(rotorBladeAngle[0],e,debugValue);
					}
                }

                Vector3d flapAx=base_link->WorldPose().Rot()
                    *Quaterniond(Vector3d(1,0,0),roll)
                    *Quaterniond(Vector3d(0,1,0),pitch)
                    *Quaterniond(Vector3d(0,0,1),rotorBladeAngle[b])
                     *flapAxDefault;

                double omega=bladeOmega[b].Length();
                Vector3d omegaAx=bladeOmega[b]/omega;
                                    
                Vector3d bladeT=Rblade*Vector3d(0,-blade_length/2.0,0);
                //double airFlapMoment=bladeAirMoment.Dot(flapAx);
                Vector3d bladeWeightForce(0,0,-blade_weight*9.81);
                Vector3d bladeWeightForceMoment=(bladeT.Cross(bladeWeightForce));                
              
                Vector3d odstrDir=bladeT-((bladeT.Dot(omegaAx))*omegaAx);
                Vector3d odstrForce=omega*omega*blade_weight*odstrDir;
                Vector3d odstrForceMoment=(bladeT.Cross(odstrForce));
                //double odstrFlapMoment=odstrForceMoment.Dot(flapAx);

                /*bladeAirMoment=bladeAirMoment.Dot(flapAx)*flapAx+bladeAirMoment.Dot(omegaAx)*omegaAx;
                bladeWeightForceMoment=bladeWeightForceMoment.Dot(flapAx)*flapAx+bladeWeightForceMoment.Dot(omegaAx)*omegaAx;//forget bladeAx momentum
                odstrForceMoment=odstrForceMoment.Dot(flapAx)*flapAx+odstrForceMoment.Dot(omegaAx)*omegaAx;//forget bladeAx momentum*/
   

                Vector3d bladeTotalForce=odstrForce+bladeWeightForce+bladeAirForce;
                Vector3d bladeTotalForceMoment=odstrForceMoment+bladeAirMoment+bladeWeightForceMoment;

                //forget bladeAx momentum
                bladeTotalForceMoment=bladeTotalForceMoment.Dot(flapAx)*flapAx+bladeTotalForceMoment.Dot(omegaAx)*omegaAx;

				/*if(counter==DEBUG_CONST)
				{
					gzdbg << b <<" angle:" << ToDeg(bladeFlapAngle[b]) <<std::endl;
                    gzdbg << b <<" moment: "<< odstrFlapMoment <<" " <<bladeWeightFlapMoment <<std::endl;
					gzdbg << b <<"  omega:" << Rblade.RotateVectorReverse(bladeOmega[b]) <<std::endl;
				}*/

				if(bladeTotalForceMoment.Length()>0)
				{
                	double J=1.0/12.0*blade_weight*blade_length*blade_length;//to je vůči těžišti
                	J+=blade_weight*blade_length/2*blade_length/2;//Steinerova větě - posun do konce listu
                	bladeOmega[b]=bladeOmega[b]+innerTimeStep*bladeTotalForceMoment/J;//tohle není úplně dobře, v ose listu je J->0 -řešíme zapomenutím tohoto momentu
				}

                rotorTotalForce+=bladeTotalForce;
                rotorTotalForceMoment+=bladeTotalForceMoment;
    
               
                /*if(counter==DEBUG_CONST)
	            {
                    gzdbg << " Blade ["<< b << "]   force: " << Rblade.RotateVectorReverse(bladeTotalForce) << std::endl;
                }*/
            }


            //sjedocení rotace okolo osy rotoru
            Vector3d rotorAx=base_link->WorldPose().Rot()
                                                *Quaterniond(Vector3d(1,0,0),roll)
                                                *Quaterniond(Vector3d(0,1,0),pitch)
                                                *Vector3d(0,0,1);
            rotorOmega=0;
            for(int b=0;b<BLADE_COUNT;b++)
                rotorOmega+=bladeOmega[b].Dot(rotorAx);
            rotorOmega/=BLADE_COUNT;

            //rotorOmega+=(rotadd*PI/25.0);

            for(int b=0;b<BLADE_COUNT;b++)
            {
                bladeOmega[b]=bladeOmega[b]+(rotorOmega-bladeOmega[b].Dot(rotorAx))*rotorAx;
            }


            if(counter==DEBUG_CONST)
            {      
                   //gzdbg <<"angle: " << rotorBladeAngle[0]
                       /*  gzdbg<<"rotor Ax: " << base_link->WorldPose().Rot()
                                                *Quaterniond(Vector3d(1,0,0),roll)
                                                *Quaterniond(Vector3d(0,1,0),pitch)
                                                *Vector3d(0,0,1) <<std::endl;
                         gzdbg<<" rotor Totoal Force: " <<rotorTotalForce<<std::endl;
                         gzdbg<<" rotor Totoal Force Moment: " <<rotorTotalForceMoment<<std::endl;
                         gzdbg<<" RotorOmega: " << rotorOmega<<std::endl;*/
            }

            //Euler integration of rotor angle
            for(int b=0;b<BLADE_COUNT;b++)
            {                
                Vector3d flapAx=base_link->WorldPose().Rot()
                    *Quaterniond(Vector3d(1,0,0),roll)
                    *Quaterniond(Vector3d(0,1,0),pitch)
                    *Quaterniond(Vector3d(0,0,1),rotorBladeAngle[b])
                     *flapAxDefault;

                rotorBladeAngle[b]+=innerTimeStep*rotorOmega;
                bladeFlapAngle[b]+=innerTimeStep*bladeOmega[b].Dot(flapAx);
				if(bladeFlapAngle[b]>ToRad(15))
				{
                    //gzdbg<<"dolní doraz" <<std::endl;
					bladeFlapAngle[b]=ToRad(15);
					bladeOmega[b]-=bladeOmega[b].Dot(flapAx)*flapAx;
				}
				if(bladeFlapAngle[b]<ToRad(-15))
				{
                    //gzdbg<<"horní doraz" <<std::endl;
					bladeFlapAngle[b]=ToRad(-15);
					bladeOmega[b]-=bladeOmega[b].Dot(flapAx)*flapAx;
				}

            }
			
           /* if(begin_angle>0)
		    {
                double angle =rotorBladeAngle[0];
                while(angle>2*PI)
                    angle-=2*PI;
				gzdbg<< "Angle: " << angle << " Total Force: "<< rotorTotalForce <<std::endl;
		    }*/
			rotorTimeStepForce+=rotorTotalForce;
		}              
		rotorTimeStepForce/=innerUpdates;

		//compute force pos
		if(counter==DEBUG_CONST)
	    {
            const Quaterniond & bodyR=base_link->WorldPose().Rot();

			//gzdbg << " Rotor Force: "<< bodyR.RotateVectorReverse(rotorTimeStepForce) <<std::endl;
            /*for(int b=0;b<BLADE_COUNT;b++)
            {
                gzdbg << " Blade ["<< b << "]   omega: " << bodyR.RotateVectorReverse(bladeOmega[b]) << std::endl;
                gzdbg << " Blade ["<< b << "]: " <<   bladeFlapAngle[b] <<std::endl;
            }*/
	    }
        Vector3d forcePosRelToCog= rotor_pos-(base_link->GetInertial()->CoG());

		base_link->AddForceAtRelativePosition(rotorTimeStepForce,forcePosRelToCog);

        //rotor polar test
        if(counter==DEBUG_CONST )
        {
            if(fabs(rotorOmega/2.0/PI*60-lastRotorOmega/2.0/PI*60)<2)
            {
                omegaStabil++;
                if(omegaStabil==10)
                {
                    const Quaterniond & bodyR=base_link->WorldPose().Rot();
                    gzdbg << "pitch: " << ToDeg(-pitch) << " RPM: " << rotorOmega/2.0/PI*60 << " Totoal Force: " << bodyR.RotateVectorReverse(rotorTimeStepForce) << std::endl;
                    if(up)
                    {
                        pitch-=ToRad(1);
                        if(pitch<ToRad(-90))
                        {
                            pitch=ToRad(-90);
                            up=false;
                        }
                    }
                    else
                    {
                        pitch+=ToRad(1);
                        if(pitch>ToRad(0))
                        {
                            pitch=ToRad(0);
                            up=true;
                        }
                    }
                    omegaStabil=0;
                    lastRotorOmega=rotorOmega;
                }
            }
            else
            {
                omegaStabil=0;
                lastRotorOmega=rotorOmega;
            }
        }

        UpdateRotorVisualPos();

        //debuging loop
        if(counter==DEBUG_CONST)
        {
           /* gazebo::msgs::Vector3d msg;
            gazebo::msgs::Set(&msg, liftForce);
            rotorfreq_pub_->Publish(msg);*/

            gzdbg<< "RPM:" << rotorOmega/2.0/PI*60 <<std::endl;
			//gzdbg << "right omega:" <<bladeOmega[0] <<std::endl;
			//gzdbg << "left omega:" <<bladeOmega[1] <<std::endl;

       /*   gzdbg<<"============================Profile===================================="<<std::endl;
            for(int i=-180;i<180;i++)
                gzdbg<< i << ":" << getCL(ToRad(i))<<":"<<getCD(ToRad(i)) <<std::endl;
            gzdbg<<"============================Profile=End================================"<<std::endl;*/
        }

    }
    
    private:
        event::ConnectionPtr updateConnection;
        WorldPtr world;
        common::Time lastUpdateTime;

        common::Time last_debug;
        double begin_angle;

        int counter; 
        LinkPtr base_link;
        Vector3d rotor_pos;
        double blade_length;
        double blade_width;
        double blade_weight;
        double delta_angle;
        double collective_angle;
        double air_density;

        int blade_polar_point_count;
        double *blade_polar_angles;
        double *blade_polar_CL;
        double *blade_polar_CD;

        int number_of_elements;
        double element_area;

        unsigned int bladeVisualID[BLADE_COUNT];

        Vector3d bladeDefaultPos;
        Vector3d flapAxDefault;

        //rotor state
        double rotorBladeAngle[BLADE_COUNT]; 
        double bladeFlapAngle[BLADE_COUNT];
        Vector3d bladeOmega[BLADE_COUNT];
        double rotorOmega;

        double lastRotorOmega;
        int omegaStabil;
        bool up;

        //input
        double roll;
        double pitch;

        //debug
        double rotadd;

        //messaging
        transport::NodePtr node_handle;
        transport::SubscriberPtr control_roll_sub;
        transport::SubscriberPtr control_pitch_sub;
        transport::PublisherPtr debug_pub;

        void UpdateRotorVisualPos()
        {
            for(int b=0;b<BLADE_COUNT;b++)
            {
                Quaterniond bladeRot=/*base_link->WorldPose().Rot()
                    */Quaterniond(Vector3d(1,0,0),roll)
                    *Quaterniond(Vector3d(0,1,0),pitch)
                    *Quaterniond(Vector3d(0,0,1),rotorBladeAngle[b])
                    *Quaterniond(flapAxDefault,bladeFlapAngle[b])
                    *Quaterniond(bladeDefaultPos,collective_angle);   
            
                base_link->SetVisualPose(bladeVisualID[b], Pose3d(rotor_pos+bladeRot.RotateVector(bladeDefaultPos),bladeRot));
            }           
        }


        void OnRollCmdMsg(ConstAnyPtr &_msg)
        {
            //roll = _msg->double_value();
            //bladeFlapAngle[0]=_msg->double_value();
        }

        void OnPitchCmdMsg(ConstAnyPtr &_msg)
        {
            //pitch = _msg->double_value();
            //pitch=0.0;
            //rotadd=_msg->double_value();
        }

        double getCL(double alpha)
        {
            //return 0.0;
            double alpha_degree=ToDeg(alpha);
            return linearApprox(alpha_degree,blade_polar_angles,blade_polar_CL,blade_polar_point_count);
        };

        double getCD(double alpha)
        {
            double alpha_degree=ToDeg(alpha);
            return linearApprox(alpha_degree,blade_polar_angles,blade_polar_CD,blade_polar_point_count);
        };

		void sendDebugMsg(double x, double y, double z)
		{
			gazebo::msgs::Vector3d msg;
			gazebo::msgs::Set(&msg, Vector3d(x, y, z));
			debug_pub->Publish(msg);
		}

		double linearApprox(double x, double *xx, double *yy, int len)
		{
			if(len<=0)
			{
				gzdbg<<"Bad lenght of array.."<<std::endl;
				return 0.0;
			}
			
			if(x<xx[0])
			{
				//gzdbg<<"lower than first element" <<std::endl;
				return yy[0];
			}

			int i=0;
			while( i<len-1 && !(x>=xx[i] && x< xx[i+1]))
				i++;

			if(i<len-1)
				return yy[i]+(x-xx[i])*((yy[i+1]-yy[i])/(xx[i+1]-xx[i]));
			else            
				return yy[len-1];


		};

     /*   transport::NodePtr node_handle_;
        transport::PublisherPtr rotorfreq_pub_;*/


  };

  GZ_REGISTER_MODEL_PLUGIN(    RotorPlugin )
}
#endif
