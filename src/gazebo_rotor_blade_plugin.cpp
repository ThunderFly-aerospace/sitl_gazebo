#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include <cmath>

#define DEBUG_CONST 500
#define SLOW_CONST 0.1


using namespace ignition::math;

namespace gazebo
{

  class BladePlugin : public ModelPlugin
  {

    public: 
	BladePlugin() {}

	~BladePlugin()
	{
		updateConnection->~Connection();
	}

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Just output a message for now
		gzdbg << " BladePlugin is attach to model[" << _model->GetName() << "]\n";

		this->blade_link = _model->GetChildLink(_sdf->GetElement("link")->Get<std::string>());
        this->link_forward = _sdf->Get<ignition::math::Vector3d>("forward");
        this->link_upward= _sdf->Get<ignition::math::Vector3d>("upward");
		this->blade_length= _sdf->Get<double>("length");
        this->blade_width=_sdf->Get<double>("width");
        this->air_density=_sdf->Get<double>("airDensity");
        this->number_of_elements=_sdf->Get<int>("elementCount");;
        this->area=this->blade_width*this->blade_length/this->number_of_elements;

		counter=0;

		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			std::bind(&BladePlugin::OnUpdate, this));

	}

	virtual void OnUpdate()
	{
		//debuging loop
        counter++;
		if(counter>DEBUG_CONST)
		{
			counter=0;
		}

        for(int i=0;i<this->number_of_elements;i++)
        {
            double first_elem_center=this->blade_length/2.0*(-1.0+1.0/this->number_of_elements);
            double elem_size=this->blade_length/this->number_of_elements;
            
            Vector3d offset=Vector3d(0,first_elem_center+i*elem_size,0);

		    const Vector3d & linVel=this->blade_link->WorldLinearVel(offset);
            if(linVel.Length()< SLOW_CONST)
            {
               /* if(counter==DEBUG_CONST)
                    gzdbg << "Too slow \n";*/
                continue;
            }

		    const Quaterniond rot=blade_link->WorldPose().Rot();

		    Vector3d forward=(rot*this->link_forward).Normalize();
		    Vector3d upward=(rot*this->link_upward).Normalize();
            Vector3d wingCut=upward.Cross(forward).Normalize();

		    Vector2d airfoilVel= Vector2d(linVel.Dot(forward),linVel.Dot(upward));
            
            double airfoilVelocity=airfoilVel.Length();
            if(airfoilVelocity< SLOW_CONST)
            {
                /*if(counter==DEBUG_CONST)
                    gzdbg << "Wing Too Slow \n";*/
                continue;
            }

		    double alpha=-std::atan2(airfoilVel.Y(),airfoilVel.X());
            double q=0.5*this->air_density*this->area*airfoilVelocity*airfoilVelocity;
            double L=q*getCL(alpha);
            double D=q*getCD(alpha);

            Vector3d liftDirection = linVel.Cross(wingCut).Normalize();
            Vector3d liftForce=L*liftDirection;
            Vector3d dragDirection= -(linVel-linVel.Dot(wingCut)).Normalize();
            Vector3d dragForce=D*dragDirection;

            this->blade_link->AddForceAtRelativePosition(liftForce,offset);
            this->blade_link->AddForceAtRelativePosition(dragForce,offset);
                        

		    //debuging loop
		    if(counter==DEBUG_CONST)
		    {             
             /*  gzdbg << "Alpha: " << alpha*360.0/(2.0*3.141592) << "\n";
			    gzdbg << "L"<<i<<": "<< rot.Inverse()*liftForce <<"\n";
                gzdbg << "D"<<i<<": "<< rot.Inverse()*dragForce <<"\n";*/
		    }
        }

	}
	
	private:
		event::ConnectionPtr updateConnection;
		int counter; 
		physics::LinkPtr blade_link;
        Vector3d link_forward;
        Vector3d link_upward;
		double blade_length;
        double blade_width;
        double air_density;
        double area;
        int number_of_elements;

        double getCL(double alpha)
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

  };

  GZ_REGISTER_MODEL_PLUGIN(	BladePlugin )
}

