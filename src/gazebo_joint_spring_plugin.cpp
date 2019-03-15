#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include <cmath>

#define DEBUG_CONST 10
#define SLOW_CONST 0.1


using namespace ignition::math;

namespace gazebo
{

  class JointSpringPlugin : public ModelPlugin
  {

    public: 
	JointSpringPlugin() {}

	~JointSpringPlugin()
	{
		updateConnection->~Connection();
	}

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Just output a message for now
		gzdbg << " JointSpringPlugin is attach to model[" << _model->GetName() << "]\n";

        this->joint = _model->GetJoint(_sdf->GetElement("joint")->Get<std::string>());
        this->parent_point=_sdf->Get<ignition::math::Vector3d>("parentPoint");
        this->child_point=_sdf->Get<ignition::math::Vector3d>("childPoint");
        this->zero_angle =_sdf->Get<double>("zero");
        this->k =_sdf->Get<double>("k");

		counter=0;

		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			std::bind(&JointSpringPlugin::OnUpdate, this));

	}

	virtual void OnUpdate()
	{
		//debuging loop
        counter++;
		if(counter>DEBUG_CONST)
		{
			counter=0;
		}


		physics::LinkPtr parent_link=this->joint->GetParent();
        physics::LinkPtr child_link=this->joint->GetChild();
        const Pose3d parent_pos_w=parent_link->WorldPose();
        const Pose3d child_pos_w=child_link->WorldPose();
        const Vector3d joint_pos_w=this->joint->WorldPose().Pos();

        Vector3d parent_point_w = parent_pos_w.Pos()+parent_pos_w.Rot()*this->parent_point;
        Vector3d child_point_w = child_pos_w.Pos()+child_pos_w.Rot()*this->child_point;

        const Vector3d joint_ax=this->joint->GlobalAxis(0);

        //setup coordinate system on plate
        Vector3d R1=(parent_point_w-joint_pos_w).Normalize(); //first base vector in plate where is join_pos, parent_point and child_point
        Vector3d R2=joint_ax.Cross(R1).Normalize(); // becasue this plate is prepndicular to ax, we can set second base vector like this

        //vector to parent is set as R1 ax, coordinates of child_point vector need to be computed
        Vector3d child_point_l=child_point_w-joint_pos_w;
        
        double ch_x=child_point_l.Dot(R1);
        double ch_y=child_point_l.Dot(R2);
        
		double diff=atan2(ch_y,ch_x)-this->zero_angle;
		if(diff<-3.1415926)
			diff+=6.283;
		if(diff>3.1415926)
			diff-=6.283;

        double force_size=child_point_l.Length()*diff*this->k;

        Vector3d parent_force=force_size*R2;
        Vector3d child_force=force_size*(child_point_l.Cross(joint_ax).Normalize());

        parent_link->AddForceAtRelativePosition(parent_force,this->parent_point);//expect CoG at (0,0,0) of parent link
        child_link->AddForceAtRelativePosition(child_force,this->child_point);//expect CoG at (0,0,0) of parent link 

	    //debuging loop
	    if(counter==DEBUG_CONST)
	    {             
           gzdbg << "Angle: " << diff << "\n";
           gzdbg << "parent force: "<< parent_force <<"\n";
           gzdbg << "child force: "<< child_force <<"\n";
	    }
	}
	
	private:
		event::ConnectionPtr updateConnection;
		int counter; 

        physics::JointPtr joint;
        Vector3d parent_point;
        Vector3d child_point;      
        double zero_angle;
        double k;

  };

  GZ_REGISTER_MODEL_PLUGIN(	JointSpringPlugin )
}

