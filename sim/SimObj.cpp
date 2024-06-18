#include "SimObj.h"
#include <iostream>
#include <fstream>

cSimObj::cSimObj()
	: mWorld(nullptr)
{
	mType = eTypeDynamic;
	mObjType = eObjTypeMax;
	mColGroup = cContactManager::gFlagAll;
	mColMask = cContactManager::gFlagAll;
}

cSimObj::eObjType cSimObj::GetObjType() const
{
	return mObjType;
}
void cSimObj::SetObjType(cSimObj::eObjType obj_type)
{
	mObjType = obj_type;
}
cSimObj::ExternalForce::ExternalForce()
{
	collision = false;
	external_force_ground = tVector::Zero();
	external_force_obstacle = tVector::Zero();
	external_force_agent = tVector::Zero();

	external_force_ground_ins = tVector::Zero();
	external_force_obstacle_ins = tVector::Zero();
	external_force_agent_ins = tVector::Zero();

	force_ground_pos = tVector::Zero();
	force_obstacle_pos = tVector::Zero();
	force_agent_pos = tVector::Zero();

	num_force_ground = 0;
	num_force_obstacle = 0;
	num_force_agent = 0;
}

cSimObj::ExternalForce cSimObj::GetExternalForce() 
{
	return mExternalForce;
}
void cSimObj::ResetExternalForce()
{
	mExternalForce.collision = false;
	mExternalForce.external_force_ground = tVector::Zero();
	mExternalForce.external_force_obstacle = tVector::Zero();
	mExternalForce.external_force_agent = tVector::Zero();

	mExternalForce.external_force_ground_ins = tVector::Zero();
	mExternalForce.external_force_obstacle_ins = tVector::Zero();
	mExternalForce.external_force_agent_ins = tVector::Zero();

	mExternalForce.force_ground_pos = tVector::Zero();
	mExternalForce.force_obstacle_pos = tVector::Zero();
	mExternalForce.force_agent_pos = tVector::Zero();

	mExternalForce.num_force_ground = 0;
	mExternalForce.num_force_obstacle = 0;
	mExternalForce.num_force_agent = 0;
}

cSimObj::~cSimObj()
{
	RemoveFromWorld();
}

void cSimObj::SetPreLinearVelocity()
{
	pre_linearvel = GetLinearVelocity();
	pre_vel = GetVelocity();
}

tVector cSimObj::GetVelocity() const
{
	return mWorld->GetLinearVelocity(this, tVector::Zero());
}

void cSimObj::SetForce(){
	next_vel = tVector(0, 0, 0, 0);
	force = tVector(0, 0, 0, 0);
	m_torque = tVector(0, 0, 0, 0);
}

void cSimObj::UpdateForce(){
	tVector cur_vel = mWorld->GetLinearVelocity(this);
	tVector acc =  GetMass() * (cur_vel - pre_linearvel) / 0.001667;
	//printf("SimObj GetLinearAcceleration pre_linearvel: %f %f %f\n", pre_linearvel[0], pre_linearvel[1], pre_linearvel[2]);
	pre_linearvel = GetLinearVelocity();

	//printf("SimObj GetLinearAcceleration pre_linearvel: %f %f %f\n", pre_linearvel[0], pre_linearvel[1], pre_linearvel[2]);
	
	force = acc;
	if(pre_linearvel[0] == 0 && pre_linearvel[1] == 0 && pre_linearvel[2] == 0){
		force = tVector(0, 0, 0, 0);
	}

}
tVector cSimObj::GetLinearForce() 
{
	tVector cur_vel = mWorld->GetLinearVelocity(this);
	tVector acc =  GetMass() * (cur_vel - pre_linearvel) / 0.001667;
	//printf("SimObj GetLinearAcceleration pre_linearvel: %f %f %f\n", pre_linearvel[0], pre_linearvel[1], pre_linearvel[2]);
	pre_linearvel = GetLinearVelocity();

	//printf("SimObj GetLinearAcceleration pre_linearvel: %f %f %f\n", pre_linearvel[0], pre_linearvel[1], pre_linearvel[2]);
	
	force = acc;
	if(pre_linearvel[0] == 0 && pre_linearvel[1] == 0 && pre_linearvel[2] == 0){
		acc = tVector(0, 0, 0, 0);
	}
	return acc;	
}

tVector cSimObj::GetForce() 
{
	tVector cur_vel = mWorld->GetLinearVelocity(this, tVector::Zero());
	tVector force =  GetMass() * (cur_vel - pre_vel) / 0.001667;
	//printf("SimObj GetLinearAcceleration pre_linearvel: %f %f %f\n", pre_linearvel[0], pre_linearvel[1], pre_linearvel[2]);
	pre_vel = GetVelocity();

	//printf("SimObj GetLinearAcceleration pre_linearvel: %f %f %f\n", pre_linearvel[0], pre_linearvel[1], pre_linearvel[2]);
	
	if(pre_vel[0] == 0 && pre_vel[1] == 0 && pre_vel[2] == 0){
		return tVector(0, 0, 0, 0);
	}
	return force;
	
}
/*
tVector cSimObj::GetTorqueImpulse() const
{
	return mWorld->GetTorqueImpulse(this);
}
*/
//Ray
tVector cSimObj::GetTotalForce() const
{
	return mWorld->GetTotalForce(this);
}

tVector cSimObj::GetNextLinearVelocity() const
{
	return next_vel;
}
void cSimObj::SetLinearVelocity(const tVector& vel)
{
	//printf("SimObj SetLinearVelocity: %f %f %f\n", vel[0], vel[1], vel[2]);
	next_vel = vel;
	mWorld->SetLinearVelocity(vel, this);
}
tVector cSimObj::GetGravity() const
{
	return mWorld->GetGravity()*GetMass();
}

void cSimObj::ApplyTorque(const tVector& torque)
{
	//printf("SimObj.cpp Torque %f %f %f\n", torque[0], torque[1], torque[2]);
	std::string filename("/home/ruizhang/Desktop/Data/Torques/llc_jg3.txt");
	std::fstream file;
	file.open(filename, std::ios_base::app | std::ios_base::in);
	if(file.is_open()){
		file << torque[0]  << std::endl;	
		file << torque[1]  << std::endl;
		file << torque[2]  << std::endl;	
	}
	file.close();
	m_torque += torque;

	mWorld->ApplyTorque(torque, this);
}

tVector cSimObj::GetmTorque()
{
	tVector temp = tVector(m_torque[0], m_torque[1], m_torque[2], m_torque[3]);
	m_torque = tVector(0, 0, 0, 0);
	return temp;
}

tVector cSimObj::GetPos() const
{
	return mWorld->GetPos(this);
}

void cSimObj::SetPos(const tVector& pos)
{
	mWorld->SetPos(pos, this);
}

void cSimObj::GetRotation(tVector& out_axis, double& out_theta) const
{
	mWorld->GetRotation(this, out_axis, out_theta);
}

tQuaternion cSimObj::GetRotation() const
{
	return mWorld->GetRotation(this);
}

void cSimObj::SetRotation(const tVector& axis, double theta)
{
	mWorld->SetRotation(axis, theta, this);
}

void cSimObj::SetRotation(const tQuaternion& q)
{
	mWorld->SetRotation(q, this);
}

tVector cSimObj::GetLinearVelocity() const
{
	return mWorld->GetLinearVelocity(this);
}

tVector cSimObj::GetLinearVelocity(const tVector& local_pos) const
{
	return mWorld->GetLinearVelocity(this, local_pos);
}

tVector cSimObj::GetAngularVelocity() const
{
	return mWorld->GetAngularVelocity(this);
}

void cSimObj::SetAngularVelocity(const tVector& vel)
{
	mWorld->SetAngularVelocity(vel, this);
}

tMatrix cSimObj::GetWorldTransform() const
{
	return mWorld->GetWorldTransform(this);
}

tMatrix cSimObj::GetLocalTransform() const
{
	return mWorld->GetLocalTransform(this);
}

void cSimObj::SetDamping(double linear_damping, double angular_damping)
{
	mWorld->SetDamping(linear_damping, angular_damping, *this);
}

double cSimObj::GetMass() const
{
	return 1 / mSimBody->getInvMass();
}

tVector cSimObj::WorldToLocalPos(const tVector& world_pos) const
{
	tMatrix world_to_local = GetLocalTransform();
	tVector local_pt = world_pos;

	local_pt[3] = 1;
	local_pt = world_to_local * local_pt;
	local_pt[3] = 0;

	return local_pt;
}

tVector cSimObj::LocalToWorldPos(const tVector& local_pos) const
{
	tMatrix local_to_world = GetWorldTransform();
	tVector world_pos = local_pos;

	world_pos[3] = 1;
	world_pos = local_to_world * world_pos;
	world_pos[3] = 0;

	return world_pos;
}

tMatrix3 cSimObj::GetLocalToWorldRotMat() const
{
	tMatrix local_to_world = GetWorldTransform();
	tMatrix3 mat = local_to_world.block(0, 0, 3, 3);
	return mat;
}

void cSimObj::ApplyForce(const tVector& force)
{
	ApplyForce(force, tVector::Zero());
}

void cSimObj::ApplyForce(const tVector& force, const tVector& local_pos)
{
	mWorld->ApplyForce(force, local_pos, this);
}

void cSimObj::ClearForces()
{
	mSimBody->clearForces();
}

void cSimObj::RegisterContact()
{
	RegisterContact(cContactManager::gFlagAll, cContactManager::gFlagAll);
}

void cSimObj::RegisterContact(int contact_flags, int filter_flags)
{
	if (!mContactHandle.IsValid())
	{
		mContactHandle = mWorld->RegisterContact(contact_flags, filter_flags);
		assert(mContactHandle.IsValid());
	}
	else
	{
		assert(false); // already registered contact
	}
}

void cSimObj::UpdateContact(int contact_flags, int filter_flags)
{
	mContactHandle.mFlags = contact_flags;
	mContactHandle.mFilterFlags = filter_flags;

	if (mContactHandle.IsValid())
	{
		mWorld->UpdateContact(mContactHandle);
	}
}

const cContactManager::tContactHandle& cSimObj::GetContactHandle() const
{
	return mContactHandle;
}

bool cSimObj::IsInContact() const
{
	bool in_contact = mWorld->IsInContact(mContactHandle);
	return in_contact;
}

tVector cSimObj::GetContactPt() const
{
	return mWorld->GetContactPt(mContactHandle);
}

tVector cSimObj::GetContactImpulse() const
{
/*
	int numManifolds = mWorld->getDispatcher()->getNumManifolds();
	for (int i=0;i<numManifolds;i++)
	{
	btPersistentManifold* contactManifold = mWorld->getDispatcher()->getManifoldByIndexInternal(i);
	btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
	btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());

	int numContacts = contactManifold->getNumContacts();
	for (int j=0;j<numContacts;j++)
	{
	btManifoldPoint& pt = contactManifold->getContactPoint(j);// get contact point
	if (pt.getDistance()<0.f)// see if obA and obB collide
	{
	float impulse = pt.m_appliedImpulse;// get impules
	const btVector3& ptA = pt.getPositionWorldOnA();// global contact position
	const btVector3& ptB = pt.getPositionWorldOnB();// global contact position
	const btVector3& normalOnB = pt.m_normalWorldOnB;// global contact vector
	}
	}
	}
	*/
	return mWorld->GetContactImpulse(mContactHandle);
}

short cSimObj::GetColGroup() const
{
	return mColGroup;
}

void cSimObj::SetColGroup(short col_group)
{
	mColGroup = col_group;
}

short cSimObj::GetColMask() const
{
	return mColMask;
}

void cSimObj::SetColMask(short col_mask)
{
	mColMask = col_mask;
}

void cSimObj::SetKinematicObject(bool is_kin)
{
	int col_flags = mSimBody->getCollisionFlags();
	if (is_kin)
	{
		col_flags |= btCollisionObject::CF_KINEMATIC_OBJECT;
	}
	else
	{
		col_flags &= ~btCollisionObject::CF_KINEMATIC_OBJECT;
	}
	mSimBody->setCollisionFlags(col_flags);
}

bool cSimObj::IsKinematicObject() const
{
	return mSimBody->isKinematicObject();
}

void cSimObj::DisableDeactivation()
{
	mSimBody->setActivationState(DISABLE_DEACTIVATION);
}

void cSimObj::CalcAABB(tVector& out_min, tVector& out_max) const
{
	mWorld->CalcAABB(this, out_min, out_max);
}

void cSimObj::Constrain(const tVector& linear_factor, const tVector& angular_factor)
{
	mWorld->Constrain(*this, linear_factor, angular_factor);
}

cSimObj::eType cSimObj::GetType() const
{
	return mType;
}

cSimObj::eShape cSimObj::GetShape() const
{
	return eShapeInvalid;
}

bool cSimObj::HasSimBody() const
{
	return mSimBody != nullptr;
}

const std::unique_ptr<btRigidBody>& cSimObj::GetSimBody() const
{
	return mSimBody;
}

const std::unique_ptr<btCollisionShape>& cSimObj::GetCollisionShape() const
{
	return mShape;
}

const std::shared_ptr<cWorld>& cSimObj::GetWorld() const
{
	return mWorld;
}

void cSimObj::Init(const std::shared_ptr<cWorld>& world)
{
	RemoveFromWorld();
	mSimBody->setUserPointer(this);
	AddToWorld(world);
}

void cSimObj::AddToWorld(const std::shared_ptr<cWorld>& world)
{
	if (mWorld != nullptr)
	{
		RemoveFromWorld();
	}

	mWorld = world;
	mWorld->AddObject(*this);
}

void cSimObj::RemoveFromWorld()
{
	if (mWorld != nullptr && mSimBody != nullptr)
	{
		int num_cons = GetNumConstraints();;
		for (int c = num_cons - 1; c >= 0; --c)
		{
			cWorld::tConstraintHandle cons = GetConstraint(c);
			mWorld->RemoveConstraint(cons);
		}

		if (mCons.IsValid())
		{
			mWorld->RemoveConstraint(mCons);
		}

		mWorld->RemoveObject(*this);
		mWorld.reset();
		mSimBody.reset();
	}
}

int cSimObj::GetNumConstraints() const
{
	return mSimBody->getNumConstraintRefs();
}

cWorld::tConstraintHandle cSimObj::GetConstraint(int c) const
{
	cWorld::tConstraintHandle handle;
	handle.mCons = mSimBody->getConstraintRef(c);
	return handle;
}
