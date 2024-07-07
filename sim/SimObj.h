#pragma once

#include <memory>

#include "sim/World.h"

class cSimObj : public btDefaultMotionState
{
public:
	enum eType
	{
		eTypeDynamic,
		eTypeStatic,
		eTypeMax
	};

	enum eObjType
	{
		eObjTypeCharacter,
		eObjTypeGround,
		eObjTypeObstacle,
		eObjTypeMax
	};

	enum eShape
	{
		eShapeInvalid,
		eShapeBox,
		eShapePlane,
		eShapeCapsule,
		eShapeSphere,
		eShapeCylinder,
		eShapeMax,
	};
	struct ExternalForce
	{

		ExternalForce();

		bool collision;
		tVector external_force_ground;
		tVector external_force_obstacle;
		tVector external_force_agent;

		tVector external_force_ground_ins;
		tVector external_force_obstacle_ins;
		tVector external_force_agent_ins;

		tVector force_ground_pos;
		tVector force_obstacle_pos;
		tVector force_agent_pos;

		int num_force_ground;
		int num_force_obstacle;
		int num_force_agent;
	};
	virtual ~cSimObj();

	mutable ExternalForce mExternalForce;

	virtual eObjType GetObjType() const;
	virtual void SetObjType(eObjType obj_type);

	virtual ExternalForce GetExternalForce(); 
	virtual void ResetExternalForce();

	virtual void SetAgentID(int id);
	virtual int GetAgentID() const;

	virtual tVector GetPos() const;
	virtual void SetPos(const tVector& pos);
	virtual void GetRotation(tVector& out_axis, double& out_theta) const;
	virtual tQuaternion GetRotation() const;
	virtual void SetRotation(const tVector& axis, double theta);
	virtual void SetRotation(const tQuaternion& q);
	virtual tVector GetLinearVelocity() const;
	virtual tVector GetTotalForce() const;//Ray
	//virtual tVector GetTorqueImpulse() const;
	//virtual tVector GetCentralImpulse() const;
	virtual tVector GetLinearForce() ;
	virtual tVector GetForce();
	virtual tVector GetVelocity() const;
	virtual tVector GetNextLinearVelocity() const;
	virtual void SetPreLinearVelocity();
	virtual void SetForce();
	virtual void UpdateForce();
	virtual tVector GetmTorque();

	virtual tVector GetLinearVelocity(const tVector& local_pos) const;
	virtual void SetLinearVelocity(const tVector& vel);
	virtual tVector GetAngularVelocity() const;
	virtual void SetAngularVelocity(const tVector& vel);
	virtual tMatrix GetWorldTransform() const;
	virtual tMatrix GetLocalTransform() const;

	virtual void SetDamping(double linear_damping, double angular_damping);

	virtual double GetMass() const;

	virtual tVector WorldToLocalPos(const tVector& world_pos) const;
	virtual tVector LocalToWorldPos(const tVector& local_pos) const;
	virtual tMatrix3 GetLocalToWorldRotMat() const;

	virtual void ApplyForce(const tVector& force);
	virtual void ApplyForce(const tVector& force, const tVector& local_pos);
	virtual void ApplyTorque(const tVector& torque);
	virtual void ClearForces();

	virtual void RegisterContact();
	virtual void RegisterContact(int contact_flags, int filter_flags);
	virtual void UpdateContact(int contact_flags, int filter_flags);
	virtual const cContactManager::tContactHandle& GetContactHandle() const;
	virtual bool IsInContact() const;
	virtual tVector GetContactPt() const;
	virtual tVector GetGravity() const;
	virtual int GetNumContacts() const;
	virtual tVector GetContactImpulse() const;
	virtual tVector GetContactImpulseGround() const;
	virtual tVector GetContactImpulseCharacter() const;
	virtual tVector GetContactImpulseObstacle() const;

	virtual tVector GetContactPtGround() const;
	virtual tVector GetContactPtCharacter() const;
	virtual tVector GetContactPtObstacle() const;

	virtual int GetNumContactGround() const;
	virtual int GetNumContactCharacter() const;
	virtual int GetNumContactObstacle() const;


	virtual tVector GetPerturbForce() const;
	virtual void ClearPertubForce();

	virtual void ResetContactImpulse() ;


	virtual short GetColGroup() const;
	virtual void SetColGroup(short col_group);
	virtual short GetColMask() const;
	virtual void SetColMask(short col_mask);
	virtual void SetKinematicObject(bool is_kin);
	virtual bool IsKinematicObject() const;

	virtual void DisableDeactivation();
	virtual void CalcAABB(tVector& out_min, tVector& out_max) const;

	virtual void Constrain(const tVector& linear_factor, const tVector& angular_factor);

	virtual eType GetType() const;
	virtual eShape GetShape() const;

	virtual bool HasSimBody() const;
	virtual const std::unique_ptr<btRigidBody>& GetSimBody() const;
	virtual const std::unique_ptr<btCollisionShape>& GetCollisionShape() const;
	virtual const std::shared_ptr<cWorld>& GetWorld() const;

protected:
	std::shared_ptr<cWorld> mWorld;
	std::unique_ptr<btRigidBody> mSimBody;
	std::unique_ptr<btCollisionShape> mShape;

	cWorld::tConstraintHandle mCons;

	cContactManager::tContactHandle mContactHandle;
	tVector pre_linearvel;
	tVector pre_vel;
	tVector force;
	tVector m_torque;
	tVector next_vel;
	tVector m_perturbforce;

	eObjType mObjType;
	int mAgentID;
	eType mType;
	short mColGroup;
	short mColMask;

	cSimObj();

	virtual void Init(const std::shared_ptr<cWorld>& world);
	virtual void AddToWorld(const std::shared_ptr<cWorld>& world);
	virtual void RemoveFromWorld();

	virtual int GetNumConstraints() const;
	virtual cWorld::tConstraintHandle GetConstraint(int c) const;
};
