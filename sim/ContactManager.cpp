#include "ContactManager.h"
#include "sim/World.h"
#include "SimObj.h"
#include <iostream>

const int cContactManager::gInvalidID = -1;

cContactManager::tContactHandle::tContactHandle()
{
	mID = gInvalidID;
	mFlags = gFlagAll;
	mFilterFlags = gFlagAll;
}

cContactManager::tContactEntry::tContactEntry()
{
	mFlags = gFlagAll;
	mFilterFlags = gFlagAll;
	mInContact = false;
}

bool cContactManager::tContactHandle::IsValid() const
{
	return mID != gInvalidID;
}

cContactManager::cContactManager(cWorld &world)
	: mWorld(world)
{
}

cContactManager::~cContactManager()
{
}

void cContactManager::Init()
{
	Clear();
}

void cContactManager::Reset()
{
	for (int i = 0; i < GetNumEntries(); ++i)
	{
		tContactEntry &entry = mContactEntries[i];
		entry.mInContact = false;
	}
}

void cContactManager::Clear()
{
	mContactEntries.clear();
}

void cContactManager::Update()
{
	ClearContacts();
	std::unique_ptr<btDiscreteDynamicsWorld> &bt_world = mWorld.GetInternalWorld();
	double timestep = mWorld.GetTimeStep();

	int num_manifolds = bt_world->getDispatcher()->getNumManifolds();
	for (int i = 0; i < num_manifolds; ++i)
	{
		btPersistentManifold *mani = bt_world->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject *obj0 = static_cast<const btCollisionObject *>(mani->getBody0());
		const btCollisionObject *obj1 = static_cast<const btCollisionObject *>(mani->getBody1());

		const cSimObj *sim_obj0 = static_cast<const cSimObj *>(obj0->getUserPointer());
		const cSimObj *sim_obj1 = static_cast<const cSimObj *>(obj1->getUserPointer());

		sim_obj0->mExternalForce.external_force_agent_ins = tVector::Zero();
		sim_obj0->mExternalForce.external_force_obstacle_ins = tVector::Zero();
		sim_obj0->mExternalForce.external_force_ground_ins = tVector::Zero();

		sim_obj1->mExternalForce.external_force_agent_ins = tVector::Zero();
		sim_obj1->mExternalForce.external_force_obstacle_ins = tVector::Zero();
		sim_obj1->mExternalForce.external_force_ground_ins = tVector::Zero();

		int num_contacts = mani->getNumContacts();
		for (int j = 0; j < num_contacts; ++j)
		{
			btManifoldPoint &pt = mani->getContactPoint(j);
			// std::cout << " Detected contact1: " << pt.getDistance() << std::endl;
			btScalar dist_tol = 0.001f;
			if (pt.getDistance() <= dist_tol)
			{
				// std::cout << " Detected contact: " << std::endl;
				// const cSimObj* sim_obj0 = static_cast<const cSimObj*>(obj0->getUserPointer());
				// const cSimObj* sim_obj1 = static_cast<const cSimObj*>(obj1->getUserPointer());
				cSimObj::eObjType obj0_type = sim_obj0->GetObjType();
				cSimObj::eObjType obj1_type = sim_obj1->GetObjType();

				const tContactHandle &h0 = sim_obj0->GetContactHandle();
				const tContactHandle &h1 = sim_obj1->GetContactHandle();

				bool valid_contact = IsValidContact(h0, h1);
				if (valid_contact)
				{
					if (h0.IsValid())
					{
						mContactEntries[h0.mID].mInContact = true;
						mContactEntries[h0.mID].mContactPt = mWorld.GetManifoldPtA(pt);
						mContactEntries[h0.mID].mContactImpulse = mWorld.GetManifoldImpulse(pt);
						// std::cout << "contact normal " << mContactEntries[h0.mID].mContactImpulse << std::endl;

						tVector pt_pos = mWorld.GetManifoldPtA(pt);
						double up = mWorld.GetManifoldImpulseValue(pt);
						double left = mWorld.GetManifoldImpulseLateral1(pt);
						double right = mWorld.GetManifoldImpulseLateral2(pt);

						up = up / timestep;
						left = left / timestep;
						right = right / timestep;

						tVector normal = tVector(pt.m_normalWorldOnB[0], pt.m_normalWorldOnB[1], pt.m_normalWorldOnB[2], 0).normalized();
						tVector frict1 = tVector(pt.m_lateralFrictionDir1[0], pt.m_lateralFrictionDir1[1], pt.m_lateralFrictionDir1[2], 0).normalized();
						tVector frict2 = tVector(pt.m_lateralFrictionDir2[0], pt.m_lateralFrictionDir2[1], pt.m_lateralFrictionDir2[2], 0).normalized();

						tVector net_impulse_world = frict1 * left + up * normal + right * frict2;

						if (obj0_type == cSimObj::eObjTypeCharacter)
						{
							// printf("ContactManager::Update obj0 type Character mID: %d\n", h0.mID);
							switch (obj1_type)
							{
							case cSimObj::eObjTypeGround:
								sim_obj0->mExternalForce.external_force_ground += net_impulse_world;
								sim_obj0->mExternalForce.external_force_ground_ins += net_impulse_world;
								sim_obj0->mExternalForce.force_ground_pos += pt_pos;
								sim_obj0->mExternalForce.num_force_ground += 1;
								break;
							case cSimObj::eObjTypeCharacter:
								sim_obj0->mExternalForce.external_force_agent += net_impulse_world;
								sim_obj0->mExternalForce.external_force_agent_ins += net_impulse_world;
								sim_obj0->mExternalForce.force_agent_pos += pt_pos;
								sim_obj0->mExternalForce.num_force_agent += 1;
								break;
							case cSimObj::eObjTypeObstacle:
								sim_obj0->mExternalForce.external_force_obstacle += net_impulse_world;
								sim_obj0->mExternalForce.external_force_obstacle_ins += net_impulse_world;
								sim_obj0->mExternalForce.force_obstacle_pos += pt_pos;
								sim_obj0->mExternalForce.num_force_obstacle += 1;
								break;
							default:
								break;
							}
						}
					}

					if (h1.IsValid())
					{
						mContactEntries[h1.mID].mInContact = true;
						mContactEntries[h1.mID].mContactPt = mWorld.GetManifoldPtB(pt);
						mContactEntries[h1.mID].mContactImpulse = mWorld.GetManifoldImpulse(pt);

						tVector pt_pos = mWorld.GetManifoldPtB(pt);
						double up = -mWorld.GetManifoldImpulseValue(pt);
						double left = -mWorld.GetManifoldImpulseLateral1(pt);
						double right = -mWorld.GetManifoldImpulseLateral2(pt);

						up = up / timestep;
						left = left / timestep;
						right = right / timestep;

						tVector normal = tVector(pt.m_normalWorldOnB[0], pt.m_normalWorldOnB[1], pt.m_normalWorldOnB[2], 0).normalized();
						tVector frict1 = tVector(pt.m_lateralFrictionDir1[0], pt.m_lateralFrictionDir1[1], pt.m_lateralFrictionDir1[2], 0).normalized();
						tVector frict2 = tVector(pt.m_lateralFrictionDir2[0], pt.m_lateralFrictionDir2[1], pt.m_lateralFrictionDir2[2], 0).normalized();

						tVector net_impulse_world = frict1 * left + up * normal + right * frict2;

						if (obj1_type == cSimObj::eObjTypeCharacter)
						{
							switch (obj0_type)
							{
							case cSimObj::eObjTypeGround:
								sim_obj1->mExternalForce.external_force_ground += net_impulse_world;
								sim_obj1->mExternalForce.external_force_ground_ins += net_impulse_world;
								sim_obj1->mExternalForce.force_ground_pos += pt_pos;
								sim_obj1->mExternalForce.num_force_ground += 1;
								break;
							case cSimObj::eObjTypeCharacter:
								sim_obj1->mExternalForce.external_force_agent += net_impulse_world;
								sim_obj1->mExternalForce.external_force_agent_ins += net_impulse_world;
								sim_obj1->mExternalForce.force_agent_pos += pt_pos;
								sim_obj1->mExternalForce.num_force_agent += 1;
								break;
							case cSimObj::eObjTypeObstacle:
								sim_obj1->mExternalForce.external_force_obstacle += net_impulse_world;
								sim_obj1->mExternalForce.external_force_obstacle_ins += net_impulse_world;
								sim_obj1->mExternalForce.force_obstacle_pos += pt_pos;
								sim_obj1->mExternalForce.num_force_obstacle += 1;
								break;
							default:
								break;
							}
						}
					}
				}
			}
		}
	}
}

cContactManager::tContactHandle cContactManager::RegisterContact(int contact_flags, int filter_flags)
{
	tContactHandle handle;
	handle.mFlags = contact_flags;
	handle.mFilterFlags = filter_flags;
	handle.mID = RegisterNewID();

	tContactEntry &entry = mContactEntries[handle.mID];
	entry.mFlags = contact_flags;
	entry.mFilterFlags = filter_flags;

	assert(handle.IsValid());
	return handle;
}

void cContactManager::UpdateContact(const cContactManager::tContactHandle &handle)
{
	assert(handle.IsValid());
	tContactEntry &entry = mContactEntries[handle.mID];
	entry.mFlags = handle.mFlags;
	entry.mFilterFlags = handle.mFilterFlags;
}

int cContactManager::GetNumEntries() const
{
	return static_cast<int>(mContactEntries.size());
}

bool cContactManager::IsInContact(const tContactHandle &handle) const
{
	if (handle.IsValid())
	{
		return mContactEntries[handle.mID].mInContact;
	}
	return false;
}

tVector cContactManager::GetContactPt(const tContactHandle &handle) const
{
	if (handle.IsValid())
	{
		return mContactEntries[handle.mID].mContactPt;
	}
	return tVector::Zero();
}

tVector cContactManager::GetContactImpulse(const tContactHandle &handle) const
{
	/// Should be valid and in contact
	if (handle.IsValid() && mContactEntries[handle.mID].mInContact)
	{
		return mContactEntries[handle.mID].mContactImpulse;
	}
	return tVector::Zero();
}

int cContactManager::RegisterNewID()
{
	int id = gInvalidID;
	id = static_cast<int>(mContactEntries.size());
	mContactEntries.resize(id + 1);
	return id;
}

void cContactManager::ClearContacts()
{
	int num_entries = GetNumEntries();
	for (int i = 0; i < num_entries; ++i)
	{
		tContactEntry &curr_entry = mContactEntries[i];
		curr_entry.mInContact = false;
		curr_entry.mContactPt.setZero();
	}
}

bool cContactManager::IsValidContact(const tContactHandle &h0, const tContactHandle &h1) const
{
	bool valid_h0 = ((h0.mFilterFlags & h1.mFlags) != 0);
	bool valid_h1 = ((h1.mFilterFlags & h0.mFlags) != 0);
	bool valid_contact = valid_h0 && valid_h1;
	return valid_contact;
}
