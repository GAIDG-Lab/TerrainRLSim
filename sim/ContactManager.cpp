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
		tContactEntry &entry = mContactEntries[i]; // 15 entries
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
	int num_manifolds = bt_world->getDispatcher()->getNumManifolds(); // current is [15,16,17,...,37,38]
	//printf("cContactManager::Update num_manifolds: %d\n", num_manifolds);
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
		//******* sum of forces of all contacts for each mainforld equals the net force of each simulation step
		int num_contacts = mani->getNumContacts();
		// printf("ContactManager::Update num_contacts: %d\n", num_contacts);
		for (int j = 0; j < num_contacts; ++j)
		{
			btManifoldPoint &pt = mani->getContactPoint(j);
			// printf("ContactManager normalWorldOnB: %f %f %f\n", pt.m_normalWorldOnB[0], pt.m_normalWorldOnB[1],pt.m_normalWorldOnB[2]);
			//  std::cout << " Detected contact1: " << pt.getDistance() << std::endl;
			btScalar dist_tol = 0.001f;
			if (pt.getDistance() <= dist_tol)
			{
				// std::cout << " Detected contact: " << std::endl;

				cSimObj::eObjType obj0_type = sim_obj0->GetObjType();
				cSimObj::eObjType obj1_type = sim_obj1->GetObjType();

				const tContactHandle &h0 = sim_obj0->GetContactHandle();
				const tContactHandle &h1 = sim_obj1->GetContactHandle();

				/**
				switch(obj0_type)
				{
				case cSimObj::eObjTypeGround:
					printf("\nContactManager::Update obj0 type Ground mID: %d\n", h0.mID);
					break;
				case cSimObj::eObjTypeCharacter:
					printf("\nContactManager::Update obj0 type Character mID: %d\n", h0.mID);
					break;
				case cSimObj::eObjTypeObstacle:
					printf("\nContactManager::Update obj0 type Obstacle mID: %d\n", h0.mID);
					break;
				default:
					printf("\nContactManager::Update obj0 type none mID: %d\n", h0.mID);
				}
				switch(obj1_type)
				{
				case cSimObj::eObjTypeGround:
					printf("ContactManager::Update obj1 type Ground mID: %d\n", h1.mID);
					break;
				case cSimObj::eObjTypeCharacter:
					printf("ContactManager::Update obj1 type Character mID: %d\n", h1.mID);
					break;
				case cSimObj::eObjTypeObstacle:
					printf("ContactManager::Update obj1 type Obstacle mID: %d\n", h1.mID);
					break;
				default:
					printf("ContactManager::Update obj1 type none mID: %d\n", h1.mID);
				}
				**/

				bool valid_contact = IsValidContact(h0, h1);
				if (valid_contact)
				{
					// printf("simObj 0 flag: %d, simObj 1 flag: %d\n", mContactEntries[h0.mID].mFlags, mContactEntries[h1.mID].mFlags);
					// printf("SimObj0 type: %s, SimObj1 type: %s\n", typeid(obj0).name(), typeid(obj1).name());

					if (h0.IsValid())
					{

						mContactEntries[h0.mID].mInContact = true;
						mContactEntries[h0.mID].mContactPt = mWorld.GetManifoldPtA(pt);
						mContactEntries[h0.mID].mContactImpulse = mWorld.GetManifoldImpulse(pt);

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
								// if(j == 0)
								//{
								sim_obj0->mExternalForce.num_force_ground += 1;
								//}

								// mContactEntriesObser[h0.mID].mContactImpulseGround = net_impulse_world;
								// mContactEntriesObser[h0.mID].mContactPtGround = pt_pos;
								// mContactEntriesObser[h0.mID].mNumImpulseGround += 1;
								break;
							case cSimObj::eObjTypeCharacter:
								sim_obj0->mExternalForce.external_force_agent += net_impulse_world;
								sim_obj0->mExternalForce.external_force_agent_ins += net_impulse_world;
								sim_obj0->mExternalForce.force_agent_pos += pt_pos;
								// if(j == 0)
								//{
								sim_obj0->mExternalForce.num_force_agent += 1;
								//}

								// mContactEntriesObser[h0.mID].mContactImpulseCharacter = net_impulse_world;
								// mContactEntriesObser[h0.mID].mContactPtCharacter = pt_pos;
								// mContactEntriesObser[h0.mID].mNumImpulseCharacter += 1;
								break;
							case cSimObj::eObjTypeObstacle:
								sim_obj0->mExternalForce.external_force_obstacle += net_impulse_world;
								sim_obj0->mExternalForce.external_force_obstacle_ins += net_impulse_world;
								sim_obj0->mExternalForce.force_obstacle_pos += pt_pos;
								// if(j == 0)
								//{
								sim_obj0->mExternalForce.num_force_obstacle += 1;
								//}

								// mContactEntriesObser[h0.mID].mContactImpulseObstacle = net_impulse_world;
								// mContactEntriesObser[h0.mID].mContactPtObstacle = pt_pos;
								// mContactEntriesObser[h0.mID].mNumImpulseObstacle += 1;
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
								// if(j == 0)
								//{
								sim_obj1->mExternalForce.num_force_ground += 1;
								//}

								// mContactEntriesObser[h1.mID].mContactImpulseGround = net_impulse_world;
								// mContactEntriesObser[h1.mID].mContactPtGround = pt_pos;
								// mContactEntriesObser[h1.mID].mNumImpulseGround += 1;
								break;
							case cSimObj::eObjTypeCharacter:
								sim_obj1->mExternalForce.external_force_agent += net_impulse_world;
								sim_obj1->mExternalForce.external_force_agent_ins += net_impulse_world;
								sim_obj1->mExternalForce.force_agent_pos += pt_pos;
								// if(j == 0)
								//{
								sim_obj1->mExternalForce.num_force_agent += 1;
								//}

								// mContactEntriesObser[h1.mID].mContactImpulseCharacter = net_impulse_world;
								// mContactEntriesObser[h1.mID].mContactPtCharacter = pt_pos;
								// mContactEntriesObser[h1.mID].mNumImpulseCharacter += 1;
								break;
							case cSimObj::eObjTypeObstacle:
								sim_obj1->mExternalForce.external_force_obstacle += net_impulse_world;
								sim_obj1->mExternalForce.external_force_obstacle_ins += net_impulse_world;
								sim_obj1->mExternalForce.force_obstacle_pos += pt_pos;
								// if(j == 0)
								//{
								sim_obj1->mExternalForce.num_force_obstacle += 1;
								//}

								// mContactEntriesObser[h1.mID].mContactImpulseObstacle = net_impulse_world;
								// mContactEntriesObser[h1.mID].mContactPtObstacle = pt_pos;
								// mContactEntriesObser[h1.mID].mNumImpulseObstacle += 1;
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
	//printf("ContactManager RegisterContact\n");
	tContactHandle handle;
	handle.mFlags = contact_flags;
	handle.mFilterFlags = filter_flags;
	handle.mID = RegisterNewID();

	tContactEntry& entry = mContactEntries[handle.mID];
	entry.mFlags = contact_flags;
	entry.mFilterFlags = filter_flags;
	
	tContactObser& entryobser = mContactEntriesObser[handle.mID];
	entryobser.mContactImpulseGround = tVector::Zero();
	entryobser.mContactPtGround = tVector::Zero();
	entryobser.mNumImpulseGround = 0;

	entryobser.mContactImpulseCharacter = tVector::Zero();
	entryobser.mContactPtCharacter = tVector::Zero();
	entryobser.mNumImpulseCharacter = 0;

	entryobser.mContactImpulseObstacle = tVector::Zero();
	entryobser.mContactPtObstacle = tVector::Zero();
	entryobser.mNumImpulseObstacle = 0;

	assert(handle.IsValid());
	return handle;
}

void cContactManager::UpdateContact(const cContactManager::tContactHandle &handle)
{
	assert(handle.IsValid());
	tContactEntry& entry = mContactEntries[handle.mID];
	entry.mFlags = handle.mFlags;
	entry.mFilterFlags = handle.mFilterFlags;
}

int cContactManager::GetNumEntries() const
{
	return static_cast<int>(mContactEntries.size());
}
int cContactManager::GetNumEntriesObser() const
{
	return static_cast<int>(mContactEntriesObser.size());
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
/*
tVector cContactManager::GetContactImpulse(const tContactHandle &handle) const
{
	/// Should be valid and in contact
	if (handle.IsValid() && mContactEntries[handle.mID].mInContact)
	{
		return mContactEntries[handle.mID].mContactImpulse;
	}
	return tVector::Zero();
}
*/
int cContactManager::GetNumContacts(const tContactHandle& handle) const
{
	return mContactEntriesObser[handle.mID].mNumImpulseGround +
		mContactEntriesObser[handle.mID].mNumImpulseCharacter +
		mContactEntriesObser[handle.mID].mNumImpulseObstacle;
}
tVector cContactManager::GetContactImpulse(const tContactHandle& handle) const
{
	if (handle.IsValid() && mContactEntries[handle.mID].mInContact)
	{
		return mContactEntries[handle.mID].mContactImpulse;
	}
	return tVector::Zero();
}
tVector cContactManager::GetContactImpulseGround(const tContactHandle& handle) const
{
	return mContactEntriesObser[handle.mID].mContactImpulseGround;
}
tVector cContactManager::GetContactImpulseCharacter(const tContactHandle& handle) const
{
	return mContactEntriesObser[handle.mID].mContactImpulseCharacter;
}
tVector cContactManager::GetContactImpulseObstacle(const tContactHandle& handle) const
{
	return mContactEntriesObser[handle.mID].mContactImpulseObstacle;
}

tVector cContactManager::GetContactPtGround(const tContactHandle& handle) const
{
	return mContactEntriesObser[handle.mID].mContactPtGround;
}
tVector cContactManager::GetContactPtCharacter(const tContactHandle& handle) const
{
	return mContactEntriesObser[handle.mID].mContactPtCharacter;
}
tVector cContactManager::GetContactPtObstacle(const tContactHandle& handle) const
{
	return mContactEntriesObser[handle.mID].mContactPtObstacle;
}

int cContactManager::GetNumContactGround(const tContactHandle& handle) const
{
	return mContactEntriesObser[handle.mID].mNumImpulseGround;
}
int cContactManager::GetNumContactCharacter(const tContactHandle& handle) const
{
	return mContactEntriesObser[handle.mID].mNumImpulseCharacter;
}
int cContactManager::GetNumContactObstacle(const tContactHandle& handle) const
{
	return mContactEntriesObser[handle.mID].mNumImpulseObstacle;
}

void cContactManager::ResetContactImpulse(const tContactHandle& handle) 
{
	mContactEntriesObser[handle.mID].mContactImpulseGround.setZero();
	mContactEntriesObser[handle.mID].mContactPtGround = tVector::Zero();
	mContactEntriesObser[handle.mID].mNumImpulseGround = 0;

	mContactEntriesObser[handle.mID].mContactImpulseCharacter = tVector::Zero();
	mContactEntriesObser[handle.mID].mContactPtCharacter = tVector::Zero();
	mContactEntriesObser[handle.mID].mNumImpulseCharacter = 0;

	mContactEntriesObser[handle.mID].mContactImpulseObstacle = tVector::Zero();
	mContactEntriesObser[handle.mID].mContactPtObstacle = tVector::Zero();
	mContactEntriesObser[handle.mID].mNumImpulseObstacle = 0;	
}

int cContactManager::RegisterNewID()
{
	int id = gInvalidID;
	id = static_cast<int>(mContactEntries.size());
	mContactEntries.resize(id + 1);
	mContactEntriesObser.resize(id + 1);
	return id;
}

void cContactManager::ClearObser()
{
	for (int i = 0; i < GetNumEntriesObser(); ++i)
	{
		tContactObser& curr_entry = mContactEntriesObser[i];
		curr_entry.mContactImpulseGround = tVector::Zero();
		curr_entry.mContactPtGround = tVector::Zero();
		curr_entry.mNumImpulseGround = 0;

		curr_entry.mContactImpulseCharacter = tVector::Zero();
		curr_entry.mContactPtCharacter = tVector::Zero();
		curr_entry.mNumImpulseCharacter = 0;

		curr_entry.mContactImpulseObstacle = tVector::Zero();
		curr_entry.mContactPtObstacle = tVector::Zero();
		curr_entry.mNumImpulseObstacle = 0;
	}
}

void cContactManager::ClearContacts()
{
	int num_entries = GetNumEntries();
	for (int i = 0; i < num_entries; ++i)
	{
		tContactEntry& curr_entry = mContactEntries[i];
		curr_entry.mInContact = false;
		curr_entry.mContactPt.setZero();
		curr_entry.mContactImpulse.setZero();
	}
}

bool cContactManager::IsValidContact(const tContactHandle &h0, const tContactHandle &h1) const
{
	bool valid_h0 = ((h0.mFilterFlags & h1.mFlags) != 0);
	bool valid_h1 = ((h1.mFilterFlags & h0.mFlags) != 0);
	bool valid_contact = valid_h0 && valid_h1;
	return valid_contact;
}
