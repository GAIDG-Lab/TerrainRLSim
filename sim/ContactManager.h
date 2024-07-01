#pragma once

#include <memory>
#include "util/MathUtil.h"

class cWorld;

class cContactManager
{
public:
	const static int gInvalidID;
	const static short gFlagAll = -1;
	const static short gFlagNone = 0;
	const static short gFlagRayTest = 1;

	struct tContactHandle
	{
		int mID;
		int mFlags;
		int mFilterFlags;

		tContactHandle();
		bool IsValid() const;
	};

	cContactManager(cWorld& world);
	virtual ~cContactManager();

	virtual void Init();
	virtual void Reset();
	virtual void Clear();
	virtual void Update();

	virtual tContactHandle RegisterContact(int contact_flags, int filter_flags);
	virtual void UpdateContact(const cContactManager::tContactHandle& handle);
	virtual int GetNumEntries() const;
	virtual int GetNumEntriesObser() const;

	virtual bool IsInContact(const tContactHandle& handle) const;
	virtual tVector GetContactPt(const tContactHandle& handle) const;

	virtual tVector GetContactImpulse(const tContactHandle& handle) const;
	virtual tVector GetContactImpulseGround(const tContactHandle& handle) const;
	virtual tVector GetContactImpulseCharacter(const tContactHandle& handle) const;
	virtual tVector GetContactImpulseObstacle(const tContactHandle& handle) const;

	virtual tVector GetContactPtGround(const tContactHandle& handle) const;
	virtual tVector GetContactPtCharacter(const tContactHandle& handle) const;
	virtual tVector GetContactPtObstacle(const tContactHandle& handle) const;

	virtual int GetNumContactGround(const tContactHandle& handle) const;
	virtual int GetNumContactCharacter(const tContactHandle& handle) const;
	virtual int GetNumContactObstacle(const tContactHandle& handle) const;

	virtual int GetNumContacts(const tContactHandle& handle) const;

	virtual void ResetContactImpulse(const tContactHandle& handle) ;

protected:
	struct tContactEntry
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		tContactEntry();

		int mFlags;
		int mFilterFlags;
		bool mInContact;
		tVector mContactPt;
		tVector mContactImpulse;
	};

	struct tContactObser
	{
		tVector mContactImpulseGround;
		tVector mContactPtGround;

		tVector mContactImpulseCharacter;
		tVector mContactPtCharacter;

		tVector mContactImpulseObstacle;
		tVector mContactPtObstacle;

		int mNumImpulseGround;
		int mNumImpulseCharacter;
		int mNumImpulseObstacle;
	};

	cWorld& mWorld;
	std::vector<tContactEntry, Eigen::aligned_allocator<tContactEntry>> mContactEntries;
	std::vector<tContactObser, Eigen::aligned_allocator<tContactObser>> mContactEntriesObser;

	virtual int RegisterNewID();
	virtual void ClearContacts();
	virtual void ClearObser();
	virtual bool IsValidContact(const tContactHandle& h0, const tContactHandle& h1) const;
};
