#pragma once

#include "sim/SimObj.h"

class cSimCylinder : public cSimObj
{
public:
	struct tParams
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		tParams();

		eType mType;
		double mMass;
		double mFriction;
		tVector mPos;
		tVector mVel;
		double mHeight;
		double mRadius;
		tVector mAxis;
		double mTheta;
	};

	cSimCylinder();
	virtual ~cSimCylinder();

	virtual void Init(const std::shared_ptr<cWorld>& world, const tParams& params);
	virtual double GetHeight() const;
	virtual double GetRadius() const;

	virtual eShape GetShape() const;
	virtual tVector GetSize() const;

protected:
};
