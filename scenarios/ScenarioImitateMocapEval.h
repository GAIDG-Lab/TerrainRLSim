#pragma once

#include "scenarios/ScenarioPoliEval.h"
#include "scenarios/ScenarioExpImitateMocap.h"
#include "anim/KinCharacter.h"

class cScenarioImitateMocapEval : virtual public cScenarioPoliEval, virtual public cScenarioExpImitateMocap
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioImitateMocapEval();
	virtual ~cScenarioImitateMocapEval();

	virtual void GetPoseErrResult(double& out_err, int& out_count) const;

	virtual void ResetRecord();
	virtual void SetResetPhase(double phase_min, double phase_max);
	virtual void SetResetPhaseSamples(int num_samples);

	virtual std::string GetName() const;

protected:

	double mResetPhaseMin;
	double mResetPhaseMax;
	int mResetPhaseSamples;

	bool mRecordPoseError;
	double mPoseErr;
	int mPoseErrCount;

	virtual void UpdatePoseErr();
	virtual void RecordAction(const std::string& out_file);
	virtual double CalcRandKinResetTime();

	virtual void ParseMiscArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void InitMisc();
	virtual void ResetMisc();
	virtual void ClearMisc();
	virtual void UpdateMisc(double time_elapsed);
};
