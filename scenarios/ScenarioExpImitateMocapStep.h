#pragma once

#include "scenarios/ScenarioExpImitateMocapTarget.h"
#include "sim/Ground.h"
#include "sim/BipedStepController3D.h"
#include "anim/KinController.h"

//#define ENABLE_KIN_CONTROLLER_TEST

class cScenarioExpImitateMocapStep : virtual public cScenarioExpImitateMocapTarget
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef cBipedStepController3D::eStance eStance;

	cScenarioExpImitateMocapStep();
	virtual ~cScenarioExpImitateMocapStep();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Init();
	virtual void Reset();

	virtual const cBipedStepController3D::tStepPlan& GetStepPlan() const;
	virtual bool EnableTargetPos() const;
	virtual void EnableTargetPos(bool enable);

	virtual std::string GetName() const;

	virtual double CalcReward() const;
	virtual double CalcRewardStep(const cBipedStepController3D::tStepPlan& step_plan) const;

	virtual bool endOfEpoch() const;

protected:
	cBipedStepController3D::tStepPlan mStepPlan;
	double mStepsPerPeriod;
	double mStepLenMin;
	double mStepLenMax;
	double mCurrStepLen;
	double mChangeStepLenProb;
	double mStepWidthMean;
	double mRootHeadingDelta;
	double mMaxHeadingDelta;
	double mMaxHeadingTurnRate;
	double mSharpTurnProb;
	double mInvalidStepHeightThreshold;
	bool mRandomizeInititalRotation;
	bool mMirrorArms;

	double mStepFailDist;
	bool mStepFail;
	bool mEnableTargetPos;

	// Assigns a weight to the reward term that incentivizes the character to walk with its feet further apart
	double mWideWalkWeight = 0;

	std::vector<int> mEndEffectors;
	std::string mKinCtrlFile;

	virtual void SetupKinController();
	virtual void BuildKinController(std::shared_ptr<cKinController>& out_ctrl) const;
	virtual void SetupControllerParams(cTerrainRLCtrlFactory::tCtrlParams& out_params) const;

	virtual void InitEndEffectors();
	virtual void ResetParams();
	virtual bool EnableUpdateStepPlan() const;

	virtual void HandleNewActionUpdate();
	virtual void ResetKinChar();
	virtual void SyncKinChar();
	virtual void SwatchEndEffectorPose(const std::vector<int>& end_effector, Eigen::VectorXd& out_pose) const;

	virtual bool HasFallen() const;

	virtual void PostSubstepUpdate(double time_step);
	virtual void ResetStepPlan();
	virtual void UpdateStepPlan(double time_step, bool force_update = false);
	virtual double CalcNextHeading(double heading);
	virtual tVector CalcNextStepPos(eStance curr_stance, double heading, const tVector& step_pos0);
	virtual void HandleNewActionUpdateKinController();

	virtual void ApplyStepPlan(const cBipedStepController3D::tStepPlan& step);
	virtual eStance GetStance() const;
	virtual eStance GetStance(double time) const;
	virtual int GetStanceFootJoint(eStance stance) const;
	virtual int GetSwingFootJoint(eStance stance) const;

	virtual double CalcStepPhase() const;

	virtual void CalcStyleRewardErr(double& out_pose_err, double& out_vel_err) const;
};
