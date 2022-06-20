#include "scenarios/ScenarioExpImitateMocapStep.h"
#include "sim/RBDUtil.h"
#include "anim/MocapStepController.h"
#include "anim/MotionFieldStepController.h"

#define ENABLE_STEP_FAIL
//#define DISABLE_FOOTSTEP_REWARD
//#define ENABLE_IND_POSE_ERR
//#define ENABLE_MOTION_FIELD
#define SAMPLE_INIT_STATES

//#define ENABLE_STYLE_MARCH
//#define ENABLE_STYLE_LEAN
//#define ENABLE_STYLE_SIDE_LEAN
//#define ENABLE_STYLE_STRAIGHT_KNEE_RIGHT
//#define ENABLE_STYLE_STRAIGHT_KNEE_LEFT

//#define ENABLE_ADAPTIVE_STEP_PLAN

double cScenarioExpImitateMocapStep::CalcRewardStep(const cBipedStepController3D::tStepPlan& step_plan) const
{
	double pose_w = 0.45;
	double vel_w = 0.05;
	double end_eff_w = 0.25;
	double root_w = 0.1;
	double com_w = 0.1;
	double heading_w = 0.1;

	double total_w = pose_w + vel_w + end_eff_w + root_w + com_w + heading_w;
	pose_w /= total_w;
	vel_w /= total_w;
	end_eff_w /= total_w;
	root_w /= total_w;
	com_w /= total_w;
	heading_w /= total_w;
	
	// note that mWideWalkWeight does not get normalized because it is associated with a linear term

	const double pose_scale = 2;
	const double vel_scale = 0.005;
	const double end_eff_scale = 5;
	const double root_scale = 10;
	const double com_scale = 10;

	const double err_scale = 1;

	eStance stance = GetStance();
	int swing_foot_id = GetSwingFootJoint(stance);
	tVector swing_foot_pos = mChar->CalcJointPos(swing_foot_id);
	int stance_foot_id = GetStanceFootJoint(stance);
	tVector stance_foot_pos = mChar->CalcJointPos(stance_foot_id);

	double step_phase = CalcStepPhase();
	const tVector& step_pos = mStepPlan.mStepPos0;
	double root_heading = mStepPlan.mRootHeading;
	bool fallen = HasFallen();
	const auto& joint_mat = mChar->GetJointMat();
	const auto& body_defs = mChar->GetBodyDefs();

	double reward = 0;

	if (!fallen)
	{
		Eigen::VectorXd pose0 = mChar->GetPose();
		Eigen::VectorXd vel0 = mChar->GetVel();
		Eigen::VectorXd pose1 = mKinChar->GetPose();
		Eigen::VectorXd vel1 = mKinChar->GetVel();

		tVector world_root_pos0 = mChar->GetRootPos();
		tMatrix origin_trans = mChar->BuildOriginTrans();

		tVector com_vel0_world = tVector::Zero();
		tVector com_vel1_world = tVector::Zero();
		if (com_w != 0)
		{
			//com_vel0_world = cRBDUtil::CalcCoMVel(joint_mat, body_defs, pose0, vel0);
			com_vel0_world = mChar->CalcCOMVel();
			com_vel1_world = cRBDUtil::CalcCoMVel(joint_mat, body_defs, pose1, vel1);
		}

		cKinTree::NormalizePoseHeading(joint_mat, pose0, vel0);
		cKinTree::NormalizePoseHeading(joint_mat, pose1, vel1);

		int root_id = mChar->GetRootID();
		tVector root_pos0 = cKinTree::GetRootPos(joint_mat, pose0);
		tVector root_pos1 = cKinTree::GetRootPos(joint_mat, pose1);
		tVector root_vel0 = cKinTree::GetRootVel(joint_mat, vel0);
		tVector root_vel1 = cKinTree::GetRootVel(joint_mat, vel1);

		double pose_err = 0;
		double vel_err = 0;
		double end_eff_err = 0;
		double root_err = 0;
		double com_err = 0;
		double heading_err = 0;
		double torque_err = 0;

		int num_end_effs = 0;
		int num_joints = mChar->GetNumJoints();
		assert(num_joints == mJointWeights.size());

		double root_rot_w = mJointWeights[root_id];
#if defined(ENABLE_IND_POSE_ERR)
		pose_err += root_rot_w * exp(-err_scale * pose_scale * cKinTree::CalcRootRotErr(joint_mat, pose0, pose1));
		//pose_err += root_rot_w * (-cKinTree::CalcRootRotErr(joint_mat, pose0, pose1) / (M_PI * M_PI) + 1);
		vel_err += root_rot_w * exp(-err_scale * vel_scale * cKinTree::CalcRootAngVelErr(joint_mat, vel0, vel1));
#else
		pose_err += root_rot_w * cKinTree::CalcRootRotErr(joint_mat, pose0, pose1);
		vel_err += root_rot_w * cKinTree::CalcRootAngVelErr(joint_mat, vel0, vel1);
#endif
		

		for (int j = root_id + 1; j < num_joints; ++j)
		{
			double w = mJointWeights[j];
#if defined(ENABLE_STYLE_MARCH)
			if (step_phase < 0.5)
			{
				int swing_hip = (stance == cBipedStepController3D::eStanceRight) ? 5 : 2;
				int swing_knee = (stance == cBipedStepController3D::eStanceRight) ? 6 : 3;

				w = (j == swing_hip) ? 0.1 * w : w;
			}
#endif

#if defined(ENABLE_STYLE_LEAN) || defined(ENABLE_STYLE_SIDE_LEAN)
			{
				int waist_id = 1;
				w = (j == waist_id) ? 0.1 * w : w;
			}
#endif

#if defined(ENABLE_STYLE_STRAIGHT_KNEE_RIGHT)
			{
				int tar_knee = 3;
				w = (j == tar_knee) ? 0 : w;
			}
#endif

#if defined(ENABLE_STYLE_STRAIGHT_KNEE_LEFT)
			{
				int tar_knee = 6;
				w = (j == tar_knee) ? 0 : w;
			}
#endif
			double curr_pose_err = cKinTree::CalcPoseErr(joint_mat, j, pose0, pose1);
			double curr_vel_err = cKinTree::CalcVelErr(joint_mat, j, vel0, vel1);
			
#if defined(ENABLE_IND_POSE_ERR)
			curr_pose_err = exp(-err_scale * pose_scale * curr_pose_err);
			//curr_pose_err = (-curr_pose_err / (M_PI * M_PI) + 1);
			curr_vel_err = exp(-err_scale * vel_scale * curr_vel_err);
#endif
			pose_err += w * curr_pose_err;
			vel_err += w * curr_vel_err;

			bool is_end_eff = mChar->IsEndEffector(j);
			if (is_end_eff)
			{
				tVector pos0 = mChar->CalcJointPos(j);
				tVector pos1 = cKinTree::CalcJointWorldPos(joint_mat, pose1, j);
				double ground_h0 = mGround->SampleHeight(pos0);
				double ground_h1 = 0;

				pos0[3] = 1;
				pos0 = origin_trans * pos0;
				pos0[3] = 0;

				tVector pos_rel0 = pos0 - root_pos0;
				tVector pos_rel1 = pos1 - root_pos1;
				pos_rel0[1] = pos0[1] - ground_h0;
				pos_rel1[1] = pos1[1] - ground_h1;

				double curr_end_err = (pos_rel1 - pos_rel0).squaredNorm();
#if !defined(DISABLE_FOOTSTEP_REWARD)
				if ((step_phase > 0.5) && (j == swing_foot_id))
				{
					curr_end_err = (pos_rel1[1] - pos_rel0[1]) * (pos_rel1[1] - pos_rel0[1]);
				}
#endif
				
				end_eff_err += curr_end_err;
				++num_end_effs;
			}
			if (mAddTorquePenalty)
			{
				cJoint& joint = this->mChar->GetJoint(j);
				tVector torque(0, 0, 0, 0);
				torque = joint.GetTotalTorque();
				torque_err = torque_err + fabs((((torque[0] + torque[1] + torque[2])/joint.GetTorqueLimit())/num_joints));
				// std::cout << "torque_err2: " << j << " sum: " << torque_err << std::endl;

			}
		}

		double style_pose_err = 0;;
		double style_vel_err = 0;
		CalcStyleRewardErr(style_pose_err, style_vel_err);
		pose_err += style_pose_err;
		vel_err += style_vel_err;

		if (num_end_effs > 0)
		{
			end_eff_err *= 2;
			end_eff_err /= num_end_effs;
		}

		double root_ground_h0 = mGround->SampleHeight(mChar->GetRootPos());
		double root_ground_h1 = 0;
		double h0 = root_pos0[1] - root_ground_h0;
		double h1 = root_pos1[1] - root_ground_h1;
		root_err = (h1 - h0) * (h1 - h0) + 0 * 0.01 * (root_vel1 - root_vel0).squaredNorm();

		com_err = 0.1 * (com_vel1_world - com_vel0_world).squaredNorm();

		double heading0 = mChar->CalcHeading();
		double heading1 = root_heading;
		heading_err = std::abs(heading1 - heading0);
		heading_err = std::min(2 * M_PI - heading_err, heading_err);

#if !defined(DISABLE_FOOTSTEP_REWARD)
		if (step_phase > 0.5)
		{
			const Eigen::VectorXd& kin_pose_world = mKinChar->GetPose();

			double kin_ground_h = 0;
			tVector tar_step_pos = step_pos;
			tVector kin_foot_pos = cKinTree::CalcJointWorldPos(joint_mat, kin_pose_world, swing_foot_id);
			double kin_foot_h = kin_foot_pos[1] - kin_ground_h;
			tar_step_pos[1] += kin_foot_h;

			tVector swing_delta = tar_step_pos - swing_foot_pos;
			end_eff_err += swing_delta.squaredNorm();
		}
#endif

#if defined(ENABLE_IND_POSE_ERR)
		double pose_reward = pose_err;
		double vel_reward = vel_err;
#else
		double pose_reward = exp(-err_scale * pose_scale * pose_err);
		double vel_reward = exp(-err_scale * vel_scale * vel_err);
#endif
		double end_eff_reward = exp(-err_scale * end_eff_scale * end_eff_err);
		double root_reward = exp(-err_scale * root_scale * root_err);
		double com_reward = exp(-err_scale * com_scale * com_err);
		double heading_reward = 0.5 * (std::cos(heading_err) + 1);
		heading_reward = std::pow(heading_reward, 4);

		reward = pose_w * pose_reward + vel_w * vel_reward + end_eff_w * end_eff_reward
			+ root_w * root_reward + com_w * com_reward + heading_w * heading_reward;

		if (mAddTorquePenalty != 0)
		{
			double torque_reward = exp(-err_scale * pose_scale * torque_err) * end_eff_w;
			// std::cout << "torque_reward: " << torque_reward << std::endl;
			reward += torque_reward;
		}

		//reward = style_pose_err; // hack hack hack
	}

	return reward;
}

cScenarioExpImitateMocapStep::cScenarioExpImitateMocapStep()
{
	mStepsPerPeriod = 2;
	mStepLenMin = 0.4;
	mStepLenMax = 0.4;
	mCurrStepLen = 0.5 * (mStepLenMin + mStepLenMax);
	mChangeStepLenProb = 0.1;
	mStepFailDist = 0.85; //Ray
	mTargetResetDist = 1;

	mStepWidthMean = 0.15;
	mMaxHeadingDelta = M_PI / 2;
	mMaxHeadingTurnRate = 0.25;
	mSharpTurnProb = 0.1;
	EnableTargetPos(false);
	EnableRandTargetPos(false);
	mRandomizeInititalRotation = false;
	mMirrorArms = false;

	mInvalidStepHeightThreshold = 0;
}

cScenarioExpImitateMocapStep::~cScenarioExpImitateMocapStep()
{
}

void cScenarioExpImitateMocapStep::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioExpImitateMocapTarget::ParseArgs(parser);
	parser->ParseDouble("step_max_heading_turn_rate", mMaxHeadingTurnRate);
	parser->ParseDouble("step_sharp_turn_prob", mSharpTurnProb);
	parser->ParseString("kin_ctrl_file", mKinCtrlFile);

	parser->ParseDouble("step_length_min", mStepLenMin);
	parser->ParseDouble("step_length_max", mStepLenMax);
	parser->ParseDouble("change_step_len_prob", mChangeStepLenProb);
	parser->ParseDouble("step_mean_width", mStepWidthMean);
	parser->ParseDouble("step_fail_dist", mStepFailDist);

	parser->ParseDouble("invalid_step_height_threshold", mInvalidStepHeightThreshold);
	parser->ParseBool("enable_target_pos", mEnableTargetPos);

	parser->ParseDouble("wide_walk_weight", mWideWalkWeight);
	parser->ParseBool("randomize_initial_rotation", mRandomizeInititalRotation);

	parser->ParseBool("include_arms_in_mirror", mMirrorArms);

	mCurrStepLen = 0.5 * (mStepLenMin + mStepLenMax);
}

void cScenarioExpImitateMocapStep::Init()
{
	cScenarioExpImitateMocapTarget::Init();

	InitEndEffectors();
	SetupKinController();

	ResetKinChar();
	if (EnableSyncChar())
	{
		SyncCharacters();
		InitCharacterPos(mChar);
		ResolveCharGroundIntersect(mChar);
	}

	ResetStepPlan();
}

void cScenarioExpImitateMocapStep::Reset()
{
	cScenarioExpImitateMocapTarget::Reset();
	ResetStepPlan();
}

const cBipedStepController3D::tStepPlan& cScenarioExpImitateMocapStep::GetStepPlan() const
{
	return mStepPlan;
}

bool cScenarioExpImitateMocapStep::EnableTargetPos() const
{
	return mEnableTargetPos;
}

void cScenarioExpImitateMocapStep::EnableTargetPos(bool enable)
{
	mEnableTargetPos = enable;
}

std::string cScenarioExpImitateMocapStep::GetName() const
{
	return "Imitate Step Exploration";
}

void cScenarioExpImitateMocapStep::SetupKinController()
{
	std::shared_ptr<cKinController> kin_ctrl;
	BuildKinController(kin_ctrl);
	
	printf("ScenarioExpImitateMocapStep.cpp SetupKinController\n");
	auto step_ctrl = std::dynamic_pointer_cast<cMocapStepController>(kin_ctrl);
	if (step_ctrl != nullptr)
	{
		// order matters!
		step_ctrl->setRelativeFilePath(this->getRelativeFilePath());
		step_ctrl->SetFootJoints(mEndEffectors[cBipedStepController3D::eStanceRight],
								mEndEffectors[cBipedStepController3D::eStanceLeft]);
		if (mMirrorArms)
		{
			step_ctrl->SetHandJoints(8,	14);

		}
		printf("ScenarioExpImitateMocapStep.cpp mCycleDur: %f\n", mCtrlParams.mCycleDur);
		step_ctrl->SetCyclePeriod(mCtrlParams.mCycleDur);
		step_ctrl->Init(mKinChar.get(), mKinCtrlFile);
	}

	mKinChar->SetController(kin_ctrl);
}

void cScenarioExpImitateMocapStep::BuildKinController(std::shared_ptr<cKinController>& out_ctrl) const
{
#if defined(ENABLE_MOTION_FIELD)
	auto ctrl = std::shared_ptr<cMotionFieldStepController>(new cMotionFieldStepController);
#else
	auto ctrl = std::shared_ptr<cMocapStepController>(new cMocapStepController);
#endif
	//ctrl->EnableAutoStepUpdate(false);
	out_ctrl = ctrl;
}

void cScenarioExpImitateMocapStep::SetupControllerParams(cTerrainRLCtrlFactory::tCtrlParams& out_params) const
{
	cScenarioExpCacla::SetupControllerParams(out_params);
}

void cScenarioExpImitateMocapStep::InitEndEffectors()
{
	mEndEffectors.clear();
	int num_joints = mChar->GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		if (mChar->IsEndEffector(j))
		{
			mEndEffectors.push_back(j);
		}
	}
	assert(mEndEffectors.size() == cBipedStepController3D::eStanceMax);
}

void cScenarioExpImitateMocapStep::ResetParams()
{
	cScenarioExpImitateMocapTarget::ResetParams();
	mRootHeadingDelta = 0;
	mStepFail = false;
}

bool cScenarioExpImitateMocapStep::EnableUpdateStepPlan() const
{
	return true;
}

double cScenarioExpImitateMocapStep::CalcReward() const
{
	return CalcRewardStep(mStepPlan);
}

void cScenarioExpImitateMocapStep::HandleNewActionUpdate()
{
	cScenarioExpImitateMocapTarget::HandleNewActionUpdate();
	HandleNewActionUpdateKinController();
}

void cScenarioExpImitateMocapStep::ResetKinChar()
{
#if defined (SAMPLE_INIT_STATES)
	cScenarioExpImitateMocapTarget::ResetKinChar();
#else
	mKinChar->Reset();
	
	if (EnabledRandStateReset())
	{
		const double phase_offset = 0.1;

		double dur = mCtrlParams.mCycleDur;
		double rand_phase = (mRand.FlipCoin()) ? 0 : 0.5;
		rand_phase += phase_offset;
		double rand_time = rand_phase * dur;

		mKinChar->SetTime(rand_time);
		mKinChar->Pose(rand_time);
	}
#endif
}

void cScenarioExpImitateMocapStep::SyncKinChar()
{
#if defined(ENABLE_KIN_CONTROLLER_TEST)
	return;
#endif
 	const auto& sim_pose = mChar->GetPose();
	const auto& sim_vel = mChar->GetVel();
	mKinChar->SetPose(sim_pose);
	mKinChar->SetVel(sim_vel);
}

void cScenarioExpImitateMocapStep::SwatchEndEffectorPose(const std::vector<int>& end_effector, Eigen::VectorXd& out_pose) const
{
	assert(end_effector.size() == 2);
	int j0 = end_effector[0];
	int j1 = end_effector[1];
	int root_id = mChar->GetRootID();
	const auto& joint_mat = mChar->GetJointMat();
	while (j0 != root_id && j1 != root_id)
	{
		int offset0 = cKinTree::GetParamOffset(joint_mat, j0);
		int offset1 = cKinTree::GetParamOffset(joint_mat, j1);
		int size0 = cKinTree::GetParamSize(joint_mat, j0);
		int size1 = cKinTree::GetParamSize(joint_mat, j1);
		assert(size0 == size1);
		
		for (int i = 0; i < size0; ++i)
		{
			double val0 = out_pose[offset0 + i];
			double val1 = out_pose[offset1 + i];
			out_pose[offset0 + i] = val1;
			out_pose[offset1 + i] = val0;
		}

		j0 = cKinTree::GetParent(joint_mat, j0);
		j1 = cKinTree::GetParent(joint_mat, j1);
	}

	assert(j0 == j1); // both shoud reach root at the same time
}

bool cScenarioExpImitateMocapStep::HasFallen() const
{
	bool fallen = false;
	fallen |= cScenarioExpImitateMocapTarget::HasFallen();

//#if defined(ENABLE_STEP_FAIL)
//	fallen |= mStepFail;
//#endif

	const tVector& step_pos = mStepPlan.mStepPos0;
	tVector root_pos = mChar->GetRootPos();
	root_pos[1] = 0.0; // project onto the ground
	double dist = (step_pos - root_pos).squaredNorm();
	if ( dist >  mTargetResetDist)
	{
		fallen |= true;
	}
//	std::cout << " mTargetResetDist " << mTargetResetDist <<
//			" fallen " << fallen << std::endl;

	return fallen;
}

bool cScenarioExpImitateMocapStep::endOfEpoch() const
{
	return this->HasFallen();
}

void cScenarioExpImitateMocapStep::PostSubstepUpdate(double time_step)
{
	cScenarioExpImitateMocapTarget::PostSubstepUpdate(time_step);
	if (EnableUpdateStepPlan())
	{
		UpdateStepPlan(time_step);
	}
}

void cScenarioExpImitateMocapStep::ResetStepPlan()
{
	SyncKinChar();

	eStance stance = GetStance();
	eStance swing = (stance == cBipedStepController3D::eStanceRight) ? 
					cBipedStepController3D::eStanceLeft : 
					cBipedStepController3D::eStanceRight;

	int stance_foot = GetStanceFootJoint(stance);
	tVector stance_pos = mChar->CalcJointPos(stance_foot);
	tVector root_pos = mChar->GetRootPos();
	tVector out_axis;
	double out_theta;
	/// This should be 0...
	mChar->GetRootRotation(out_axis, out_theta);

	double heading = 0;
	if (mRandomizeInititalRotation)
	{
		tVector rotation_axis;
		heading = mRand.RandDouble(-M_PI, M_PI);
		rotation_axis = tVector(0.0, 1.0, 0.0, 0.0);

		tQuaternion new_rotation = cMathUtil::AxisAngleToQuaternion(rotation_axis, heading);

		// TODO: Need to fix this, character initial link velocities need to be lined up with this rotation
		// character->SetRootRotation(new_rotation);

		// Sets rotation and position
		mChar->SetRootTransform(root_pos, new_rotation);
		out_theta = heading;
		// mChar->GetRootRotation(out_axis, out_theta);
	}
	tVector curr_center = root_pos;
	if (EnableTargetPos())
	{
		tVector delta = mTargetPos - curr_center;
		if (delta.squaredNorm() > 0.01)
		{
			heading = std::atan2(-delta[2], delta[0]) + out_theta;
		}
	}
	else
	{
		stance_pos[2] = (stance == cBipedStepController3D::eStanceRight) ? mStepWidthMean : -mStepWidthMean;
		stance_pos[2] *= 0.5;
	}

	stance_pos[1] = mGround->SampleHeight(stance_pos);
	mCurrStepLen = mRand.RandDouble(mStepLenMin, mStepLenMax);
	tVector pos1 = CalcNextStepPos(swing, heading, stance_pos);
	tVector pos2 = CalcNextStepPos(stance, heading, pos1);

	mStepPlan.mStance = stance;
	mStepPlan.mStepPos0 = pos1;
	mStepPlan.mStepPos1 = pos2;
	mStepPlan.mRootHeading = heading;

	ApplyStepPlan(mStepPlan);
}

void cScenarioExpImitateMocapStep::UpdateStepPlan(double time_step, bool force_update /*= false*/)
{
	//string filename("/home/ruizhang/Desktop/data.txt");
	//fstream file;

	//file.open(filename, std::ios_base::app | std::ios_base::in);

	const double step_err_tol = mStepFailDist;

	eStance stance = GetStance();
	bool update = stance != mStepPlan.mStance;
	update |= force_update;

	if (update)
	{
#if !defined(ENABLE_MOTION_FIELD)
		SyncKinChar();
#endif

		const tVector& step_pos = mStepPlan.mStepPos0;
		tVector root_pos = mChar->GetRootPos();
		root_pos[1] = 0.0; // project onto the ground
		/**
		double dist = (step_pos - root_pos).squaredNorm();
		if ( dist >  mTargetResetDist)
		{
			ResetStepPlan();
			return;
		}
		**/
		tVector old_pos = mStepPlan.mStepPos0;
		tVector next_pos = mStepPlan.mStepPos1;
		/*
		if(file.is_open()){
			file << "\nstep plan before\n";

			file << " old_pos before: " << mStepPlan.mStepPos0<< " next_pos before: " << mStepPlan.mStepPos1 << endl;
		}
		*/
		int stance_foot = GetStanceFootJoint(stance);
		int swing_foot = GetSwingFootJoint(stance);

		tVector stance_pos = mChar->CalcJointPos(stance_foot);
		tVector swing_pos = mChar->CalcJointPos(swing_foot);


		double prev_heading = mStepPlan.mRootHeading;


		tVector pos_delta = old_pos - stance_pos;

		pos_delta[1] = 0;
		double step_err = pos_delta.squaredNorm();
		if (step_err > step_err_tol)
		{
#if defined(ENABLE_STEP_FAIL)
			mStepFail = true;
#else
			next_pos = root_pos + step_err_tol * ((next_pos - root_pos) / (next_pos - root_pos).squaredNorm());
			next_pos[1] = mGround->SampleHeight(next_pos);
#endif // ENABLE_STEP_FAIL
		}

		mStepPlan.mStepPos0 = stance_pos;
		mStepPlan.mStepPos0[1] = mGround->SampleHeight(mStepPlan.mStepPos0);
		mStepPlan.mStepPos1 = next_pos;

		/*
		if(file.is_open()){
			file << "\nstep plan after\n";

			file << " old_pos after: " << mStepPlan.mStepPos0<< " next_pos after: " << next_pos << endl;
		}
		*/
		double new_heading = prev_heading;
		tVector new_pos = tVector::Zero();


		new_heading = CalcNextHeading(prev_heading);
		new_pos = CalcNextStepPos(stance, new_heading, next_pos);

#if defined(ENABLE_ADAPTIVE_STEP_PLAN)
		// hack
		tVector err1 = new_pos - root_pos;
		tVector err2 = next_pos - root_pos;

		err1[1] = 0;
		err2[1] = 0; // hack

		/*
		if(file.is_open()){
			file << "\nFoot Update\n";

			file << " err1 before: " << err1.squaredNorm() << " err2 before: " << err2.squaredNorm() << endl;
		}
		else{
			printf("File not found\n");
		}
		file.close();
		*/

		if (err1.squaredNorm() > 2 * step_err_tol) {
			new_pos = root_pos + 2 * step_err_tol * ((new_pos - root_pos) / (new_pos - root_pos).squaredNorm());
		}
		if (err2.squaredNorm() > step_err_tol) {
			next_pos = root_pos + step_err_tol * ((next_pos - root_pos) / (next_pos - root_pos).squaredNorm());
		}


		new_pos[1] = mGround->SampleHeight(new_pos);
		next_pos[1] = mGround->SampleHeight(next_pos);
#endif

		tVector err1 = new_pos - root_pos;
		tVector err2 = next_pos - root_pos;

		err1[1] = 0;
		err2[1] = 0; // hack

		if (err1.squaredNorm() > 2 * step_err_tol) {
			new_pos = root_pos + 2 * step_err_tol * ((new_pos - root_pos) / (new_pos - root_pos).squaredNorm());
		}
		if (err2.squaredNorm() > step_err_tol) {
			next_pos = root_pos + step_err_tol * ((next_pos - root_pos) / (next_pos - root_pos).squaredNorm());
		}

		new_pos[1] = mGround->SampleHeight(new_pos);
		next_pos[1] = mGround->SampleHeight(next_pos);
		mStepPlan.mStance = stance;
		mStepPlan.mStepPos0 = next_pos;
		mStepPlan.mStepPos1 = new_pos;
		mStepPlan.mRootHeading = new_heading;

		ApplyStepPlan(mStepPlan);
	}
}

double cScenarioExpImitateMocapStep::CalcNextHeading(double heading)
{
	const double sample_dist = 1.5;
	double next_heading = heading;

	if (mWorld->GetSimMode() == cWorld::eSimMode2D)
	{
		return 0.0;
	}
	tVector curr_center = 0.5 * (mStepPlan.mStepPos0 + mStepPlan.mStepPos1);
	if (EnableTargetPos())
	{
		tVector delta = mTargetPos - curr_center;
		if (delta.squaredNorm() > 0.01)
		{
			next_heading = std::atan2(-delta[2], delta[0]);
		}
	}
	else
	{
		// risk of infinite loop?
		while (true)
		{
			bool sharp_turn = mRand.FlipCoin(mSharpTurnProb);
			double delta_heading = 0;
			if (sharp_turn)
			{
				delta_heading = mRand.RandDouble(-M_PI, M_PI);
			}
			else
			{
				delta_heading = mRand.RandDoubleNorm(0, mMaxHeadingTurnRate);
			}

			next_heading = heading + delta_heading;
			tVector sample_pos = curr_center + sample_dist * tVector(std::cos(next_heading), 0, -std::sin(next_heading), 0);
			double sample_h = mGround->SampleHeight(sample_pos);
			
			// hack
			//if (sample_h >= mInvalidStepHeightThreshold)
			{
				break;
			}
		}
	}
	
	return next_heading;
}

tVector cScenarioExpImitateMocapStep::CalcNextStepPos(eStance curr_stance, double heading, const tVector& step_pos0)
{
	if (mRand.FlipCoin(mChangeStepLenProb))
	{
		mCurrStepLen = mRand.RandDouble(mStepLenMin, mStepLenMax);
#if defined(ENABLE_DEBUG_PRINT)
		printf("New Step Len: %.5f\n", mCurrStepLen);
#endif
	}

	tVector new_pos_delta = tVector(mCurrStepLen, 0, mStepWidthMean, 0);
	if (curr_stance == cBipedStepController3D::eStanceLeft)
	{
		new_pos_delta[2] = -new_pos_delta[2];
	}

	tQuaternion quat = cMathUtil::AxisAngleToQuaternion(tVector(0, 1, 0, 0), heading);
	new_pos_delta = cMathUtil::QuatRotVec(quat, new_pos_delta);

	tVector new_pos = step_pos0 + new_pos_delta;
	new_pos[1] = mGround->SampleHeight(new_pos);

	return new_pos;
}

void cScenarioExpImitateMocapStep::HandleNewActionUpdateKinController()
{
#if defined(ENABLE_MOTION_FIELD)
	SyncKinChar();
	
	auto kin_step_ctrl = std::dynamic_pointer_cast<cMotionFieldStepController>(mKinChar->GetController());
	if (kin_step_ctrl != nullptr)
	{
		kin_step_ctrl->ForceStepUpdate();
	}
#endif
}

void cScenarioExpImitateMocapStep::ApplyStepPlan(const cBipedStepController3D::tStepPlan& step)
{
	auto step_ctrl = std::dynamic_pointer_cast<cBipedStepController3D>(mChar->GetController());
	if (step_ctrl != nullptr)
	{
		step_ctrl->SetStepPlan(step);
	}

	auto kin_step_ctrl = std::dynamic_pointer_cast<cMocapStepController>(mKinChar->GetController());
	if (kin_step_ctrl != nullptr)
	{
		kin_step_ctrl->SetStepPlan(step);
		kin_step_ctrl->ForceStepUpdate();
	}
}

cScenarioExpImitateMocapStep::eStance cScenarioExpImitateMocapStep::GetStance() const
{
	double kin_time = mKinChar->GetTime();
	return GetStance(kin_time);
}

cScenarioExpImitateMocapStep::eStance cScenarioExpImitateMocapStep::GetStance(double time) const
{
	eStance stance = cBipedStepController3D::eStanceMax;
	auto kin_step_ctrl = std::dynamic_pointer_cast<cMocapStepController>(mKinChar->GetController());
	if (kin_step_ctrl != nullptr)
	{
		stance = kin_step_ctrl->GetStance(time);
	}
	return stance;
}

int cScenarioExpImitateMocapStep::GetStanceFootJoint(eStance stance) const
{
	int joint_id = (stance == cBipedStepController3D::eStanceRight) ? mEndEffectors[0] : mEndEffectors[1];
	return joint_id;
}

int cScenarioExpImitateMocapStep::GetSwingFootJoint(eStance stance) const
{
	int joint_id = (stance == cBipedStepController3D::eStanceRight) ? mEndEffectors[1] : mEndEffectors[0];
	return joint_id;
}

double cScenarioExpImitateMocapStep::CalcStepPhase() const
{
	double kin_time = mKinChar->GetTime();
	double dur = mKinChar->GetMotionDuration();
	double step_dur = dur / mStepsPerPeriod;
	double step_phase = std::fmod(kin_time / step_dur, 1);
	return step_phase;
}

void cScenarioExpImitateMocapStep::CalcStyleRewardErr(double& out_pose_err, double& out_vel_err) const
{
	// hack hack hack
	// this is all hacks

	out_pose_err = 0;
	out_vel_err = 0;

	eStance stance = GetStance();
	double step_phase = CalcStepPhase();
	const Eigen::VectorXd& pose = mChar->GetPose();
	
#if defined(ENABLE_STYLE_MARCH)
	if (step_phase < 0.5)
	{
		//const double march_w = 0.5;
		const double march_w = 2;

		const tQuaternion target_hip_rot = cMathUtil::AxisAngleToQuaternion(tVector(0, 0, 1, 0), M_PI);

		int swing_hip = (stance == cBipedStepController3D::eStanceRight) ? 5 : 2;
		int swing_knee = (stance == cBipedStepController3D::eStanceRight) ? 6 : 3;

		
		int hip_param_offset = mChar->GetParamOffset(swing_hip);
		int hip_param_size = mChar->GetParamSize(swing_hip);
		int knee_param_offset = mChar->GetParamOffset(swing_knee);
		int knee_param_size = mChar->GetParamSize(swing_knee);

		/*
		tQuaternion hip_rot = cMathUtil::VecToQuat(pose.segment(hip_param_offset, hip_param_size));
		tQuaternion q_diff = cMathUtil::QuatDiff(target_hip_rot, hip_rot);
		double hip_err = cMathUtil::QuatTheta(q_diff);
		//hip_err *= hip_err;
		hip_err = std::abs(hip_err);

		double knee_theta = pose(knee_param_offset);
		double knee_err = -M_PI - knee_theta;
		//knee_err *= knee_err;
		//knee_err = std::abs(knee_err);

		out_pose_err += march_w * hip_err;
		*/
		//out_pose_err += march_w * knee_err;
		
		const tVector target_knee_pos = tVector(0.4, 0.8, 0, 0);
		tMatrix origin_trans = mChar->BuildOriginTrans();
		tVector knee_pos = mChar->CalcJointPos(swing_knee);
		knee_pos[3] = 1;
		knee_pos = origin_trans * knee_pos;

		double x_err = target_knee_pos[0] - knee_pos[0];
		double y_err = target_knee_pos[1] - knee_pos[1];
		//knee_err *= knee_err;
		double knee_err = 0.25 * std::abs(x_err) + std::abs(y_err);
		out_pose_err += march_w * knee_err;
	}
#endif

#if defined(ENABLE_STYLE_LEAN) || defined(ENABLE_STYLE_SIDE_LEAN)
	{
		int waist_id = 1;
		const double lean_w = 0.25;

#if defined(ENABLE_STYLE_SIDE_LEAN)
		tVector axis = tVector(1, 0, 0, 0);
#else
		tVector axis = tVector(0, 0, 1, 0);
#endif
		const tQuaternion target_waist_rot = cMathUtil::AxisAngleToQuaternion(axis, -M_PI / 2);

		int waist_param_offset = mChar->GetParamOffset(waist_id);
		int waist_param_size = mChar->GetParamSize(waist_id);
		tQuaternion waist_rot = cMathUtil::VecToQuat(pose.segment(waist_param_offset, waist_param_size));
		tQuaternion q_diff = cMathUtil::QuatDiff(target_waist_rot, waist_rot);
		double waist_err = cMathUtil::QuatTheta(q_diff);
		waist_err = std::abs(waist_err);

		out_pose_err += lean_w * waist_err;
	}
#endif

#if defined(ENABLE_STYLE_STRAIGHT_KNEE_RIGHT)
	{
		const double knee_w = 0.25;
		int tar_knee = 3;
		double target_theta = 0;

		int knee_param_offset = mChar->GetParamOffset(tar_knee);
		double knee_theta = pose(knee_param_offset);
		double knee_err = target_theta - knee_theta;
		knee_err *= knee_err;

		out_pose_err += knee_w * knee_err;
	}
#endif

#if defined(ENABLE_STYLE_STRAIGHT_KNEE_LEFT)
	{
		const double knee_w = 0.25;
		int tar_knee = 6;
		double target_theta = 0;

		int knee_param_offset = mChar->GetParamOffset(tar_knee);
		double knee_theta = pose(knee_param_offset);
		double knee_err = target_theta - knee_theta;
		knee_err *= knee_err;

		out_pose_err += knee_w * knee_err;
	}
#endif
}