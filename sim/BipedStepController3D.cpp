#include "sim/BipedStepController3D.h"
#include "sim/SimCharacter.h"

#define AUG
#define CUM_EXTERNAL
//#define AVG_EXTERNAL
//#define CUM_GRF
//#define AVG_GRF
//#define CUM_NET
//#define AVG_NET

cBipedStepController3D::tStepPlan::tStepPlan()
{
	mStance = eStanceRight;
	mStepPos0.setZero();
	mStepPos1.setZero();
	mRootHeading = 0;
}

cBipedStepController3D::cBipedStepController3D() : cCtPDPhaseController()
{
	mViewDist = 1;
	mViewDistMin = -0.2;

#if defined(ENABLE_HACK_LLC_LERP)
	mHackLerp = 0;
#endif
}

cBipedStepController3D::~cBipedStepController3D()
{
}

void cBipedStepController3D::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cCtPDPhaseController::Init(character, gravity, param_file);
	InitEndEffectors();
	InitPoliState();

#if defined(ENABLE_HACK_LLC_LERP)
	HackLoadNet();
#endif
}

const cBipedStepController3D::tStepPlan& cBipedStepController3D::GetStepPlan() const
{
	return mStepPlan;
}

void cBipedStepController3D::SetStepPlan(const tStepPlan& plan)
{
	mStepPlan = plan;
}

void cBipedStepController3D::InitEndEffectors()
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
}

int cBipedStepController3D::GetNumEndEffectors() const
{
	return static_cast<int>(mEndEffectors.size());
}

cBipedStepController3D::eStance cBipedStepController3D::PredictNextStance(double time_step) const
{
	double phase = mPhase + time_step / GetCycleDur();
	phase = std::fmod(phase, 1.0);
	return GetStance(phase);
}

void cBipedStepController3D::BuildNNInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const
{
	cCtPDPhaseController::BuildNNInputOffsetScaleTypes(out_types);

	int contact_offset = GetContactStateOffset();
	int contact_size = GetContactStateSize();
	for (int i = 0; i < contact_size; ++i)
	{
		out_types[contact_offset + i] = cNeuralNet::eOffsetScaleTypeFixed;
	}

	int task_offset = GetTaskStateOffset();
	int task_size = GetTaskStateSize();
	for (int i = 0; i < task_size; ++i)
	{
		out_types[task_offset + i] = cNeuralNet::eOffsetScaleTypeFixed;
	}
}

cBipedStepController3D::eStance cBipedStepController3D::GetStance() const
{
	return GetStance(mPhase);
}

cBipedStepController3D::eStance cBipedStepController3D::GetStance(double phase) const
{
	eStance stance = (phase < 0.5) ? eStanceRight : eStanceLeft;
	return stance;
}

int cBipedStepController3D::GetPoliStateSize() const
{
	//printf("BipedStepController3D::GetPoliStateSize\n");
	int state_size = cCtPDPhaseController::GetPoliStateSize();
	state_size += GetContactStateSize();
	state_size += GetTaskStateSize();

// #if defined(AUG)
// 	state_size += GetAugStateSize();
// #endif
	return state_size;
}

int cBipedStepController3D::GetContactStateOffset() const
{
	//printf("BipedStepController3D::GetContactStateOffset\n");
	return cCtPDPhaseController::GetPoliStateSize();
}

int cBipedStepController3D::GetContactStateSize() const
{
	//printf("BipedStepController3D::GetContactStateSize\n");

	int aug_size = 0;
#if defined(CUM_EXTERNAL)
	aug_size = mChar->GetNumJoints() * 3;
#elif defined(AVG_EXTERNAL)
	aug_size = mChar->GetNumJoints() * 3;
#elif defined(CUM_GRF)
	aug_size = 6;
#elif defined(AVG_GRF)
	aug_size = 6;
#elif defined(CUM_NET)
	aug_size = 3;
#elif defined(AVG_NET)
	aug_size = 3;
#else

#endif
	return GetNumEndEffectors() + aug_size;
}



int cBipedStepController3D::GetAugStateOffset() const
{
	return cCtPDPhaseController::GetPoliStateSize() + GetContactStateSize() ;
}

int cBipedStepController3D::GetAugStateSize() const
{
	int aug_size = 0;
#if defined(CUM_EXTERNAL)
	aug_size = mChar->GetNumJoints() * 3;
#elif defined(AVG_EXTERNAL)
	aug_size = mChar->GetNumJoints() * 3;
#elif defined(CUM_GRF)
	aug_size = 6;
#elif defined(AVG_GRF)
	aug_size = 6;
#elif defined(CUM_NET)
	aug_size = 3;
#elif defined(AVG_NET)
	aug_size = 3;
#else

#endif
	return aug_size;
}
int cBipedStepController3D::GetTaskStateOffset() const
{
// #if defined(AUG)
// 	return cCtPDPhaseController::GetPoliStateSize() + GetContactStateSize() + GetAugStateSize();
// #else
	return cCtPDPhaseController::GetPoliStateSize() + GetContactStateSize();
// #endif
}

int cBipedStepController3D::GetTaskStateSize() const
{
	return eTaskParamMax;
}
void cBipedStepController3D::BuildPoliState(Eigen::VectorXd& out_state) const
{
	//printf("\nBipedStepController3D::BuildPoliState\n");
	cCtPDPhaseController::BuildPoliState(out_state);

	Eigen::VectorXd contact_state;
	Eigen::VectorXd task_state;
	BuildContactState(contact_state);

// #if defined(AUG)
// 	//printf("BipedStepController3D::BuildPoliState\n");
// 	Eigen::VectorXd aug_state;
// 	BuildAugState(aug_state);
// 	int aug_offset = GetAugStateOffset();
// 	int aug_size = GetAugStateSize();
// 	//printf("BipedStepController3D::BuildPoliState aug_offset: %d\n", aug_offset);
// 	//printf("BipedStepController3D::BuildPoliState aug_size: %d\n", aug_size);
	
// #endif

	BuildTaskState(task_state);

	int contact_offset = GetContactStateOffset();
	int contact_size = GetContactStateSize();
	int task_offset = GetTaskStateOffset();
	int task_size = GetTaskStateSize();

	//printf("BipedStepController3D::BuildPoliState contact_offset: %d\n", contact_offset);
	//printf("BipedStepController3D::BuildPoliState contact_size: %d\n", contact_size);
	
	//printf("BipedStepController3D::BuildPoliState task_offset: %d\n", task_offset);
	//printf("BipedStepController3D::BuildPoliState task_size: %d\n", task_size);

	out_state.segment(contact_offset, contact_size) = contact_state;

// #if defined(AUG)
// 	out_state.segment(aug_offset, aug_size) = aug_state;
// #endif

	out_state.segment(task_offset, task_size) = task_state;


	//printf("BipedStepController3D::BuildPoliState out_state size: %d\n", out_state.size());
	//std::cout << "BipedStepController3D::BuildPoliState out_state:\n" << out_state << std::endl;
	
}

void cBipedStepController3D::BuildAugState(Eigen::VectorXd& out_state) const
{
	//printf("BipedStepController3D::BuildAugState\n");
	
#if defined(CUM_EXTERNAL)
	int num_parts = mChar->GetNumBodyParts();
	out_state.resize(num_parts * 3);

	for (int i = 0; i < num_parts; ++i)
	{
		int part_id = RetargetJointID(i);
		if (mChar->IsValidBodyPart(part_id))
		{
			const auto& curr_part = mChar->GetBodyPart(part_id);
			tVector curr_force = curr_part->mContactForce.mCumForce;

			out_state.segment(i * 3, 3) = curr_force.segment(0, 3);
		}
	}
#elif defined(AVG_EXTERNAL)
	int num_parts = mChar->GetNumBodyParts();
	out_state.resize(num_parts * 3);

	for (int i = 0; i < num_parts; ++i)
	{
		int part_id = RetargetJointID(i);
		if (mChar->IsValidBodyPart(part_id))
		{
			const auto& curr_part = mChar->GetBodyPart(part_id);
			tVector curr_force = curr_part->mContactForce.mCumForce / (double)(curr_part->mContactForce.mNumForce);

			out_state.segment(i * 3, 3) = curr_force.segment(0, 3);
		}
	}

#elif defined(CUM_GRF)
	out_state.resize(6);
	int num_end_effectors = GetNumEndEffectors();
	for (int e = 0; e < num_end_effectors; ++e)
	{
		int joint_id = mEndEffectors[e];
		joint_id = RetargetJointID(joint_id);
		if (mChar->IsValidBodyPart(joint_id))
		{
			const auto& curr_part = mChar->GetBodyPart(joint_id);
			tVector curr_force = curr_part->mContactForce.mCumForce;

			out_state.segment(e * 3, 3) = curr_force.segment(0, 3);
		}
	}
#elif defined(AVG_GRF)
	out_state.resize(6);
	int num_end_effectors = GetNumEndEffectors();
	for (int e = 0; e < num_end_effectors; ++e)
	{
		int joint_id = mEndEffectors[e];
		joint_id = RetargetJointID(joint_id);
		if (mChar->IsValidBodyPart(joint_id))
		{
			const auto& curr_part = mChar->GetBodyPart(joint_id);
			tVector curr_force = curr_part->mContactForce.mCumForce / (double)(curr_part->mContactForce.mNumForce);
			
			out_state.segment(e * 3, 3) = curr_force.segment(0, 3);
		}
	}
#elif defined(CUM_NET)
	out_state.resize(3);
	int num_parts = mChar->GetNumBodyParts();

	tVector net_force = tVector::Zero();
	for (int i = 0; i < num_parts; ++i)
	{
		int part_id = RetargetJointID(i);
		if (mChar->IsValidBodyPart(part_id))
		{
			const auto& curr_part = mChar->GetBodyPart(part_id);
			net_force += curr_part->mContactForce.mCumForce;
		}
	}
	out_state.segment(0, 3) = net_force.segment(0, 3);
#elif defined(AVG_NET)
	out_state.resize(3);
	int num_parts = mChar->GetNumBodyParts();

	tVector net_force = tVector::Zero();
	double num_forces = 0;
	for (int i = 0; i < num_parts; ++i)
	{
		int part_id = RetargetJointID(i);
		if (mChar->IsValidBodyPart(part_id))
		{
			const auto& curr_part = mChar->GetBodyPart(part_id);
			net_force += curr_part->mContactForce.mCumForce;
			num_forces += curr_part->mContactForce.mNumForce;
		}
	}
	net_force = net_force / num_forces;
	out_state.segment(0, 3) = net_force.segment(0, 3);
#else

#endif
	//std::cout << "BipedStepController3D::BuildAugState state: \n" << out_state << std::endl;
	//printf("BipedStepController3D::BuildPoliStateAug size: %d\n", out_state.size());
	mChar->ClearExternalForces();
}

void cBipedStepController3D::BuildContactState(Eigen::VectorXd& out_state) const
{
	int num_end_effectors = GetNumEndEffectors();
	int num_parts = mChar->GetNumBodyParts();

#if defined(CUM_EXTERNAL)
	out_state.resize(num_end_effectors + num_parts * 3);
#elif defined(CUM_GRF)
	out_state.resize(num_end_effectors + 2 * 3);
#elif defined(CUM_NET)
	out_state.resize(num_end_effectors + 3);
#endif
	for (int e = 0; e < num_end_effectors; ++e)
	{
		int joint_id = mEndEffectors[e];
		joint_id = RetargetJointID(joint_id);
		bool in_contact = mChar->IsInContact(joint_id);
		double val = (in_contact) ? 1 : 0;
		out_state[e] = val;
	}
#if defined(CUM_EXTERNAL)
	for (int i = 0; i < num_parts; ++i)
	{
		int part_id = RetargetJointID(i);
		if (mChar->IsValidBodyPart(part_id))
		{
			const auto& curr_part = mChar->GetBodyPart(part_id);
			tVector curr_force = curr_part->mContactForce.mCumForce;

			out_state.segment(num_end_effectors + i * 3, 3) = curr_force.segment(0, 3);
		}
	}
#elif defined(CUM_GRF)

	for (int e = 0; e < num_end_effectors; ++e)
	{
		int joint_id = mEndEffectors[e];
		joint_id = RetargetJointID(joint_id);
		if (mChar->IsValidBodyPart(joint_id))
		{
			const auto& curr_part = mChar->GetBodyPart(joint_id);
			tVector curr_force = curr_part->mContactForce.mCumForce;

			out_state.segment(num_end_effectors + e * 3, 3) = curr_force.segment(0, 3);
		}
	}
#elif defined(CUM_NET)

	tVector net_force = tVector::Zero();
	for (int i = 0; i < num_parts; ++i)
	{
		int part_id = RetargetJointID(i);
		if (mChar->IsValidBodyPart(part_id))
		{
			const auto& curr_part = mChar->GetBodyPart(part_id);
			net_force += curr_part->mContactForce.mCumForce;
		}
	}
	out_state.segment(num_end_effectors, 3) = net_force.segment(0, 3);
#else

#endif
	//std::cout << "BipedStepController3D::BuildPoliState aug_state:\n" << out_state.segment(num_end_effectors, out_state.size()-num_end_effectors) << std::endl;
/**
	for (int i = 0; i < num_parts; ++i)
	{
		int part_id = RetargetJointID(i);
		if (mChar->IsValidBodyPart(part_id))
		{
			const auto& curr_part = mChar->GetBodyPart(part_id);
			tVector curr_force = curr_part->mContactForce.mCumForce;

			out_state.segment(num_end_effectors + i * 3, 3) = curr_force.segment(0, 3);
		}
	}
**/
	//printf("BipedStepController3D::BuildContactState size: %d\n", out_state.size());
}

void cBipedStepController3D::BuildTaskState(Eigen::VectorXd& out_state) const
{
	out_state = tTaskParams::Zero();
	double tar_heading = mStepPlan.mRootHeading;
	tVector heading_dir = tVector(std::cos(tar_heading), 0, -std::sin(tar_heading), 0);

	tVector root_pos = mChar->GetRootPos();
	double ground_h = mGround->SampleHeight(root_pos);
	tMatrix heading_trans = mChar->BuildOriginTrans();
	heading_dir = heading_trans * heading_dir;
	tar_heading = std::atan2(-heading_dir[2], heading_dir[0]);

 	eStance curr_stance = GetStance();
	//assert(curr_stance == mStepPlan.mStance);

	bool right_stance = mStepPlan.mStance == eStanceRight;
	int swing_id = (right_stance) ? mEndEffectors[eStanceLeft] : mEndEffectors[eStanceRight];
	tVector swing_pos = mChar->CalcJointPos(swing_id);
	tVector pos0 = mStepPlan.mStepPos0;
	tVector delta0 = pos0 - swing_pos;
	delta0 = heading_trans * delta0;
	delta0[1] = pos0[1] - ground_h;

	int stance_id = (right_stance) ? mEndEffectors[eStanceRight] : mEndEffectors[eStanceLeft];
	tVector stance_pos = mChar->CalcJointPos(stance_id);
	tVector pos1 = mStepPlan.mStepPos1;
	tVector delta1 = pos1 - stance_pos;
	delta1 = heading_trans * delta1;
	delta1[1] = pos1[1] - ground_h;

	tVector right_delta0;
	tVector left_delta0;
	tVector right_delta1;
	tVector left_delta1;

	if (right_stance)
	{
		right_delta0 = tVector::Zero();
		left_delta0 = delta0;
		right_delta1 = delta1;
		left_delta1 = tVector::Zero();
	}
	else
	{
		right_delta0 = delta0;
		left_delta0 = tVector::Zero();
		right_delta1 = tVector::Zero();
		left_delta1 = delta1;
	}

	out_state[eTaskParamStepRightX0] = right_delta0[0];
	out_state[eTaskParamStepRightY0] = right_delta0[1];
	out_state[eTaskParamStepRightZ0] = right_delta0[2];
	out_state[eTaskParamStepLeftX0] = left_delta0[0];
	out_state[eTaskParamStepLeftY0] = left_delta0[1];
	out_state[eTaskParamStepLeftZ0] = left_delta0[2];
	out_state[eTaskParamStepRightX1] = right_delta1[0];
	out_state[eTaskParamStepRightY1] = right_delta1[1];
	out_state[eTaskParamStepRightZ1] = right_delta1[2];
	out_state[eTaskParamStepLeftX1] = left_delta1[0];
	out_state[eTaskParamStepLeftY1] = left_delta1[1];
	out_state[eTaskParamStepLeftZ1] = left_delta1[2];
	out_state[eTaskParamRootHeading] = tar_heading;

	//printf("BipedStepController3D::BuildTaskState size: %d\n", out_state.size());
}

// hack to get around pdphase controller being 2d, terrain should be compositional not inherited
int cBipedStepController3D::GetNumGroundSamples() const
{
    return cCtController::GetNumGroundSamples();
}

tVector cBipedStepController3D::CalcGroundSamplePos(int s) const
{
    return cCtController::CalcGroundSamplePos(s);
}

void cBipedStepController3D::GetViewBound(tVector& out_min, tVector& out_max) const
{
    cCtController::GetViewBound(out_min, out_max);
    /*  
    double view_dist = GetViewDist();
	tVector root_pos = mChar->GetRootPos();
	root_pos[1] = 0;
	out_min = root_pos + tVector(-2, 0, -2, 0);
	out_max = root_pos + tVector(view_dist, 0, 2, 0);
    */
}

// hack hack hack
void cBipedStepController3D::EvalNet(const Eigen::VectorXd& x, Eigen::VectorXd& out_y) const
{
	cCtPDPhaseController::EvalNet(x, out_y);
#if defined(ENABLE_HACK_LLC_LERP)
	Eigen::VectorXd hack_y;
	mHackNet->Eval(x, hack_y);

	double t = mHackLerp;
	out_y = (1 - t) * out_y + t * hack_y;
#else
	//out_y[0] += -M_PI * 0.2;
#endif
}

#if defined(ENABLE_HACK_LLC_LERP)
void cBipedStepController3D::SetHackLerp(double lerp)
{
	mHackLerp = lerp;
}

double cBipedStepController3D::GetHackLerp() const
{
	return mHackLerp;
}

void cBipedStepController3D::HackLoadNet()
{
	mHackNet = std::unique_ptr<cNeuralNet>(new cNeuralNet());
	mHackNet->LoadNet("data/policies/biped3d/nets/biped3d_step_dphase1_actor_net.prototxt");
	mHackNet->LoadModel("data/policies/biped3d/models/biped3d_step_dphase_march_model.h5");
}
#endif
