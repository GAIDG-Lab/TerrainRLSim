#include "sim/BipedStepController3D.h"
#include "sim/SimCharacter.h"

const int jh = 0;
const int avg_EFs = 1;
const int ins_EFs = 2;
const int cum_EFs = 3;
const int avg_NF = 4;
const int ins_NF = 5;
const int cum_NF = 6;
const int avg_GRFs = 7;
const int ins_GRFs = 8;
const int cum_GRFs = 9;

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

void cBipedStepController3D::Init(cSimCharacter *character, const tVector &gravity, const std::string &param_file)
{
	cCtPDPhaseController::Init(character, gravity, param_file);
	InitEndEffectors();
	InitPoliState();

#if defined(ENABLE_HACK_LLC_LERP)
	HackLoadNet();
#endif
}

void cBipedStepController3D::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file, const std::string a_type)
{
	printf("BipedStepController3D::Init2\n");
	mAugType = -1;
	SetAugType(a_type);
	cCtPDPhaseController::Init(character, gravity, param_file);
	InitEndEffectors();
	InitPoliState();

#if defined(ENABLE_HACK_LLC_LERP)
	HackLoadNet();
#endif
}

void cBipedStepController3D::SetAugType(std::string atype){
	if(atype != "")
	{
		mAugType = std::stoi(atype);
	}
}

const cBipedStepController3D::tStepPlan &cBipedStepController3D::GetStepPlan() const
{
	return mStepPlan;
}

void cBipedStepController3D::SetStepPlan(const tStepPlan &plan)
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

void cBipedStepController3D::BuildNNInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType> &out_types) const
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
	int state_size = cCtPDPhaseController::GetPoliStateSize();
	state_size += GetContactStateSize();
	state_size += GetExternalStateSize();
	state_size += GetTaskStateSize();
	return state_size;
}

int cBipedStepController3D::GetExternalStateSize() const
{
	int state_size = 0;
	int num_joints = mChar->GetNumJoints();
	//printf("cBipedStepController3D::GetExternalStateSize num_joints:%d\n", num_joints);
	switch (mAugType)
	{
	case jh:
		state_size = num_joints;
		break;
	case avg_EFs:
		state_size = num_joints * 3;
		break;
	case ins_EFs:
		state_size = num_joints * 3;
		break;
	case cum_EFs:
		state_size = num_joints * 3;
		break;
	case avg_NF:
		state_size = 3;
		break;
	case ins_NF:
		state_size = 3;
		break;
	case cum_NF:
		state_size = 3;
		break;
	case avg_GRFs:
		state_size = 3 * 2;
		break;
	case ins_GRFs:
		state_size = 3 * 2;
		break;
	case cum_GRFs:
		state_size = 3 * 2;
		break;
	default:
		state_size = 0;
		break;
	}
	// printf("BipedStepController3D::GetExternalStateSize state_size: %d\n", state_size);
	return state_size;
}

int cBipedStepController3D::GetExternalStateOffset() const
{
	return cCtPDPhaseController::GetPoliStateSize() + GetContactStateSize();
}

int cBipedStepController3D::GetContactStateOffset() const
{
	return cCtPDPhaseController::GetPoliStateSize();
}

int cBipedStepController3D::GetContactStateSize() const
{
	return GetNumEndEffectors();
}

int cBipedStepController3D::GetTaskStateOffset() const
{
	return cCtPDPhaseController::GetPoliStateSize() + GetContactStateSize() + GetExternalStateSize();
}

int cBipedStepController3D::GetTaskStateSize() const
{
	return eTaskParamMax;
}

void cBipedStepController3D::BuildExternalState(Eigen::VectorXd &out_state) const
{
	int rot_dim = 3;
	out_state = Eigen::VectorXd::Zero(GetExternalStateSize());

	switch (mAugType)
	{
		case jh:
		{
			int idx = 0;
			int num_parts = mChar->GetNumBodyParts();

			for (int i = 0; i < num_parts; ++i)
			{
				int part_id = RetargetJointID(i);
				if (mChar->IsValidBodyPart(part_id))
				{
					const auto &curr_part = mChar->GetBodyPart(part_id);

					tVector curr_pos = curr_part->GetPos();
					out_state[idx] = curr_pos[1];
					idx += 1;
				}
			}
			break;
		}
		case avg_EFs:
		{
			int num_parts = mChar->GetNumBodyParts();

			int idx = 0;
			tVector net_force = tVector::Zero();
			double num_forces = 0;
			for (int i = 0; i < num_parts; ++i)
			{
				if (mChar->IsValidBodyPart(i))
				{
					const auto &curr_part = mChar->GetBodyPart(i);
					net_force += curr_part->mExternalForce.external_force_agent;
					net_force += curr_part->mExternalForce.external_force_obstacle;
					if (mChar->IsEndEffector(i))
					{
						net_force += curr_part->mExternalForce.external_force_ground;
						num_forces += curr_part->mExternalForce.num_force_ground;
					}

					num_forces += curr_part->mExternalForce.num_force_agent;
					num_forces += curr_part->mExternalForce.num_force_obstacle;

					if (num_forces > 0)
					{
						net_force /= num_forces;
					}
					out_state.segment(idx, rot_dim) = net_force.segment(0, rot_dim);

					idx += rot_dim;
				}
			}
			break;
		}
		case ins_EFs:
		{
			int num_parts = mChar->GetNumBodyParts();

			int idx = 0;
			tVector net_force = tVector::Zero();
			for (int i = 0; i < num_parts; ++i)
			{
				if (mChar->IsValidBodyPart(i))
				{
					const auto &curr_part = mChar->GetBodyPart(i);
					net_force += curr_part->mExternalForce.external_force_agent_ins;
					net_force += curr_part->mExternalForce.external_force_obstacle_ins;
					if (mChar->IsEndEffector(i))
					{
						net_force += curr_part->mExternalForce.external_force_ground_ins;
					}

					out_state.segment(idx, rot_dim) = net_force.segment(0, rot_dim);

					idx += rot_dim;
				}
			}
			break;
		}
		case cum_EFs:
		{
			int num_parts = mChar->GetNumBodyParts();

			int idx = 0;
			tVector net_force = tVector::Zero();
			for (int i = 0; i < num_parts; ++i)
			{
				if (mChar->IsValidBodyPart(i))
				{
					const auto &curr_part = mChar->GetBodyPart(i);
					net_force += curr_part->mExternalForce.external_force_agent;
					net_force += curr_part->mExternalForce.external_force_obstacle;
					if (mChar->IsEndEffector(i))
					{
						net_force += curr_part->mExternalForce.external_force_ground;
					}

					out_state.segment(idx, rot_dim) = net_force.segment(0, rot_dim);

					idx += rot_dim;
				}
			}
			break;
		}
		case avg_NF:
		{
			int num_parts = mChar->GetNumBodyParts();
			tVector avg_net_force = tVector::Zero();
			double num_forces = 0;
			// tVector net_gravity = tVector::Zero();
			for (int i = 0; i < num_parts; ++i)
			{
				if (mChar->IsValidBodyPart(i))
				{
					const auto &curr_part = mChar->GetBodyPart(i);
					// net_gravity += curr_part->GetGravity();
					avg_net_force += curr_part->mExternalForce.external_force_agent;
					avg_net_force += curr_part->mExternalForce.external_force_ground;
					avg_net_force += curr_part->mExternalForce.external_force_obstacle;

					num_forces += curr_part->mExternalForce.num_force_agent;
					num_forces += curr_part->mExternalForce.num_force_ground;
					num_forces += curr_part->mExternalForce.num_force_obstacle;
				}
			}
			if (num_forces > 0)
			{
				avg_net_force /= num_forces;
			}
			out_state.segment(0, 3) = avg_net_force.segment(0, 3);

			break;
		}
		case ins_NF:
		{
			int num_parts = mChar->GetNumBodyParts();
			tVector ins_net_force = tVector::Zero();
			// tVector net_gravity = tVector::Zero();
			for (int i = 0; i < num_parts; ++i)
			{
				if (mChar->IsValidBodyPart(i))
				{
					const auto &curr_part = mChar->GetBodyPart(i);
					// net_gravity += curr_part->GetGravity();
					ins_net_force += curr_part->mExternalForce.external_force_agent_ins;
					ins_net_force += curr_part->mExternalForce.external_force_ground_ins;
					ins_net_force += curr_part->mExternalForce.external_force_obstacle_ins;
				}
			}
			out_state.segment(0, 3) = ins_net_force.segment(0, 3);

			break;
		}
		case cum_NF:
		{
			int num_parts = mChar->GetNumBodyParts();
			tVector cum_net_force = tVector::Zero();
			// tVector net_gravity = tVector::Zero();
			for (int i = 0; i < num_parts; ++i)
			{
				if (mChar->IsValidBodyPart(i))
				{
					const auto &curr_part = mChar->GetBodyPart(i);
					// net_gravity += curr_part->GetGravity();
					cum_net_force += curr_part->mExternalForce.external_force_agent;
					cum_net_force += curr_part->mExternalForce.external_force_ground;
					cum_net_force += curr_part->mExternalForce.external_force_obstacle;
				}
			}
			out_state.segment(0, 3) = cum_net_force.segment(0, 3);

			break;
		}
		case avg_GRFs:
		{
			int num_parts = mChar->GetNumBodyParts();

			int idx = 0;
			for (int i = 0; i < num_parts; ++i)
			{
				if (mChar->IsValidBodyPart(i) && mChar->IsEndEffector(i))
				{
					const auto &curr_part = mChar->GetBodyPart(i);

					tVector GRFs = curr_part->mExternalForce.external_force_ground;
					double num_forces = curr_part->mExternalForce.num_force_ground;
					if (num_forces > 0)
					{
						GRFs /= num_forces;
					}
					out_state.segment(idx, rot_dim) = GRFs.segment(0, rot_dim);
					idx += rot_dim;
				}
			}
			break;
		}
		case ins_GRFs:
		{
			int num_parts = mChar->GetNumBodyParts();
			int idx = 0;
			for (int i = 0; i < num_parts; ++i)
			{
				if (mChar->IsValidBodyPart(i) && mChar->IsEndEffector(i))
				{
					const auto &curr_part = mChar->GetBodyPart(i);

					tVector GRFs = curr_part->mExternalForce.external_force_ground_ins;
					out_state.segment(idx, rot_dim) = GRFs.segment(0, rot_dim);
					idx += rot_dim;
				}
			}
			break;
		}
		case cum_GRFs:
		{
			int num_parts = mChar->GetNumBodyParts();
			int idx = 0;
			for (int i = 0; i < num_parts; ++i)
			{
				if (mChar->IsValidBodyPart(i) && mChar->IsEndEffector(i))
				{
					const auto &curr_part = mChar->GetBodyPart(i);

					tVector GRFs = curr_part->mExternalForce.external_force_ground;
					out_state.segment(idx, rot_dim) = GRFs.segment(0, rot_dim);
					idx += rot_dim;
				}
			}
			break;
		}
		default:
			break;
	}
}

void cBipedStepController3D::BuildPoliState(Eigen::VectorXd& out_state) const
{
	cCtPDPhaseController::BuildPoliState(out_state);

	Eigen::VectorXd contact_state;
	Eigen::VectorXd external_state;
	Eigen::VectorXd task_state;
	BuildContactState(contact_state);
	BuildExternalState(external_state);
	BuildTaskState(task_state);

	int contact_offset = GetContactStateOffset(); //199
	int contact_size = GetContactStateSize(); //2
	int task_offset = GetTaskStateOffset(); //201
	int task_size = GetTaskStateSize();//7

	int external_offset = GetExternalStateOffset();
	int external_size = GetExternalStateSize(); //15

	out_state.segment(contact_offset, contact_size) = contact_state; //208
	out_state.segment(external_offset, external_size) = external_state;
	out_state.segment(task_offset, task_size) = task_state; //208

	mChar->ResetExternalForce();
}

void cBipedStepController3D::BuildContactState(Eigen::VectorXd &out_state) const
{
	int num_end_effectors = GetNumEndEffectors();
	out_state.resize(num_end_effectors);
	for (int e = 0; e < num_end_effectors; ++e)
	{
		int joint_id = mEndEffectors[e];
		joint_id = RetargetJointID(joint_id);
		bool in_contact = mChar->IsInContact(joint_id);
		double val = (in_contact) ? 1 : 0;
		out_state[e] = val;
	}
}

void cBipedStepController3D::BuildTaskState(Eigen::VectorXd &out_state) const
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
	// assert(curr_stance == mStepPlan.mStance);

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
	printf("cBipedStepController3D::BuildTaskState is done!\n");
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

void cBipedStepController3D::GetViewBound(tVector &out_min, tVector &out_max) const
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
void cBipedStepController3D::EvalNet(const Eigen::VectorXd &x, Eigen::VectorXd &out_y) const
{
	cCtPDPhaseController::EvalNet(x, out_y);
#if defined(ENABLE_HACK_LLC_LERP)
	Eigen::VectorXd hack_y;
	mHackNet->Eval(x, hack_y);

	double t = mHackLerp;
	out_y = (1 - t) * out_y + t * hack_y;
#else
	// out_y[0] += -M_PI * 0.2;
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
