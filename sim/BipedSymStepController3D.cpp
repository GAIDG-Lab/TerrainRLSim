#include "sim/BipedSymStepController3D.h"
#include "sim/SimCharacter.h"

const int gNumPhaseBins = 2;
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

cBipedSymStepController3D::cBipedSymStepController3D() : cBipedStepController3D()
{
}

cBipedSymStepController3D::~cBipedSymStepController3D()
{
}

const std::vector<int>& cBipedSymStepController3D::GetFlipJointOrder() const
{
	return mFlipJointOrder;
}

void cBipedSymStepController3D::InitEndEffectors()
{
	cBipedStepController3D::InitEndEffectors();

	int num_joints = mChar->GetNumJoints();
	mFlipJointOrder.resize(num_joints);
	for (int j = 0; j < num_joints; ++j)
	{
		mFlipJointOrder[j] = j;
	}

	assert(mEndEffectors.size() == 2);
	int right_j = mEndEffectors[0];
	int left_j = mEndEffectors[1];
	while (left_j != right_j && left_j != gInvalidIdx && right_j != gInvalidIdx)
	{
		mFlipJointOrder[right_j] = left_j;
		mFlipJointOrder[left_j] = right_j;
		right_j = mChar->GetParentJoint(right_j);
		left_j = mChar->GetParentJoint(left_j);
	}
}

void cBipedSymStepController3D::BuildPoliState(Eigen::VectorXd& out_state) const
{
	//printf("BipedSymStepController3D BuildPoliState\n");
	time_t my_time = time(NULL);
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

	//printf("BipedSymStepController3D::contact_offset: %d\n", contact_offset);
	//printf("BipedSymStepController3D::contact_size: %d\n", contact_size);
	//printf("BipedSymStepController3D::task_offset: %d\n", task_offset);
	//printf("BipedSymStepController3D::task_size: %d\n", task_size);
	//printf("BipedSymStepController3D::external_offset: %d\n", external_offset);
	//printf("BipedSymStepController3D::external_size: %d\n", external_size);

	out_state.segment(contact_offset, contact_size) = contact_state; //208
	out_state.segment(external_offset, external_size) = external_state;
	//std::cout << "BipedSymStepController3D::BuildPoliState state_size: " << external_state.size() << ", state: \n" << external_state << std::endl;
	out_state.segment(task_offset, task_size) = task_state; //208

	//printf("BipedSymStepController3D::BuildPoliState task_size: %d\n", task_size);
	//printf("BipedSymStepController3D::BuildPoliState state size: %d\n", out_state.size());
	mChar->ResetExternalForce();
}

void cBipedSymStepController3D::BuildExternalState(Eigen::VectorXd &out_state) const
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

void cBipedSymStepController3D::BuildPhaseState(Eigen::VectorXd& out_state) const
{
	out_state = Eigen::VectorXd::Zero(GetPhaseStateSize());

	double phase = std::fmod(2 * GetPhase(), 1.0);
#if defined(ENABLE_COS_PHASE)
	int dphase_offset = 2;
	double theta = 2 * M_PI * phase;
	out_state[0] = std::cos(theta);
	out_state[1] = std::sin(theta);
#else
	int dphase_offset = 1;
	out_state[0] = phase;
#endif

#if defined(ENABLE_PHASE_STATE_BINS)
	int bin = static_cast<int>(phase * GetNumPhaseBins());
	out_state[bin + dphase_offset] = 1;
#endif // ENABLE_PHASE_STATE_BINS
}

void cBipedSymStepController3D::BuildTaskState(Eigen::VectorXd& out_state) const
{
	out_state = tSymTaskParams::Zero();
	double tar_heading = mStepPlan.mRootHeading;
	tVector heading_dir = tVector(std::cos(tar_heading), 0, -std::sin(tar_heading), 0);

	tVector root_pos = mChar->GetRootPos();
	double ground_h = mGround->SampleHeight(root_pos);
	tMatrix heading_trans = mChar->BuildOriginTrans();

	if (FlipStance())
	{
		heading_trans.row(2) *= -1; // reflect z
	}

	heading_dir = heading_trans * heading_dir;
	tar_heading = std::atan2(-heading_dir[2], heading_dir[0]);

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

	out_state[eSymTaskParamRootHeading] = tar_heading;
	out_state[eSymTaskParamStepX0] = delta0[0];
	out_state[eSymTaskParamStepY0] = delta0[1];
	out_state[eSymTaskParamStepZ0] = delta0[2];
	out_state[eSymTaskParamStepX1] = delta1[0];
	out_state[eSymTaskParamStepY1] = delta1[1];
	out_state[eSymTaskParamStepZ1] = delta1[2];
}

int cBipedSymStepController3D::GetTaskStateSize() const
{
	return eSymTaskParamMax;
}

int cBipedSymStepController3D::RetargetJointID(int joint_id) const
{
	int new_joint_id = joint_id;
	if (FlipStance())
	{
		new_joint_id = mFlipJointOrder[joint_id];
	}
	return new_joint_id;
}

bool cBipedSymStepController3D::FlipStance() const
{
	eStance stance = GetStance();
	bool flip_stance = stance == eStanceLeft;
	return flip_stance;
}

#if defined(ENABLE_PHASE_STATE_BINS)
int cBipedSymStepController3D::GetNumPhaseBins() const
{
	return gNumPhaseBins;
}
#endif // ENABLE_PHASE_STATE_BINS