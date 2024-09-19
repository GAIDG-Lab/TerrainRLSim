#include "MultiCharWaypointController.h"
#include "sim/GroundDynamicObstacles3D.h"

//#define ENABLE_HEIGHT_MAP

cMultiCharWaypointController::cMultiCharWaypointController() : cWaypointController()
{
	// mGroundOriginVel.setZero();
}

cMultiCharWaypointController::~cMultiCharWaypointController()
{
}

void cMultiCharWaypointController::BuildTargetPosState(Eigen::VectorXd& out_state) const
{
	//printf("MultiCharWaypointController::BuildTargetPosState\n");
	out_state = Eigen::VectorXd::Zero(GetTargetPosStateSize());

	tVector target_pos = mChar->GetCurrentGroundTarget();
	printf("MultiCharWaypointController::BuildTargetPosState target_pos: %f, %f\n", target_pos[0], target_pos[2]);
	target_pos[3] = 1;
	tMatrix origin_trans = mChar->BuildOriginTrans();
	target_pos = origin_trans * target_pos;
	target_pos[1] = 0;
	target_pos[3] = 0;

	if (FlipStance())
	{
		target_pos[2] = -target_pos[2];
	}

	double tar_theta = std::atan2(-target_pos[2], target_pos[0]);
	double tar_dist = target_pos.norm();

	out_state[0] = tar_theta;
	out_state[1] = tar_dist;
	//printf("MultiCharWaypointController::BuildTargetPosState out_state size: %d\n", out_state.size());
	//printf("MultiCharWaypointController::BuildTargetPosState out_state: %f, %f\n", out_state[0], out_state[1]);
}

void cMultiCharWaypointController::ApplyAction(const tAction& action)
{
	cWaypointController::ApplyAction(action);
	// this->HandleNewActionUpdate();
}

/*
bool cMultiCharWaypointController::NewActionUpdate() const
{
	return false;
}
*/
