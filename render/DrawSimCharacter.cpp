#include "DrawSimCharacter.h"
#include "DrawCharacter.h"
#include "sim/DogController.h"
#include "sim/DogControllerCacla.h"
#include "sim/DogControllerMACE.h"
#include "sim/GoatControllerMACE.h"
#include "sim/RaptorControllerCacla.h"
#include "sim/RaptorControllerMACE.h"
#include "sim/RaptorControllerMACE.h"
#include "sim/MonopedHopperControllerCacla.h"
#include "sim/MonopedHopperControllerMACE.h"
#include "sim/MonopedHopperControllerMACE.h"
#include "sim/WaypointController.h"
#include "sim/WaypointVelController.h"
#include "sim/SimBox.h"
#include "render/DrawObj.h"
#include "render/DrawPerturb.h"
#include "render/GraphUtil.h"

void cDrawSimCharacter::Draw(const cSimCharacter& character, const tVector& fill_tint, const tVector& line_col, bool enable_draw_shape)
{

	{
		tVector line_col2 = line_col;
		line_col2[0]=0.0;
		line_col2[1]=0.0;
		line_col2[2]=0.0;


		DrawSimBody(character, fill_tint, line_col2);
	}

	bool has_draw_shapes = character.HasDrawShapes();
	// if (has_draw_shapes )
	// {
	// 			tVector line_col3 = line_col;
	// 	line_col3[0]=0.0;
	// 	line_col3[1]=0.0;
	// 	line_col3[2]=0.0;
	// 	DrawShapes(character, fill_tint, line_col3);
	// }
//	else

}

void cDrawSimCharacter::DrawCoM(const cSimCharacter& character, double marker_size, double vel_scale, 
								const tVector& col, const tVector& offset)
{
	const double arrow_size = marker_size * 0.65;
	tVector com = character.CalcCOM();
	tVector com_vel = character.CalcCOMVel();
	
	cDrawUtil::SetLineWidth(4);
	cDrawUtil::SetColor(tVector(col[0], col[1], col[2], col[3]));
	cDrawUtil::DrawCross(com + offset, marker_size);
	cDrawUtil::DrawArrow2D(com + offset, com + offset + com_vel * vel_scale, arrow_size);
}

void cDrawSimCharacter::DrawTorque(const cSimCharacter& character, const tVector& offset)
{
	int num_joints = character.GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		const cJoint& joint = character.GetJoint(j);
		if (joint.IsValid())
		{
			tVector torque = joint.GetTotalTorque();
			tVector pos = joint.GetPos();
			cDrawPerturb::DrawTorque2D(pos + offset, torque);
		}
	}
}

void cDrawSimCharacter::DrawLinearVelocity(const cSimCharacter& character, const tVector& offset)
{
	int num_parts = character.GetNumBodyParts();
	const double marker_size = 0.10;
	const double arrow_size = marker_size * 0.65;

	for (int j = 0; j < num_parts; ++j)
	{
		const auto& curr_part = character.GetBodyPart(j);
		tVector pos = curr_part->GetPos();
		tVector vel = curr_part->GetLinearVelocity();
		cDrawUtil::SetColor(tVector(0, 0.75, 0, 0.5));
		cDrawUtil::DrawArrow2D(pos, pos + vel, arrow_size);
	}
}
void cDrawSimCharacter::DrawTotalForce(const cSimCharacter& character, const tVector& offset)
{
	int num_parts = character.GetNumBodyParts();
	const double marker_size = 0.10;
	const double arrow_size = marker_size * 0.65;

	for (int j = 0; j < num_parts; ++j)
	{
		const auto& curr_part = character.GetBodyPart(j);
		tVector pos = curr_part->GetPos();
		tVector ground = tVector(pos[0], 0, pos[2], 0);
		cDrawUtil::SetColor(tVector(1, 0, 0, 0.5));
		cDrawUtil::DrawArrow2D(pos, ground, arrow_size);
	}
}
void cDrawSimCharacter::DrawAngularVelocity(const cSimCharacter& character, const tVector& offset)
{
	int num_parts = character.GetNumBodyParts();
	const double marker_size = 0.10;
	const double arrow_size = marker_size * 0.65;

	for (int j = 0; j < num_parts; ++j)
	{
		const auto& curr_part = character.GetBodyPart(j);
		tVector pos = curr_part->GetPos();
		tVector vel = curr_part->GetAngularVelocity() * 0.1;
		cDrawUtil::SetColor(tVector(1, 0, 0, 0.5));
		cDrawUtil::DrawArrow2D(pos, pos + vel, arrow_size);
	}
}
void cDrawSimCharacter::DrawVelocity(const cSimCharacter& character, const tVector& offset)
{
	int num_parts = character.GetNumBodyParts();
	const double marker_size = 0.10;
	const double arrow_size = marker_size * 0.65;

	for (int j = 0; j < num_parts; ++j)
	{
		const auto& curr_part = character.GetBodyPart(j);
		tVector pos = curr_part->GetPos();
		tVector linear_vel = curr_part->GetLinearVelocity();
		tVector angular_vel = curr_part->GetAngularVelocity();
		tVector vel = (linear_vel + angular_vel) * 0.1;

		cDrawUtil::SetColor(tVector(0.5, 0.75, 0, 0.5));
		cDrawUtil::DrawArrow2D(pos, pos + vel, arrow_size);
	}
}
void cDrawSimCharacter::DrawmForces(const cSimCharacter& character, const tVector& offset)
{
	int num_parts = character.GetNumBodyParts();
	const double marker_size = 0.10;
	const double arrow_size = marker_size * 0.65;

	for (int j = 0; j < num_parts; ++j)
	{
		const auto& curr_part = character.GetBodyPart(j);
		tVector pos = curr_part->GetPos();
		tVector force = curr_part->GetmTorque();

		printf("DrawmForces j: %d, m_force: %f %f %f\n", j, force[0], force[1], force[2]);
		cDrawUtil::SetColor(tVector(0.5, 0.75, 0, 0.5));
		cDrawUtil::DrawArrow2D(pos, pos + force, arrow_size);
	}
}
//Draw forces from acceleration from velocityinlocalpoint

void cDrawSimCharacter::DrawImpulse(const cSimCharacter& character, const tVector& offset)
{
	int num_parts = character.GetNumBodyParts();
	const double marker_size = 0.10;
	const double arrow_size = marker_size * 0.65;

	for (int j = 0; j < num_parts; ++j)
	{
		const auto& curr_part = character.GetBodyPart(j);
		tVector pos = curr_part->GetPos();
		tVector impulse = curr_part->GetContactImpulse();
		cDrawUtil::DrawArrow2D(pos, pos + impulse, arrow_size);
	}
}
void cDrawSimCharacter::DrawNetLinearVelocity(const cSimCharacter& character, const tVector& offset)
{
	int num_parts = character.GetNumBodyParts();
	const double marker_size = 0.10;
	const double arrow_size = marker_size * 0.65;

	tVector net_vel = tVector(0,0,0,0);
	tVector net_pos = tVector(0,0,0,0);

	for (int j = 0; j < num_parts; ++j)
	{
		const auto& curr_part = character.GetBodyPart(j);
		tVector pos = curr_part->GetPos();
		tVector vel = curr_part->GetLinearVelocity();

		net_pos += pos;
		net_vel += vel;
	}
	net_pos = net_pos / num_parts;
	net_vel = net_vel / num_parts;

	tVector root_pos = character.GetRootPos();
	cDrawUtil::SetColor(tVector(0, 0.75, 0, 0.5));
	cDrawUtil::DrawArrow2D(net_pos, net_pos + net_vel, arrow_size);
}
void cDrawSimCharacter::DrawNetForce(const cSimCharacter& character, const tVector& offset)
{
	int num_parts = character.GetNumBodyParts();
	const double marker_size = 0.50;
	const double arrow_size = marker_size * 0.65;

	tVector net_gravity = tVector(0, 0, 0, 0);
	for (int j = 0; j < num_parts; ++j)
	{
		const auto& curr_part = character.GetBodyPart(j);
		net_gravity += curr_part->GetGravity();
	}
	printf("DrawSimCharacter.cpp net_gravity: %f %f %f\n",  net_gravity[0], net_gravity[1], net_gravity[2]);
	tVector net_force = character.GetInsNetExternalForce();
	net_force += net_gravity;
	tVector net_force_pos = character.GetNetExternalForcePos();
	

	printf("DrawSimCharacter.cpp net_force: %f %f %f\n",  net_force[0], net_force[1], net_force[2]);
	cDrawUtil::SetColor(tVector(1, 0, 0, 0.5));
	cDrawUtil::DrawArrow2D(net_force_pos, net_force_pos+net_force, arrow_size);

}

void cDrawSimCharacter::DrawGRFs(const cSimCharacter& character, const cGround& ground, double marker_size, double vel_scale,
										const tVector& pos_col, const tVector& vel_col, const tVector& offset)
{
	marker_size = 0.5;
	const double arrow_size = marker_size * 0.65;
	tVector root_pos = character.GetRootPos();
	double ground_h = ground.SampleHeight(root_pos);
	tVector ground_pos = root_pos;
	ground_pos[1] = ground_h;

	//cDrawUtil::SetColor(tVector(pos_col[0], pos_col[1], pos_col[2], pos_col[3]));
	//cDrawUtil::DrawArrow2D(ground_pos + offset, root_pos + offset, arrow_size);

	tVector gravities = tVector(0, 0, 0, 0);
	for (int i = 0; i < character.GetNumBodyParts(); ++i)
	{
		if (character.IsValidBodyPart(i))
		{
			
			const auto& curr_part = character.GetBodyPart(i);
			//tVector gravity = curr_part->GetGravity();
			//gravities += gravity;
			//if ( (i == 5 || i == 11))
			if (curr_part->IsInContact() && character.IsEndEffector(i))
			{
				tVector pos = curr_part->GetPos();
				tVector curr_impulse = curr_part->mExternalForce.external_force_ground_ins;
				//curr_impulse += gravity;
				/**
				int num_contacts = curr_part->GetNumContactGround();
				tVector curr_impulse = tVector(0, 0, 0, 0);

				if(num_contacts != 0){
					curr_impulse = curr_part->GetContactImpulseGround() / num_contacts;;
				}
				
				curr_part->ResetContactImpulse();
				**/

				cDrawUtil::SetColor(tVector(1, 0, 0, 0.5));
				cDrawUtil::DrawArrow2D(pos, pos + curr_impulse, arrow_size);
				printf("DrawSimCharacter::DrawGRFs joint_id: %d, GRF: %f, %f, %f\n", i, curr_impulse[0], curr_impulse[1], curr_impulse[2]);
			}
		}
	}
	//printf("Gravity of the agent is: %f\n", gravities[1]);
	//cDrawUtil::SetColor(tVector(1, 0, 0, 0.5));
	//cDrawUtil::DrawArrow2D(root_pos, root_pos + net, arrow_size);
}
void cDrawSimCharacter::DrawForces(const cSimCharacter& character, const tVector& offset)
{
	int num_parts = character.GetNumBodyParts();
	const double marker_size = 0.50;
	const double arrow_size = marker_size * 0.65;


	for (int i = 0; i < character.GetNumBodyParts(); ++i)
	{
		if (character.IsValidBodyPart(i))
		{
			const auto& curr_part = character.GetBodyPart(i);
			if (!character.IsEndEffector(i))
			// if (curr_part->IsInContact() && character.IsEndEffector(i))
			{
				tVector pos = curr_part->GetPos();
				tVector external_force = tVector::Zero();
				external_force += curr_part->mExternalForce.external_force_ground_ins;
				external_force += curr_part->mExternalForce.external_force_agent_ins;
				external_force += curr_part->mExternalForce.external_force_obstacle_ins;
				
				if(external_force.norm() != 0)
				{
					cDrawUtil::SetColor(tVector(0.5, 0.75, 0, 0.5));
					cDrawUtil::DrawArrow2D(pos, pos + external_force, arrow_size);
					printf("DrawSimCharacter::DrawForces force: %f, %f, %f\n", external_force[0], external_force[1], external_force[2]);
				}
				
			}
		}
	}
}

void cDrawSimCharacter::DrawCtrlInfo(const cCharController* ctrl, const cGround* ground, const tVector& offset, bool draw_3d)
{
	if (ctrl != nullptr)
	{
		DrawCtrlInfoGroundSamples(ctrl, offset, draw_3d);
	}
}

void cDrawSimCharacter::DrawPoliInfo(const cCharController* ctrl, const cCamera& cam)
{
#if defined(ENABLE_DEBUG_VISUALIZATION)
	const cTerrainRLCharController* trl_ctrl = dynamic_cast<const cTerrainRLCharController*>(ctrl);
	if (trl_ctrl != nullptr)
	{
		const cCircularBuffer<double>& val_log = trl_ctrl->GetPoliValLog();
		double aspect = cam.GetAspectRatio();
		DrawInfoValLog(val_log, aspect);
	}
#endif
}

void cDrawSimCharacter::DrawCharFeatures(const cSimCharacter& character, const cGround& ground, double marker_size, double vel_scale,
										const tVector& pos_col, const tVector& vel_col, const tVector& offset)
{
	const double arrow_size = marker_size * 0.65;
	tVector root_pos = character.GetRootPos();
	double ground_h = ground.SampleHeight(root_pos);
	tVector ground_pos = root_pos;
	ground_pos[1] = ground_h;

	cDrawUtil::SetColor(tVector(pos_col[0], pos_col[1], pos_col[2], pos_col[3]));
	cDrawUtil::DrawArrow2D(ground_pos + offset, root_pos + offset, arrow_size);

	for (int i = 0; i < character.GetNumBodyParts(); ++i)
	{
		if (character.IsValidBodyPart(i))
		{
			const auto& curr_part = character.GetBodyPart(i);
			tVector pos = curr_part->GetPos();
			tVector vel = curr_part->GetLinearVelocity();

			cDrawUtil::SetColor(tVector(pos_col[0], pos_col[1], pos_col[2], pos_col[3]));
			cDrawUtil::DrawArrow2D(root_pos + offset, pos + offset, arrow_size);
			cDrawUtil::SetColor(tVector(vel_col[0], vel_col[1], vel_col[2], vel_col[3]));
			cDrawUtil::DrawArrow2D(pos + offset, pos + vel * vel_scale + offset, arrow_size);
		}
	}
}

void cDrawSimCharacter::DrawGroundReactionForces(const cSimCharacter& character, const cGround& ground, double marker_size, double vel_scale,
										const tVector& pos_col, const tVector& vel_col, const tVector& offset)
{
	const double arrow_size = marker_size * 0.65;
	tVector root_pos = character.GetRootPos();
	double ground_h = ground.SampleHeight(root_pos);
	tVector ground_pos = root_pos;
	ground_pos[1] = ground_h;

	cDrawUtil::SetColor(tVector(pos_col[0], pos_col[1], pos_col[2], pos_col[3]));
	cDrawUtil::DrawArrow2D(ground_pos + offset, root_pos + offset, arrow_size);

	for (int i = 0; i < character.GetNumBodyParts(); ++i)
	{
		if (character.IsValidBodyPart(i))
		{
			const auto& curr_part = character.GetBodyPart(i);
			if (curr_part->IsInContact())
			// if (curr_part->IsInContact() && character.IsEndEffector(i))
			{
				tVector curr_impulse = curr_part->GetContactImpulse() * 10;
				// std::cout << "current link: " << i << " contact impulse: " << curr_impulse << std::endl;

				tVector pos = curr_part->GetPos();
				// tVector vel = curr_part->GetLinearVelocity();

				// cDrawUtil::SetColor(tVector(pos_col[0], pos_col[1], pos_col[2], pos_col[3]));
				// cDrawUtil::DrawArrow2D(root_pos + offset, pos + offset, arrow_size);
				cDrawUtil::SetColor(tVector(vel_col[0], vel_col[1], vel_col[2], vel_col[3]));
				cDrawUtil::DrawArrow2D(pos, pos + curr_impulse, arrow_size);
			}
		}
	}
}

void cDrawSimCharacter::DrawTerainFeatures(const cSimCharacter& character, double marker_size,
											const tVector& terrain_col, const tVector& offset)
{
	const double arrow_size = marker_size * 0.65;
	int num_terrain_features = GetCharNumGroundFeatures(character);

 	tMatrix sample_trans = GetCharGroundSampleTrans(character);
	double base_h = sample_trans(1, 3);

	cDrawUtil::SetColor(tVector(terrain_col[0], terrain_col[1], terrain_col[2], terrain_col[3]));
	for (int i = 0; i < num_terrain_features; ++i)
	{
		tVector sample = GetCharGroundSample(character, i);
		
		tVector base_sample = sample;
		base_sample[1] = base_h;
		
		cDrawUtil::DrawArrow2D(base_sample + offset, sample + offset, arrow_size);
	}
}

void cDrawSimCharacter::DrawPolicyPlots(const cCharController* ctrl, const cCamera& cam)
{
	const tVector& char_feature_col = tVector(1, 0, 0, 1);
	const tVector& terr_feature_col = tVector(0, 0, 1, 1);
	const tVector& action_feature_col = tVector(0, 0.5, 0, 1);
	const tVector& action_val_col = tVector(0, 0.5, 0, 1);

	double aspect = cam.GetAspectRatio();
// #ifndef USE_OpenGLES
	cDrawUtil::MatrixMode(Util::PROJECTION_MAT);
	cDrawUtil::PushMatrix();
	cDrawUtil::LoadIdentity();

	cDrawUtil::MatrixMode(Util::MODELVIEW_MAT);
	cDrawUtil::PushMatrix();
	cDrawUtil::LoadIdentity();
// #endif
	const double h = 0.4;
	const double w = 16.0 / 9 * h / aspect;
	const double x_offset = -w * 1.05;
	const double y_offset = -h * 1.05;

	const double char_min_val = -2;
	const double char_max_val = 2;
	const double char_base_val = 0;

	Eigen::VectorXd char_features;
#if defined(ENABLE_DEBUG_VISUALIZATION)
	ctrl->GetVisCharacterFeatures(char_features);
#endif

	tVector char_features_plot_size = tVector(w, h, 0, 0);
	tVector char_features_plot_pos = tVector(1 + x_offset, 1 + y_offset, -1, 0);
	char_features_plot_pos += 0.5 * char_features_plot_size;
	char_features_plot_pos[1] += y_offset;

	cGraphUtil::tBarPlot char_features_plot;
	char_features_plot.mMinVal = char_min_val;
	char_features_plot.mMaxVal = char_max_val;
	char_features_plot.mBaseVal = char_base_val;
	char_features_plot.mVals = char_features;
	char_features_plot.mColors.push_back(char_feature_col);
	cGraphUtil::DrawBarPlot(char_features_plot, char_features_plot_pos, char_features_plot_size);

	
	const double terr_min_val = -2;
	const double terr_max_val = 2;
	const double terr_base_val = 0;

	Eigen::VectorXd terr_features;
#if defined(ENABLE_DEBUG_VISUALIZATION)
	ctrl->GetVisTerrainFeatures(terr_features);
#endif
	tVector terr_features_plot_size = char_features_plot_size;
	tVector terr_features_plot_pos = char_features_plot_pos;
	terr_features_plot_pos[0] += x_offset;

	cGraphUtil::tBarPlot terr_features_plot;
	terr_features_plot.mMinVal = terr_min_val;
	terr_features_plot.mMaxVal = terr_max_val;
	terr_features_plot.mBaseVal = terr_base_val;
	terr_features_plot.mVals = terr_features;
	terr_features_plot.mColors.push_back(terr_feature_col);
	cGraphUtil::DrawBarPlot(terr_features_plot, terr_features_plot_pos, terr_features_plot_size);


	const double action_min_val = -1;
	const double action_max_val = 1;
	const double action_base_val = 0;

	Eigen::VectorXd action_features;
#if defined(ENABLE_DEBUG_VISUALIZATION)
	ctrl->GetVisActionFeatures(action_features);
#endif
	tVector action_features_plot_size = char_features_plot_size;
	tVector action_features_plot_pos = char_features_plot_pos;
	action_features_plot_pos[1] += y_offset;

	cGraphUtil::tBarPlot action_features_plot;
	action_features_plot.mMinVal = action_min_val;
	action_features_plot.mMaxVal = action_max_val;
	action_features_plot.mBaseVal = action_base_val;
	action_features_plot.mVals = action_features;
	action_features_plot.mColors.push_back(action_feature_col);
	cGraphUtil::DrawBarPlot(action_features_plot, action_features_plot_pos, action_features_plot_size);


	const double val_min_val = 0;
	const double val_max_val = 1;
	const double val_base_val = 0;

	Eigen::VectorXd action_vals;
#if defined(ENABLE_DEBUG_VISUALIZATION)
	ctrl->GetVisActionValues(action_vals);
#endif
	if (action_vals.size() > 0)
	{
		tVector val_plot_size = action_features_plot_size;
		tVector val_plot_pos = action_features_plot_pos;
		val_plot_pos[0] += x_offset;

		cGraphUtil::tBarPlot val_plot;
		val_plot.mMinVal = val_min_val;
		val_plot.mMaxVal = val_max_val;
		val_plot.mBaseVal = val_base_val;
		val_plot.mVals = action_vals;
		val_plot.mColors.push_back(tVector(0, 0, 1, 0.5));
		val_plot.mColors.push_back(tVector(1, 0, 0, 0.5));
		val_plot.mColors.push_back(tVector(0, 0.5, 0, 0.5));
		val_plot.mColors.push_back(tVector(0.75, 0, 0.75, 0.5));
		val_plot.mColors.push_back(tVector(0, 0.5, 0.5, 0.5));
		val_plot.mColors.push_back(tVector(0, 0, 0, 0.5));
		cGraphUtil::DrawBarPlot(val_plot, val_plot_pos, val_plot_size);
	}
// #ifndef USE_OpenGLES
	cDrawUtil::MatrixMode(Util::PROJECTION_MAT);
	cDrawUtil::PopMatrix();

	cDrawUtil::MatrixMode(Util::MODELVIEW_MAT);
	cDrawUtil::PopMatrix();
// #endif
}

void cDrawSimCharacter::DrawInfoValLog(const cCircularBuffer<double>& val_log, double aspect)
{
	const double min_val = 0;
	const double max_val = 1;
	
	int num_val = static_cast<int>(val_log.GetSize());
	if (num_val > 0)
	{
// #ifndef USE_OpenGLES
		cDrawUtil::MatrixMode(Util::PROJECTION_MAT);
		cDrawUtil::PushMatrix();
		cDrawUtil::LoadIdentity();

		cDrawUtil::MatrixMode(Util::MODELVIEW_MAT);
		cDrawUtil::PushMatrix();
		cDrawUtil::LoadIdentity();
// #endif
		const double h = 0.4;
		const double w = 16.0 / 9 * h / aspect;

		tVector origin = tVector::Zero();
		origin[0] = 1 - w * 1.05;
		origin[1] = 1 - h * 1.05;
		origin[2] = -1;

		int capacity = static_cast<int>(val_log.GetCapacity());

		double prev_val = val_log[0];
		cDrawUtil::SetLineWidth(1);
		cDrawUtil::SetColor(tVector(1, 1, 1, 0.5));
		cDrawUtil::DrawRect(origin + 0.5 * tVector(w, h, 0, 0), tVector(w, h, 0, 0));
		cDrawUtil::SetColor(tVector(0, 0, 0, 1));
		cDrawUtil::DrawRect(origin + 0.5 * tVector(w, h, 0, 0), tVector(w, h, 0, 0), cDrawUtil::eDrawWire);

		cDrawUtil::SetLineWidth(1);
		cDrawUtil::SetPointSize(2);
		cDrawUtil::SetColor(tVector(1, 0, 0, 0.5));

		for (int i = 1; i < num_val; ++i)
		{
			double curr_val = val_log[i];

			tVector a = tVector::Zero();
			tVector b = tVector::Zero();

			a[0] = w * (i - 1.0) / (capacity - 1.0);
			b[0] = w * (i) / (capacity - 1.0);

			a[1] = h * cMathUtil::Clamp((prev_val - min_val) / (max_val - min_val), 0.0, 1.0);
			b[1] = h * cMathUtil::Clamp((curr_val - min_val) / (max_val - min_val), 0.0, 1.0);

			a += origin;
			b += origin;

			cDrawUtil::DrawLine(a, b);
			cDrawUtil::DrawPoint(b);
			prev_val = curr_val;
		}
// #ifndef USE_OpenGLES
		cDrawUtil::MatrixMode(Util::PROJECTION_MAT);
		cDrawUtil::PopMatrix();

		cDrawUtil::MatrixMode(Util::MODELVIEW_MAT);
		cDrawUtil::PopMatrix();
// #endif
	}
}

int cDrawSimCharacter::GetCharNumGroundFeatures(const cSimCharacter& character)
{
	const auto& ctrl = character.GetController();
	int num_samples = 0;
	if (ctrl != nullptr)
	{
		num_samples = ctrl->GetNumGroundSamples();
	}
	
	return num_samples;
}

tVector cDrawSimCharacter::GetCharGroundSample(const cSimCharacter& character, int i)
{
	const auto& ctrl = character.GetController();
	tVector sample = tVector::Zero();
	if (ctrl != nullptr)
	{
		sample = ctrl->GetGroundSample(i);
	}

	return sample;
}

tMatrix cDrawSimCharacter::GetCharGroundSampleTrans(const cSimCharacter& character)
{
	const auto& ctrl = character.GetController();
	tMatrix trans = tMatrix::Identity();
	if (ctrl != nullptr)
	{
		trans = ctrl->GetGroundSampleTrans();
	}

	return trans;
}

void cDrawSimCharacter::DrawSimBody(const cSimCharacter& character, const tVector& fill_tint, const tVector& line_col)
{
	const tVector gContactCol = tVector(0.5, 0.75, 0.5, 1);

	cDrawUtil::SetLineWidth(1);
	for (int i = 0; i < character.GetNumBodyParts(); ++i)
	{
		if (character.IsValidBodyPart(i))
		{
			const std::shared_ptr<cSimBox>& curr_part = std::static_pointer_cast<cSimBox>(character.GetBodyPart(i));
			tVector pos = curr_part->GetPos();

			tVector col;
			if (curr_part->IsInContact())
			{
				col = gContactCol;
			}
			else
			{
				col = character.GetPartColor(i);
				col = col.cwiseProduct(fill_tint);
			}

			cDrawUtil::SetColor(col);


			cDrawObj::Draw(curr_part.get(), cDrawUtil::eDrawSolid);
			tVector wire_color = tVector(0.3, 0.3, 0.3, 1);

			if (line_col[3] > 0)
			{
				cDrawUtil::SetColor(wire_color);
				cDrawObj::Draw(curr_part.get(), cDrawUtil::eDrawWire);
			}
		}
	}
}

void cDrawSimCharacter::DrawShapes(const cSimCharacter& character, const tVector& fill_tint, const tVector& line_col)
{
	// assert(character.HasDrawShapes());
	// const auto& shape_defs = character.GetDrawShapeDefs();
	// size_t num_shapes = shape_defs.rows();

	// cDrawUtil::SetLineWidth(1);
	// for (int i = 1; i < num_shapes; ++i)
	// {
	// 	cKinTree::tDrawShapeDef curr_def = shape_defs.row(i);
	// 	cDrawCharacter::DrawShape(character, curr_def, fill_tint, line_col);
	// }


	const tVector gContactCol = tVector(0.5, 0.75, 0.5, 1);





	//if (character.m_id>0)
	//{
	if (character.m_id>4)
	{
		cDrawUtil::SetLineWidth(1);
		for (int i = 0; i < character.GetNumBodyParts(); ++i)
		{
			if (character.IsValidBodyPart(i))
			{
				const std::shared_ptr<cSimBox>& curr_part = std::static_pointer_cast<cSimBox>(character.GetBodyPart(i));
				tVector pos = curr_part->GetPos();

				tVector col;
				if (curr_part->IsInContact())
				{
					col = gContactCol;
				}
				else
				{
					col = character.GetPartColor(i);
					col = col.cwiseProduct(fill_tint);
				}

				col[0]=col[0]+1;
				col[2]=col[2]-1;

				cDrawUtil::SetColor(col);
				cDrawObj::Draw(curr_part.get(), cDrawUtil::eDrawSolid);
				tVector wire_color = tVector(0.3, 0.3, 0.3, 1);

				if (line_col[3] > 0)
				{
					cDrawUtil::SetColor(wire_color);
					cDrawObj::Draw(curr_part.get(), cDrawUtil::eDrawWire);
				}
			}
		}
	}




}

void cDrawSimCharacter::DrawCtrlInfoGroundSamples(const cCharController* ctrl, const tVector& offset, bool draw_3d)
{
	const double r = 0.02;

	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(offset);

	int num_ground_samples = ctrl->GetNumGroundSamples();
	cDrawUtil::SetColor(tVector(1, 0, 0, 0.5));
	cDrawUtil::SetPointSize(3);
	for (int s = 0; s < num_ground_samples; ++s)
	{
		tVector sample_pos = ctrl->GetGroundSample(s);
		sample_pos[1] += 0.001;

		// glVertex3d(sample_pos[0], sample_pos[1], sample_pos[2]);
		tVector p_ = tVector(sample_pos[0], sample_pos[1], sample_pos[2], 0);
		cDrawUtil::DrawPoint(p_);

	}
	cDrawUtil::PopMatrix();

	auto waypoint_ctrl = dynamic_cast<const cWaypointController*>(ctrl);
	if (waypoint_ctrl != nullptr)
	{
		DrawCtrlInfoGroundSamples(waypoint_ctrl->GetLLC().get(), offset, draw_3d);
	}

	auto waypoint_vel_ctrl = dynamic_cast<const cWaypointVelController*>(ctrl);
	if (waypoint_vel_ctrl != nullptr)
	{
		DrawCtrlInfoGroundVelSamples(ctrl, offset, draw_3d);
	}
}

void cDrawSimCharacter::DrawCtrlInfoGroundVelSamples(const cCharController* ctrl, const tVector& offset, bool draw_3d)
{
	const double vel_scale = 0.2;

	auto waypoint_vel_ctrl = dynamic_cast<const cWaypointVelController*>(ctrl);
	if (waypoint_vel_ctrl != nullptr)
	{
		cDrawUtil::PushMatrix();
		cDrawUtil::Translate(offset);

		int num_ground_samples = waypoint_vel_ctrl->GetNumGroundSamples();
		cDrawUtil::SetColor(tVector(0, 0, 1, 0.5));
		cDrawUtil::SetLineWidth(2);

		for (int s = 0; s < num_ground_samples; ++s)
		{
			tVector vel = waypoint_vel_ctrl->GetGroundVelSample(s);
			tVector start = waypoint_vel_ctrl->GetGroundSample(s);
			tVector end = start + vel_scale * vel;
			//cDrawUtil::DrawLine(sample_pos, sample_pos + vel_scale * vel);
			cDrawUtil::DrawLine(start, end);
		}
		cDrawUtil::PopMatrix();
	}
}
