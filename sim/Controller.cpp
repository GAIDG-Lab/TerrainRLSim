#include "Controller.h"
#include "sim/SimCharacter.h"
#include "util/FileUtil.h"
#include <iostream>

cController::cController()
{
}

cController::~cController()
{

}


void cController::Init(cSimCharacter* character)
{
	std::cout << "cController Init 21! "  << std::endl;
	assert(character != nullptr);
	Clear();
	mChar = character;
	std::cout << "cController Init done 21! "  << std::endl;
}

void cController::Reset()
{
	SetMode(eModeActive);
}


void cController::Clear()
{
	mValid = false;
	mChar = nullptr;
	SetMode(eModeActive);
}

void cController::Update(double time_step)
{
	// *whistle whistle*....nothing to see here
}

bool cController::IsValid() const
{
	return mValid;
}

bool cController::LoadParams(const std::string& param_file)
{
	return true;
}

void cController::BuildOptParams(Eigen::VectorXd& out_params) const
{
}

void cController::SetOptParams(const Eigen::VectorXd& params)
{
}

void cController::SetOptParams(const Eigen::VectorXd& params, Eigen::VectorXd& out_params) const
{
	out_params = params;
}

int cController::GetNumOptParams() const
{
	return 0;
}

void cController::FetchOptParamScale(Eigen::VectorXd& out_scale) const
{
}

void cController::OutputOptParams(const std::string& file, const Eigen::VectorXd& params) const
{
	FILE* f = cFileUtil::OpenFile(file, "w");
	if (f != nullptr)
	{
		OutputOptParams(f, params);
		cFileUtil::CloseFile(f);
	}
}

void cController::OutputOptParams(FILE* f, const Eigen::VectorXd& params) const
{
}

void cController::ReadParams(const std::string& file)
{
	if (file != "")
	{
		std::ifstream f_stream(file);
		if (f_stream.is_open())
		{
			ReadParams(f_stream);
		}
		f_stream.close();
	}
}

void cController::ReadParams(std::ifstream& f_stream)
{
}

void cController::SetActive(bool active)
{
	SetMode((active) ? eModeActive : eModeInactive);
}

bool cController::IsActive() const
{
	return mMode != eModeInactive;
}

void cController::SetMode(eMode mode)
{
	mMode = mode;
}

cWorld::eSimMode cController::GetSimMode() const
{
	const auto& world = mChar->GetWorld();
	cWorld::eSimMode sim_mode = world->GetSimMode();
	return sim_mode;
}

bool cController::Is3D() const
{
	cWorld::eSimMode sim_mode = GetSimMode();
	return sim_mode == cWorld::eSimMode3D;
}
