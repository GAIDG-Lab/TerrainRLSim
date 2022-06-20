#include "DrawScenarioImitateMocapEval.h"
#include "scenarios/ScenarioImitateMocapEval.h"
#include "anim/KinCharacter.h"
#include "render/DrawUtil.h"
#include "render/DrawCharacter.h"

const double gLinkWidth = 0.025;
const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gFilLColor = tVector(0.6, 0.65, 0.675, 1);
const tVector gKinCharOffset = tVector(0, 0, 0.5, 0);

cDrawScenarioImitateMocapEval::cDrawScenarioImitateMocapEval(cCamera& cam)
	: cDrawScenarioPoliEval(cam)
{
	mDrawKinChar = true;
}

cDrawScenarioImitateMocapEval::~cDrawScenarioImitateMocapEval()
{
}

void cDrawScenarioImitateMocapEval::Init()
{
	cDrawScenarioPoliEval::Init();
	mDrawKinChar = true;
}

void cDrawScenarioImitateMocapEval::Reset()
{
	auto eval_scene = std::dynamic_pointer_cast<cScenarioPoliEval>(mScene);
	if (eval_scene != nullptr)
	{
		eval_scene->EndEpisodeRecord();
	}
	cDrawScenarioPoliEval::Reset();
}

void cDrawScenarioImitateMocapEval::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenarioPoliEval::Keyboard(key, x, y);

	switch (key)
	{
	case 'k':
		ToggleDrawKinChar();
		break;
	default:
		break;
	}
}

void cDrawScenarioImitateMocapEval::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioImitateMocapEval>(new cScenarioImitateMocapEval());
}

void cDrawScenarioImitateMocapEval::DrawCharacters() const
{
	//cDrawScenarioPoliEval::DrawCharacters();

	if (mDrawKinChar)
	{
		DrawKinChar();
	}
}

void cDrawScenarioImitateMocapEval::DrawKinChar() const
{
	/**
	const auto& kin_char = GetKinChar();
	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(GetDrawKinCharOffset());
	cDrawCharacter::Draw(*kin_char.get(), gLinkWidth, gFilLColor, GetLineColor());
	cDrawUtil::PopMatrix();

	const auto& kin_char2 = GetKinChar2();
	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(tVector(1, 0, 0, 0));
	cDrawCharacter::Draw(*kin_char2.get(), gLinkWidth, gFilLColor, GetLineColor());
	cDrawUtil::PopMatrix();
	**/
	std::vector<std::shared_ptr<cKinCharacter>> chars = GetKinChars();
	for(int i = 0; i < chars.size(); i++){
		const auto& kin_char = chars[i];
		cDrawUtil::PushMatrix();
		cDrawUtil::Translate(tVector(i-10, 0, 0, 0));
		cDrawCharacter::Draw(*kin_char.get(), gLinkWidth, gFilLColor, GetLineColor());
		cDrawUtil::PopMatrix();
	}
}

const std::shared_ptr<cKinCharacter>& cDrawScenarioImitateMocapEval::GetKinChar() const
{
	auto scene = std::dynamic_pointer_cast<cScenarioImitateMocapEval>(mScene);
	return scene->GetKinChar();
}

const std::shared_ptr<cKinCharacter>& cDrawScenarioImitateMocapEval::GetKinChar2() const
{
	auto scene = std::dynamic_pointer_cast<cScenarioImitateMocapEval>(mScene);
	return scene->GetKinChar2();
}

const std::vector<std::shared_ptr<cKinCharacter>>& cDrawScenarioImitateMocapEval::GetKinChars() const
{
	auto scene = std::dynamic_pointer_cast<cScenarioImitateMocapEval>(mScene);
	return scene->GetKinChars();
}

const std::shared_ptr<cKinCharacter>& cDrawScenarioImitateMocapEval::CreateKinChar() const
{
	auto kin_char = std::shared_ptr<cKinCharacter>(new cKinCharacter());
	kin_char->EnableVelUpdate(true);
	bool succ = kin_char->Init("/home/beth/Desktop/playground/TerrainRLSim/data/characters/biped3d_mocap.txt",
							   "/home/beth/Desktop/playground/TerrainRLSim/data/motions/biped3d_walk.txt");
	return kin_char;
}

void cDrawScenarioImitateMocapEval::ToggleDrawKinChar()
{
	mDrawKinChar = !mDrawKinChar;
	if (mDrawKinChar)
	{
		printf("Mocap\n");
		printf("Enable draw kin character\n");
	}
	else
	{
		printf("Disable draw kin character\n");
	}
}

tVector cDrawScenarioImitateMocapEval::GetDrawKinCharOffset() const
{
	/*
	const auto& kin_char = GetKinChar();
	const auto& sim_char = mScene->GetCharacter();
	tVector kin_pos = kin_char->GetRootPos();
	tVector sim_pos = sim_char->GetRootPos();
	tVector target_pos = sim_pos += tVector(-1, 0, 1, 0);
	target_pos[1] = kin_pos[1];
	return target_pos - kin_pos;
	*/
	eDrawMode draw_mode = GetDrawMode();
	if (draw_mode == eDrawMode2D)
	{
		return gKinCharOffset;
	}
	return tVector::Zero();
}

tVector cDrawScenarioImitateMocapEval::GetCamTrackPos() const
{
	return cDrawScenarioPoliEval::GetCamTrackPos();
	//return GetKinChar()->GetRootPos();
}
