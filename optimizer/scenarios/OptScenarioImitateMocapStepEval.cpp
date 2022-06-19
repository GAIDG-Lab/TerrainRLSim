#include "OptScenarioImitateMocapStepEval.h"
#include "scenarios/ScenarioImitateStepEval.h"

cOptScenarioImitateMocapStepEval::cOptScenarioImitateMocapStepEval()
{
}

cOptScenarioImitateMocapStepEval::~cOptScenarioImitateMocapStepEval()
{
}

void cOptScenarioImitateMocapStepEval::BuildScene(int id, std::shared_ptr<cScenarioPoliEval>& out_scene) const
{
	auto eval_scene = std::shared_ptr<cScenarioImitateStepEval>(new cScenarioImitateStepEval());
	out_scene = eval_scene;
}