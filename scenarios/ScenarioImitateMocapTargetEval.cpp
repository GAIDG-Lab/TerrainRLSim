#include "ScenarioImitateMocapTargetEval.h"
#include "scenarios/ScenarioExpImitateMocapTarget.h"
#include "sim/CtTargetController.h"
#include "sim/CtPhaseController.h"

cScenarioImitateMocapTargetEval::cScenarioImitateMocapTargetEval() :
	cScenarioExpImitateMocapTarget(),
	cScenarioImitateMocapEval()
{
}

cScenarioImitateMocapTargetEval::~cScenarioImitateMocapTargetEval()
{
}

std::string cScenarioImitateMocapTargetEval::GetName() const
{
	return "Imitate Target Evaluation";
}