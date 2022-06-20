#pragma once
#include "DrawScenarioImitateMocapEval.h"

class cDrawScenarioImitateMocapTargetEval : public cDrawScenarioImitateMocapEval
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioImitateMocapTargetEval(cCamera& cam);
	virtual ~cDrawScenarioImitateMocapTargetEval();

	virtual void Keyboard(unsigned char key, int x, int y);

protected:

	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;
	virtual void DrawMisc() const;
	virtual void DrawTargetPos() const;
	virtual void HandleRayTest(const cWorld::tRayTestResult& result);
	virtual void SetTargetPos(const tVector& pos);

	virtual bool EnableRandTargetPos() const;
	virtual void ToggleRandTargetPos();
};
