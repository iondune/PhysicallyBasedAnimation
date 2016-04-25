
#pragma once

#include <ionCore.h>

class MosekSolver
{

public:

	static void Solve();

private:

	static void * Environment;

	static void Init();

};
