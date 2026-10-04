/* Same ordered formulas, used as a portable reference for validation. */
#include "cpContactSolver.h"
#include "cpContactSolverScalar.inc"

void
cpContactSolverKernelScalar(cpContactSolverContext *s, int iterations)
{
 for(int iteration = 0; iteration < iterations; ++iteration){
  for(int i = 0; i < s->packetCount; ++i){
   cpContactSolverPacket *p = &s->packets[i];
   for(int lane = 0; lane < p->lanes; ++lane) solveScalarPacket(s, p, lane);
  }
 }
}
