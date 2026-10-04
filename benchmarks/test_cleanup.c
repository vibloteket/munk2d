#include "chipmunk/chipmunk.h"

#include <stdio.h>
#include <stdlib.h>

/* Exercise the actual benchmark cleanup, counting frees even without LSan. */
static int freed_constraints;

static void
counted_constraint_free(cpConstraint *constraint)
{
	freed_constraints++;
	cpConstraintFree(constraint);
}

#define cpConstraintFree counted_constraint_free
#define main munkbench_cli_main
#include "munkbench.c"
#undef main
#undef cpConstraintFree

static void
require_cleanup(cpBool condition, const char *message)
{
	if(!condition){
		fprintf(stderr, "MunkBench cleanup regression: %s\n", message);
		exit(1);
	}
}

static void
check_cleanup(cpSpace *space, int expected_constraints)
{
	freed_constraints = 0;
	free_space_children(space);
	require_cleanup(freed_constraints == expected_constraints,
		"not all owned constraints were freed");

	PtrArray remaining;
	ptr_array_init(&remaining);
	cpSpaceEachConstraint(space, collect_constraint, &remaining);
	cpSpaceEachShape(space, collect_shape, &remaining);
	cpSpaceEachBody(space, collect_body, &remaining);
	require_cleanup(remaining.count == 0, "space still owns children after cleanup");
	ptr_array_dispose(&remaining);
	/* Also verifies that cleanup did not free the embedded static body. */
	cpSpaceFree(space);
}

static void
check_sleeping_constraints(cpBool with_shapes)
{
	cpSpace *space = cpSpaceNew();
	cpSpaceSetSleepTimeThreshold(space, 0.5);
	cpBody *a = cpSpaceAddBody(space, cpBodyNew(1, 1));
	cpBody *b = cpSpaceAddBody(space, cpBodyNew(1, 1));
	cpBody *awake = cpSpaceAddBody(space, cpBodyNew(1, 1));
	cpBody *user_static = cpSpaceAddBody(space, cpBodyNewStatic());
	cpSpaceAddConstraint(space, cpPivotJointNew2(cpSpaceGetStaticBody(space), a, cpvzero, cpvzero));
	cpSpaceAddConstraint(space, cpPivotJointNew2(a, b, cpvzero, cpvzero));
	cpSpaceAddConstraint(space, cpPivotJointNew2(awake, user_static, cpvzero, cpvzero));
	if(with_shapes){
		cpSpaceAddShape(space, cpCircleShapeNew(a, 1, cpvzero));
		cpSpaceAddShape(space, cpCircleShapeNew(b, 1, cpvzero));
	}
	cpBodySleep(a);
	cpBodySleepWithGroup(b, a);
	require_cleanup(cpBodyIsSleeping(a) && cpBodyIsSleeping(b), "test bodies must sleep");

	PtrArray active;
	ptr_array_init(&active);
	cpSpaceEachConstraint(space, collect_constraint, &active);
	require_cleanup(active.count == 1, "sleeping constraints must be absent from active iteration");
	ptr_array_dispose(&active);
	check_cleanup(space, 3);
}

static void
check_constraint_mix(int steps)
{
	void *data = NULL;
	cpSpace *space = init_constraint_mix(100, &data);
	ConstraintMixState *state = (ConstraintMixState *)data;
	for(int i = 0; i < steps; i++) constraint_mix_update(space, state, 1.0/FPS);
	int count = state->count;
	check_cleanup(space, count);
	constraint_mix_destroy_state(state);
}

int
main(void)
{
	cpSpace *empty = cpSpaceNew();
	Summary summary = collect_summary(empty, 0);
	require_cleanup(summary.contact_pairs == 0, "empty summary must have no contacts");
	check_cleanup(empty, 0);
	check_sleeping_constraints(cpTrue);
	check_sleeping_constraints(cpFalse);
	check_constraint_mix(0);
	check_constraint_mix(181);
	check_constraint_mix(600);
	printf("MunkBench cleanup frees active and sleeping constraints, with and without shapes.\n");
	return 0;
}
