#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "chipmunk/chipmunk_private.h"
#include "chipmunk/cpHastySpace.h"

#define BODY_COUNT 8
#define JOINT_COUNT 6
#define ARBITER_CAPACITY (3*BODY_COUNT)

typedef struct Fixture {
	cpSpace *space;
	void (*step)(cpSpace *, cpFloat);
	void (*freeSpace)(cpSpace *);
	cpBody *bodies[BODY_COUNT];
	cpShape *shapes[BODY_COUNT], *ground;
	cpConstraint *joints[JOINT_COUNT];
	cpBody *active[BODY_COUNT];
	int activeCount;
	cpArbiter *arbiters[ARBITER_CAPACITY];
	int arbiterCount, contactCounts[ARBITER_CAPACITY];
	struct cpContact contacts[ARBITER_CAPACITY][CP_MAX_CONTACTS_PER_ARBITER];
} Fixture;

typedef struct WakeContext {
	Fixture *fixture;
	int outerCalls, innerCalls, postCalls;
} WakeContext;

static void
check(cpBool ok, const char *message)
{
	if(!ok){ fprintf(stderr, "Sleep/wake regression: %s\n", message); exit(1); }
}

static void
check_unique(cpArray *array)
{
	for(int i = 0; i < array->num; i++){
		check(array->arr[i] != NULL, "NULL active entry");
		for(int j = 0; j < i; j++) check(array->arr[i] != array->arr[j], "duplicate active entry");
	}
}

static void
check_body_order(Fixture *f)
{
	check(f->space->dynamicBodies->num == f->activeCount, "active body count");
	for(int i = 0; i < f->activeCount; i++)
		check(f->space->dynamicBodies->arr[i] == f->active[i], "swap-with-last body order changed");
	check_unique(f->space->dynamicBodies);
}

static void
init_fixture(Fixture *f, cpBool shapes, cpBool hasty)
{
	memset(f, 0, sizeof(*f));
	f->space = hasty ? cpHastySpaceNew() : cpSpaceNew();
	f->step = hasty ? cpHastySpaceStep : cpSpaceStep;
	f->freeSpace = hasty ? cpHastySpaceFree : cpSpaceFree;
	/* HastySpace defaults to one thread; keep tests deterministic. */
	cpSpaceSetGravity(f->space, cpv(0, -10));
	cpSpaceSetSleepTimeThreshold(f->space, 100);
	f->ground = cpSpaceAddShape(f->space, cpSegmentShapeNew(cpSpaceGetStaticBody(f->space), cpv(-10, 0), cpv(40, 0), 0));
	for(int i = 0; i < BODY_COUNT; i++){
		cpBody *body = cpSpaceAddBody(f->space, i == BODY_COUNT - 1 ? cpBodyNewKinematic() : cpBodyNew(1, 1));
		/* The first sleeping component has body/body contacts as well as
		 * ground contacts, exercising both ownership sides during restoration. */
		cpBodySetPosition(body, cpv(i < 3 ? 0.9*i : 4*i, i == BODY_COUNT - 1 ? 4 : 0.45));
		f->active[f->activeCount++] = f->bodies[i] = body;
		/* Keep an awake contact to trigger collision callbacks even when the
		 * sleeping components have no shapes. */
		if(shapes || i == 6) f->shapes[i] = cpSpaceAddShape(f->space, cpCircleShapeNew(body, 0.5, cpvzero));
	}
	cpBody *s = cpSpaceGetStaticBody(f->space);
	const int a[JOINT_COUNT] = {0, 1, -1, 3, 3, 6};
	const int b[JOINT_COUNT] = {1, 2, 0, 4, -1, 7};
	for(int i = 0; i < JOINT_COUNT; i++){
		f->joints[i] = cpSpaceAddConstraint(f->space, cpPinJointNew(a[i] < 0 ? s : f->bodies[a[i]], b[i] < 0 ? s : f->bodies[b[i]], cpvzero, cpvzero));
	}
	/* Initialize contact buffers and nonzero cached impulses before sleeping. */
	f->step(f->space, 1.0/60.0);
	check_body_order(f);
	f->arbiterCount = f->space->arbiters->num;
	check(f->arbiterCount <= ARBITER_CAPACITY, "fixture contact capacity");
	int dynamicPairs = 0;
	for(int i = 0; i < f->arbiterCount; i++){
		cpArbiter *arb = f->arbiters[i] = (cpArbiter *)f->space->arbiters->arr[i];
		if(cpBodyGetType(arb->body_a) == CP_BODY_TYPE_DYNAMIC && cpBodyGetType(arb->body_b) == CP_BODY_TYPE_DYNAMIC) dynamicPairs++;
		f->contactCounts[i] = arb->count;
		memcpy(f->contacts[i], arb->contacts, arb->count*sizeof(struct cpContact));
	}
	check(!shapes || dynamicPairs > 0, "shaped fixture must have dynamic/dynamic contacts");
}

static void
sleep_body(Fixture *f, int index, int group)
{
	cpBody *body = f->bodies[index];
	int at = 0;
	while(at < f->activeCount && f->active[at] != body) at++;
	check(at < f->activeCount, "sleep body must initially be active");
	f->active[at] = f->active[--f->activeCount];
	cpBodySleepWithGroup(body, group < 0 ? NULL : f->bodies[group]);
	check(cpBodyIsSleeping(body), "explicit sleep failed");
	check_body_order(f);
}

static void
prepare_sleep(Fixture *f)
{
	sleep_body(f, 0, -1);
	sleep_body(f, 1, 0);
	sleep_body(f, 2, 0);
	sleep_body(f, 3, -1);
	sleep_body(f, 4, 3);
	sleep_body(f, 5, -1);
	/* Repeating sleep with the same group must leave all lists unchanged. */
	cpBodySleepWithGroup(f->bodies[1], f->bodies[0]);
	check_body_order(f);
	check(f->space->constraints->num == 1, "only awake/kinematic joint should remain active");
	check(f->space->sleepingComponents->num == 3, "three sleeping components");
}

static const int wake_order[] = {0, 2, 1, 3, 4, 5};

static void
check_queue(WakeContext *ctx, int count)
{
	Fixture *f = ctx->fixture;
	check(cpSpaceIsLocked(f->space), "wake callback must hold space lock");
	check(f->space->rousedBodies->num == count, "wake queue count");
	check_unique(f->space->rousedBodies);
	for(int i = 0; i < count; i++){
		cpBody *body = f->bodies[wake_order[i]];
		check(f->space->rousedBodies->arr[i] == body, "wake queue order");
		check(!cpBodyIsSleeping(body), "queued body must no longer have a sleeping root");
		check(body->sleeping.next == NULL, "queued body still linked to sleeping component");
		if(f->shapes[wake_order[i]]){
			cpShape *shape = f->shapes[wake_order[i]];
			check(cpSpatialIndexContains(f->space->staticShapes, shape, shape->hashid), "shape moved before outer unlock");
		}
	}
	check_body_order(f); /* Queueing must not activate bodies before unlock. */
}

static void
check_contact_graph(Fixture *f)
{
	int total = 0;
	for(int i = 0; i <= BODY_COUNT; i++){
		cpBody *body = i == BODY_COUNT ? cpSpaceGetStaticBody(f->space) : f->bodies[i];
		cpArbiter *prev = NULL;
		int count = 0;
		for(cpArbiter *arb = body->arbiterList; arb;){
			check(++count <= f->space->arbiters->num, "cycle in body contact list");
			check(arb->body_a == body || arb->body_b == body, "contact linked to wrong body");
			check(cpArrayContains(f->space->arbiters, arb), "inactive contact remains threaded");
			struct cpArbiterThread *thread = cpArbiterThreadForBody(arb, body);
			check(thread->prev == prev, "broken contact back-link");
			prev = arb;
			arb = thread->next;
			total++;
		}
	}
	check(total == 2*f->space->arbiters->num, "contacts must be threaded once per body");
}

static void
check_restored(Fixture *f)
{
	check(!cpSpaceIsLocked(f->space), "space should be unlocked");
	check(f->space->rousedBodies->num == 0, "queue not drained");
	check(f->space->sleepingComponents->num == 0, "component roots not removed");
	check(f->space->constraints->num == JOINT_COUNT, "constraints not restored exactly once");
	check_unique(f->space->constraints);
	check_unique(f->space->arbiters);
	check_contact_graph(f);
	check_body_order(f);
	for(int i = 0; i < JOINT_COUNT; i++) check(cpArrayContains(f->space->constraints, f->joints[i]), "missing joint after wake");
	for(int i = 0; i < BODY_COUNT; i++){
		check(!cpBodyIsSleeping(f->bodies[i]), "body still sleeping after wake");
		if(f->shapes[i]) check(cpSpatialIndexContains(f->space->dynamicShapes, f->shapes[i], f->shapes[i]->hashid), "shape not restored to dynamic tree");
	}
}

static void
append_expected_wake(Fixture *f)
{
	check(f->activeCount == 2, "expected two bodies before wake append");
	for(int i = 0; i < 6; i++) f->active[f->activeCount++] = f->bodies[wake_order[i]];
}

static void
post_wake(cpSpace *space, void *key, void *data)
{
	WakeContext *ctx = (WakeContext *)data;
	(void)space; (void)key;
	ctx->postCalls++;
	append_expected_wake(ctx->fixture);
	check_restored(ctx->fixture);
}

static void
inner_query(cpShape *shape, void *data)
{
	WakeContext *ctx = (WakeContext *)data;
	Fixture *f = ctx->fixture;
	(void)shape;
	check(f->space->locked >= 2, "nested query must retain both locks");
	cpBodyActivate(f->bodies[4]);
	cpBodyActivate(f->bodies[5]);
	cpBodyActivate(f->bodies[3]);
	cpBodyActivate(f->bodies[1]);
	check_queue(ctx, 6);
	ctx->innerCalls++;
}

static void
outer_query(cpShape *shape, void *data)
{
	WakeContext *ctx = (WakeContext *)data;
	Fixture *f = ctx->fixture;
	(void)shape;
	if(ctx->outerCalls++ == 0){
		cpBodyActivate(f->bodies[2]);
		cpBodyActivate(f->bodies[0]);
		cpBodyActivate(f->bodies[1]);
		check_queue(ctx, 3);
		cpSpaceBBQuery(f->space, cpBBNew(-11, -2, 41, 6), CP_SHAPE_FILTER_ALL, inner_query, ctx);
		check(ctx->innerCalls > 0, "nested query callback not invoked");
		check_queue(ctx, 6); /* Inner unlock must not drain the queue. */
		check(ctx->postCalls == 0, "post-step callback ran too early");
		check(cpSpaceAddPostStepCallback(f->space, post_wake, ctx, ctx), "post-step registration failed");
	} else {
		cpBodyActivate(f->bodies[0]);
		cpBodyActivate(f->bodies[4]);
		check_queue(ctx, 6);
	}
}

static void
collision_wake(cpArbiter *arb, cpSpace *space, void *data)
{
	(void)arb; (void)space;
	outer_query(NULL, data);
}

static void
trace_state(Fixture *f, int mode, cpBool shapes)
{
	/* Deterministic differential-test output, never timing or throughput. */
	printf("mode=%d shapes=%d\n", mode, shapes);
	for(int i = 0; i < f->space->dynamicBodies->num; i++){
		cpBody *body = (cpBody *)f->space->dynamicBodies->arr[i];
		int id = 0;
		while(id < BODY_COUNT && f->bodies[id] != body) id++;
		check(id < BODY_COUNT, "unknown active body");
		printf("body=%d p=%a,%a v=%a,%a angle=%a w=%a\n", id, body->p.x, body->p.y, body->v.x, body->v.y, body->a, body->w);
	}
	for(int i = 0; i < JOINT_COUNT; i++) printf("joint=%d impulse=%a\n", i, cpConstraintGetImpulse(f->joints[i]));
}

static void
free_fixture(Fixture *f)
{
	for(int i = 0; i < JOINT_COUNT; i++){
		cpSpaceRemoveConstraint(f->space, f->joints[i]);
		cpConstraintFree(f->joints[i]);
	}
	for(int i = 0; i < BODY_COUNT; i++){
		if(f->shapes[i]){ cpSpaceRemoveShape(f->space, f->shapes[i]); cpShapeFree(f->shapes[i]); }
		cpSpaceRemoveBody(f->space, f->bodies[i]);
		cpBodyFree(f->bodies[i]);
	}
	cpSpaceRemoveShape(f->space, f->ground);
	cpShapeFree(f->ground);
	f->freeSpace(f->space);
}

static void
run_case(int mode, cpBool shapes, cpBool hasty)
{
	Fixture f;
	init_fixture(&f, shapes, hasty);
	prepare_sleep(&f);
	WakeContext ctx = {&f, 0, 0, 0};
	if(mode == 0){
		cpBodyActivate(f.bodies[2]);
		cpBodyActivate(f.bodies[4]);
		cpBodyActivate(f.bodies[5]);
		append_expected_wake(&f);
	} else {
		/* Expected append is visible to post-step checks, but callbacks check
		 * against the still-sleeping active-list model until unlock. */
		if(mode >= 2){
			cpCollisionHandler *handler = cpSpaceAddGlobalCollisionHandler(f.space);
			if(mode == 2) handler->preSolveFunc = collision_wake;
			else handler->postSolveFunc = collision_wake;
			handler->userData = &ctx;
			f.step(f.space, 1.0/60.0);
		} else {
			cpSpaceBBQuery(f.space, cpBBNew(-11, -2, 41, 6), CP_SHAPE_FILTER_ALL, outer_query, &ctx);
		}
		check(ctx.outerCalls > 0 && ctx.postCalls == 1, "wake/post-step callback counts");
	}
	check_restored(&f);
	if(mode < 2){
		for(int i = 0; i < f.arbiterCount; i++){
			check(f.arbiters[i]->count == f.contactCounts[i], "contact count changed during wake");
			check(memcmp(f.arbiters[i]->contacts, f.contacts[i], f.contactCounts[i]*sizeof(struct cpContact)) == 0, "cached contacts changed during sleep/wake");
		}
	}
	/* Remove the collision callback before another simulation step. */
	if(mode >= 2){
		cpCollisionHandler *handler = cpSpaceAddGlobalCollisionHandler(f.space);
		handler->preSolveFunc = cpCollisionHandlerDoNothing.preSolveFunc;
		handler->postSolveFunc = cpCollisionHandlerDoNothing.postSolveFunc;
	}
	f.step(f.space, 1.0/60.0);
	check_contact_graph(&f);
	trace_state(&f, mode, shapes);
	/* Re-sleep and wake to cover repeated pool/list reuse. */
	prepare_sleep(&f);
	cpBodyActivate(f.bodies[0]); cpBodyActivate(f.bodies[3]); cpBodyActivate(f.bodies[5]);
	append_expected_wake(&f);
	check_restored(&f);
	free_fixture(&f);
}

int
main(int argc, char **argv)
{
	cpBool hasty = argc == 2 && strcmp(argv[1], "--hasty") == 0;
	check(argc == 1 || hasty, "unexpected test arguments");
	for(int mode = 0; mode < 4; mode++){
		run_case(mode, cpFalse, hasty);
		run_case(mode, cpTrue, hasty);
	}
	puts("Sleep/wake bookkeeping tests passed.");
	return 0;
}
