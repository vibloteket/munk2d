#include <stdio.h>
#include <stdlib.h>

#if !defined(_WIN32)
#include <sys/resource.h>
#endif

#include "chipmunk/chipmunk.h"

static void
limit_stack_for_regression(void)
{
#if !defined(_WIN32)
  struct rlimit limit;

  limit.rlim_cur = 256 * 1024;
  limit.rlim_max = 256 * 1024;
  setrlimit(RLIMIT_STACK, &limit);
#endif
}

static void
count_shape(cpShape *shape, void *data)
{
  int *count = (int *)data;
  (void)shape;
  (*count)++;
}

static void
free_space_shapes(cpSpace *space, cpShape **shapes, int count)
{
  int i;
  for(i = 0; i < count; i++) {
    cpSpaceRemoveShape(space, shapes[i]);
    cpShapeFree(shapes[i]);
  }
}

/* Public spatial-index regression: preserve swap-with-last iteration order,
 * membership, node reuse, and movement between paired static/dynamic trees. */
typedef struct LeafItem { cpBB bb; int id; } LeafItem;
typedef struct LeafModel {
  cpSpatialIndex *tree;
  LeafItem *items[64];
  int count, visited;
} LeafModel;

static void leaf_check(int ok)
{
  if(!ok) { fprintf(stderr, "BBTree leaf-array regression\n"); exit(1); }
}
static cpBB leaf_bounds(void *obj) { return ((LeafItem *)obj)->bb; }
static void check_leaf_order(void *obj, void *data)
{
  LeafModel *model = (LeafModel *)data;
  leaf_check(model->visited < model->count);
  leaf_check(model->items[model->visited++] == obj);
}
static void check_leaf_model(LeafModel *model)
{
  leaf_check(cpSpatialIndexCount(model->tree) == model->count);
  model->visited = 0;
  cpSpatialIndexEach(model->tree, check_leaf_order, model);
  leaf_check(model->visited == model->count);
  for(int i = 0; i < model->count; i++)
    leaf_check(cpSpatialIndexContains(model->tree, model->items[i], (cpHashValue)model->items[i]->id));
}
static void add_leaf(LeafModel *model, LeafItem *item)
{
  leaf_check(!cpSpatialIndexContains(model->tree, item, (cpHashValue)item->id));
  cpSpatialIndexInsert(model->tree, item, (cpHashValue)item->id);
  model->items[model->count++] = item;
  check_leaf_model(model);
}
static LeafItem *remove_leaf(LeafModel *model, int slot)
{
  LeafItem *item = model->items[slot];
  cpSpatialIndexRemove(model->tree, item, (cpHashValue)item->id);
  model->items[slot] = model->items[--model->count];
  model->items[model->count] = NULL;
  leaf_check(!cpSpatialIndexContains(model->tree, item, (cpHashValue)item->id));
  check_leaf_model(model);
  return item;
}
static void test_leaf_array_order(void)
{
  LeafItem items[64];
  LeafModel models[2] = {{0}, {0}};
  models[0].tree = cpBBTreeNew(leaf_bounds, NULL);
  models[1].tree = cpBBTreeNew(leaf_bounds, models[0].tree);
  for(int i = 0; i < 64; i++) {
    items[i].id = i + 1;
    items[i].bb = cpBBNew(i%8, i/8, i%8 + 1.5, i/8 + 1.5);
    add_leaf(&models[1], &items[i]);
  }
  /* Explicitly exercise first, middle, last, and final-leaf removal. */
  for(int i = 0; i < 64; i++) {
    int n = models[1].count;
    remove_leaf(&models[1], i%3 == 0 ? 0 : (i%3 == 1 ? n/2 : n-1));
  }
  for(int i = 0; i < 64; i++) add_leaf(&models[i%2], &items[i]);
  unsigned int random = 0x12345678u;
  for(int step = 0; step < 512; step++) {
    random ^= random << 13; random ^= random >> 17; random ^= random << 5;
    int from = (int)(random & 1u);
    if(models[from].count) {
      LeafItem *item = remove_leaf(&models[from], (int)((random >> 1) % (unsigned int)models[from].count));
      add_leaf(&models[1-from], item);
      item->bb = cpBBOffset(item->bb, cpv(0.125, -0.0625));
      cpSpatialIndexReindexObject(models[1-from].tree, item, (cpHashValue)item->id);
    }
    if(step%17 == 0) {
      cpBBTreeOptimize(models[0].tree);
      cpBBTreeOptimize(models[1].tree);
    }
    check_leaf_model(&models[0]);
    check_leaf_model(&models[1]);
  }
  for(int i = 0; i < 2; i++)
    while(models[i].count) remove_leaf(&models[i], models[i].count/2);
  cpSpatialIndexFree(models[1].tree);
  cpSpatialIndexFree(models[0].tree);
}

int
main(void)
{
  const int count = 4000;
  int i;
  int hits = 0;
  cpSpace *space = cpSpaceNew();
  cpShape **shapes = (cpShape **)calloc((size_t)count, sizeof(cpShape *));
  cpBody *body = cpSpaceGetStaticBody(space);

  if(space == NULL || shapes == NULL) {
    fprintf(stderr, "Failed to allocate bb tree test resources.\n");
    free(shapes);
    cpSpaceFree(space);
    return 1;
  }

  limit_stack_for_regression();
  test_leaf_array_order();

  for(i = 0; i < count; i++) {
    shapes[i] = cpCircleShapeNew(body, 1.0, cpvzero);
    cpSpaceAddShape(space, shapes[i]);
  }

  cpSpaceBBQuery(space, cpBBNew(-2, -2, 2, 2), CP_SHAPE_FILTER_ALL, count_shape, &hits);
  if(hits != count) {
    fprintf(stderr, "Expected %d query hits, got %d.\n", count, hits);
    free_space_shapes(space, shapes, count);
    free(shapes);
    cpSpaceFree(space);
    return 1;
  }

  free_space_shapes(space, shapes, count);
  free(shapes);
  cpSpaceFree(space);

  printf("Inserted and queried %d degenerate shapes without overflowing the BB tree traversal stack.\n", count);
  return 0;
}
