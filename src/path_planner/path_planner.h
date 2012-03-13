#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <uquad_aux_math.h>

#define W_HOVER 334.28

typedef enum path_type{
    HOVER = 0,
    PATH_TYPE_COUNT
}path_type_t;

typedef struct set_point{
    uquad_mat_t *x;
    uquad_mat_t *w;
}set_point_t;

typedef struct path_planner{
    path_type_t pt;
    set_point_t *sp;
}path_planner_t;

path_planner_t *pp_init(void);

int pp_update_setpoint(path_planner_t *pp, uquad_mat_t *x);

void pp_deinit(path_planner_t *pp);

#endif //PATH_PLANNER_H
