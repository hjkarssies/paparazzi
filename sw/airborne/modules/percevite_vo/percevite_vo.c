/*
 * Copyright (C) nilay994 <nilay994>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.

 */
/** @file "modules/percevite_vo/percevite_vo.h"
 * @author nilay994 <nilay994>
 * Percevite's Velocity Obsctacle for GPS and avoid
 */

 /* printf and fprintf */
#include <stdio.h>

/* MAKE MATRIX PTR */
#include "math/pprz_algebra_float.h"

#include "modules/percevite_vo/percevite_vo.h"

FILE *vo_f;

/* externed from percevite_wifi */
drone_data_t dr_data[MAX_DRONES];

void polar2cart(float mag, float directn, float *cart) {
  cart[0] = mag * cosf(directn);
  cart[1] = mag * sinf(directn);
}

void cart2polar(float *vel_vec, float *mag, float *dir) {
  *mag = float_vect_norm(vel_vec, 2);
  *dir = atan2f(vel_vec[1], vel_vec[0]);
}

void calc_proj_matrix(const float *a, const float *b, float **c) {
  float inner_product = float_vect_dot_product(a, b, 2);
  if (fabsf(inner_product) < 0.001) {
    inner_product = 0.001;
  }
  c[0][0] = a[0] * b[0] / inner_product;
  c[0][1] = a[0] * b[1] / inner_product;
  c[1][0] = a[1] * b[0] / inner_product;
  c[1][1] = a[1] * b[1] / inner_product;
}

void percevite_vo_resolve_by_project(drone_data_t *robot_a, float angle1, float angle2, float *centre, float *new_vel_a_bdy) {
    
    float vel_a_bdy[2], vel_a_w[2];
    polar2cart(robot_a->vel, robot_a->head, vel_a_bdy);

    // body frame to world frame velocity
    vel_a_w[0] = robot_a->pos.x + vel_a_bdy[0];
    vel_a_w[1] = robot_a->pos.y + vel_a_bdy[1];

    float yintercept = centre[1] - (tanf(angle1) * centre[0]);
    float xintercept = 0.0;

    // if slope is zero, projection is the x-coordinate itself
    if (angle2 == 0) {
      new_vel_a_bdy[0] = (vel_a_w[0] - robot_a->pos.x);
      new_vel_a_bdy[1] = -robot_a->pos.y;
      return;
    } else {
      xintercept = - yintercept / tanf(angle1);
    }

    float line_prop[3] = {xintercept, yintercept, tanf(angle1)};

    // slope-intercept form, poly1d(m, c), line = poly1d([line_prop[2], line_prop[1]])
    // calculate vector passing through origin, y = mx
    float vector_from_line[2] = {line_prop[0], -line_prop[1]};

    // prevent divide by zero
    if (float_vect_dot_product(vector_from_line, vector_from_line, 2) > 0.1) {
        // calculate projection matrix with property P**2 = P
        float _projection_matrix[2][2] = {0};
        MAKE_MATRIX_PTR(projection_matrix, _projection_matrix, 2);

        calc_proj_matrix(vector_from_line, vector_from_line, projection_matrix);

        float cc_vector[2] = {vel_a_w[0], (vel_a_w[1] - line_prop[1])};
        float projected_pt[2];
        float_mat_vect_mul(projected_pt, projection_matrix, cc_vector, 2, 2);
        projected_pt[1] = projected_pt[1] + line_prop[1];

        // resolved velocity on collision cone
        // plt.plot(newvela[0], newvela[1], 'og', alpha=0.2)

        // convert back to body frame velocities, make this the new body frame velocity
        new_vel_a_bdy[0] = projected_pt[0] - robot_a->pos.x;
        new_vel_a_bdy[1] = projected_pt[1] - robot_a->pos.y;
        
    } else {
        // if norm is lesser, give back old vel.. (not sure)
        new_vel_a_bdy[0] = vel_a_bdy[0];
        new_vel_a_bdy[1] = vel_a_bdy[1];
    }
    // return resolved velocity in cartesian;
}


void percevite_vo_detect(drone_data_t *robot1, drone_data_t *robot2) {
  static bool first_call = true;
  static float oldvel, oldhead;
  // hacky restore..
  if (first_call) {
    oldvel  = robot1->vel;
    oldhead = robot1->head;
    first_call = false;
  } 
  
  float drel[2], vrel[2]; 
  
  drel[0] = robot1->pos.x - robot2->pos.x;
  drel[1] = robot1->pos.y - robot2->pos.y;

  float robot1vel_cart[2], robot2vel_cart[2];
  polar2cart(robot1->vel, robot1->head, robot1vel_cart);
  polar2cart(robot2->vel, robot2->head, robot2vel_cart);
  vrel[0] = robot1vel_cart[0] - robot2vel_cart[0];
  vrel[1] = robot1vel_cart[1] - robot2vel_cart[1];

  float norm_vrel = float_vect_norm(vrel, 2);
  if (norm_vrel < 0.1) {
    norm_vrel = 0.1;
  }

  // for details on these two variables read Dennis Wijngaarden's prelim
  float tcpa = -float_vect_dot_product(drel, vrel, 2) / pow(norm_vrel, 2);
  float dcpa = sqrtf((fabsf(pow(float_vect_norm(drel, 2), 2) - (pow(tcpa, 2) * pow(float_vect_norm(vrel, 2), 2)))));
  // printf("tcpa: %f, dcpa: %f\n", tcpa, dcpa);

  float angleb = atan2(robot2->pos.y, robot2->pos.x);
  float deltad = float_vect_norm(drel,2);
  float angleb1 = angleb - atan(RR / deltad);
  float angleb2 = angleb + atan(RR / deltad);

  float robot2_offset[2];
  polar2cart(robot2->vel, robot2->head, robot2_offset);

  float centre[2];
  centre[0] = robot1->pos.x + robot2_offset[0]; 
  centre[1] = robot1->pos.y + robot2_offset[1];
  
  // collision is imminent if tcpa > 0 and dcpa < RR
  if ((tcpa > 0) && (dcpa < RR)) {
    float newvel_cart[2];
    percevite_vo_resolve_by_project(robot1, angleb1, angleb2, centre, newvel_cart);

    float resolved_vel_bdy, resolved_head_bdy;
    cart2polar(newvel_cart, &resolved_vel_bdy, &resolved_head_bdy);

    /* TODO: rate and mag bound here and send to ctrl outerloop */
    resolved_vel_bdy = min(max(resolved_vel_bdy, -3.0), 3.0);
    resolved_head_bdy = resolved_head_bdy + 0.0;

    if (robot1->vel < resolved_vel_bdy) {
      printf("Speed up\n");
    } 
    if (robot1->vel > resolved_vel_bdy) {
      printf("Slow down\n");
    }
    if (robot1->head < resolved_head_bdy) {
      printf("Rotate couter-cw\n");
    }
    if (robot1->head > resolved_head_bdy) {
      printf("Rotate cw\n");
    }
    
    robot1->vel = resolved_vel_bdy;
    robot1->head = resolved_head_bdy;
  }
  // if no collision predicted, don't resolve and be what you were
  else {
    robot1->vel  = oldvel;
    robot1->head = oldhead;
  }
}

void vo_simulate_loop(drone_data_t* robot_sim) {
  float del[2];
  polar2cart(robot_sim->vel, robot_sim->head, del);
  robot_sim->pos.x += del[0];
  robot_sim->pos.y += del[1];
}

drone_data_t drone1, drone2;
void percevite_vo_init(void) {
  #ifdef SIM
  drone1.pos.x = -25.0;
  drone1.pos.y = -25.0;
  drone1.vel = 1.3;
  drone1.head = (D2R) * 45.0;

  drone2.pos.x = -10.0;
  drone2.pos.y = -10.0;
  drone2.vel = 0.4;
  drone2.head = (D2R) * 45.0;
  #endif

  // vo_f = fopen("vo.csv", "w+");
}

/* 5Hz periodic loop */
void percevite_vo_periodic(void) {

  // simulate what happens to both robots to later extern it to Percevite WiFi
  // overwrite dr_data for tx...
  #ifdef SIM
  if (SELF_ID == 1) {
    vo_simulate_loop(&drone1);
    dr_data[SELF_ID] = drone1;
  } 
  if (SELF_ID == 2) {
    vo_simulate_loop(&drone2);
    dr_data[SELF_ID] = drone2;
  }
  #else
  // make copies from externed dr_data of percevite_wifi
  drone1 = dr_data[1];
  drone2 = dr_data[2];
  #endif

  // change the vel and head of robot1
  percevite_vo_detect(&drone1, &drone2);

}

/* print projection matrix
for (int it = 0; it < 2; it++) {
  for (int jt = 0; jt < 2; jt++) {
    printf("%f, ", projection_matrix[it][jt]);
  }
  printf("\n");
}
printf("\n"); 

// TODO: remember to comment propagation of the drone which is not curr drone.. 
void vo_simulate_loop(void) {
  float del[2];
  polar2cart(robot1.vel, robot1.head, del);
  robot1.pos[0] += del[0];
  robot1.pos[1] += del[1];

  polar2cart(robot2.vel, robot2.head, del);
  robot2.pos[0] += del[0];
  robot2.pos[1] += del[1];

  fprintf(vo_f, "%f,%f,%f,%f,%f,%f,%f,%f\n", 
          robot1.pos[0], robot1.pos[1], robot1.vel, robot1.head,
          robot2.pos[0], robot2.pos[1], robot2.vel, robot2.head);
} */