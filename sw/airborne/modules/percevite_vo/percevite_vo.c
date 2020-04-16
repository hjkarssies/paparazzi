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
#include "modules/percevite_vo/percevite_vo.h"
#include "math/pprz_algebra_float.h"
#include <stdio.h>

FILE *vo_f;

void polar2cart(float mag, float directn, float *cart) {
  cart[0] = mag * cosf(directn);
  cart[1] = mag * sinf(directn);
}

void cart2polar(float *vel_vec, float *vel_polar) {
  float mag = float_vect_norm(vel_vec, 2);
  float dir = atan2f(vel_vec[1], vel_vec[0]);
  vel_polar[0] = mag;
  vel_polar[1] = dir;
}

// TODO: FloatVect elements referred with ".x, .y"
// pass them via reference..

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

void vo_init(void) {
  // // your init code here
  // robot1.pos[0] = -7.0;
  // robot1.pos[1] = 0.0;
  // robot1.vel = 0.4;
  // robot1.head = (D2R) * 45.0;
  // robot1.oldvel = 0.4;
  // robot1.oldhead = (D2R) * 45.0;

  // robot2.pos[0] = 7.0;
  // robot2.pos[1] = 0.0;
  // robot2.vel = 0.4;
  // robot2.head = (D2R) * 135.0;
  // robot2.oldvel = 0.4;
  // robot2.oldhead = (D2R) * 135.0;
  vo_f = fopen("vo.csv", "w+");
}

void vo_resolve_by_project(robot_t robot_a, float angle1, float angle2, float *centre, float *new_vela) {
    
    // body frame to world frame velocity
    float vela_cart[2], curr_vela[2];
    polar2cart(robot_a.vel, robot_a.head, vela_cart);

    curr_vela[0] = robot_a.pos[0] + vela_cart[0];
    curr_vela[1] = robot_a.pos[1] + vela_cart[1];

    float yintercept = centre[1] - (tanf(angle1) * centre[0]);

    float xintercept = 0.0;
    // if slope is zero, projection is the x-coordinate itself
    if (angle2 == 0) {
      new_vela[0] = (curr_vela[0] - robot_a.pos[0]);
      new_vela[1] = -robot_a.pos[1];
    } else {
      xintercept = - yintercept / tanf(angle1);
    }
        
    float line_prop[3] = {xintercept, yintercept, tanf(angle1)};

    // slope - intercept form, poly1d(m, c)
    // line1 = poly1d([line_prop[2], line_prop[1]])

    // calculate vector passing through origin, y = mx
    float vector_from_line[2] = {line_prop[0], -line_prop[1]};

    if (float_vect_dot_product(vector_from_line, vector_from_line, 2) > 0.1) {
        // calculate projection matrix
        // projection matrix P**2 = P
        float _projection_matrix[2][2] = {0};
        MAKE_MATRIX_PTR(projection_matrix, _projection_matrix, 2);

        calc_proj_matrix(vector_from_line, vector_from_line, projection_matrix);

        // for (int it = 0; it < 2; it++) {
        //   for (int jt = 0; jt < 2; jt++) {
        //     printf("%f, ", projection_matrix[it][jt]);
        //   }
        //   printf("\n");
        // }

        // printf("\n");
        float cc_vector[2] = {curr_vela[0], (curr_vela[1] - line_prop[1])};
        float projected_pt[2];
        float_mat_vect_mul(projected_pt, projection_matrix, cc_vector, 2, 2);
        projected_pt[1] = projected_pt[1] + line_prop[1];

        // resolved velocity on collision cone
        // plt.plot(newvela[0], newvela[1], 'og', alpha=0.2)

        // convert back to body frame velocities
        // make this the new body frame velocity
        // return newvela;
        new_vela[0] = projected_pt[0] - robot_a.pos[0];
        new_vela[1] = projected_pt[1] - robot_a.pos[1];
        
    } else {
        // back to body frame velocity
        new_vela[0] = curr_vela[0] - robot_a.pos[0];
        new_vela[1] = curr_vela[1] - robot_a.pos[1];
        // return new_vela;
    }

}

void detect(void) {
  float drel[2], vrel[2]; 
  
  drel[0] = robot1.pos[0] - robot2.pos[0];
  drel[1] = robot1.pos[1] - robot2.pos[1];

  float robot1vel_cart[2], robot2vel_cart[2];
  polar2cart(robot1.vel, robot1.head, robot1vel_cart);
  polar2cart(robot2.vel, robot2.head, robot2vel_cart);
  vrel[0] =  robot1vel_cart[0] - robot2vel_cart[0];
  vrel[1] =  robot1vel_cart[1] - robot2vel_cart[1];

  float norm_vrel = float_vect_norm(vrel, 2);
  if (norm_vrel < 0.1) {
    norm_vrel = 0.1;
  }

  float tcpa = -float_vect_dot_product(drel, vrel, 2) / pow(norm_vrel, 2);
  float dcpa = sqrtf((fabsf(pow(float_vect_norm(drel, 2), 2) - (pow(tcpa, 2) * pow(float_vect_norm(vrel, 2), 2)))));
  // printf("tcpa: %f, dcpa: %f\n", tcpa, dcpa);

  float angleb = atan2(robot2.pos[1], robot2.pos[0]);
  float deltad = float_vect_norm(drel,2);
  float angleb1 = angleb - atan(RR/deltad);
  float angleb2 = angleb + atan(RR/deltad);

  float robot2_offset[2];
  polar2cart(robot2.vel, robot2.head, robot2_offset);

  float centre[2];
  centre[0] = robot1.pos[0] + robot2_offset[0]; 
  centre[1] = robot1.pos[1] + robot2_offset[1];
  
  if ((tcpa > 0) && (dcpa < RR)) {
    float newvel_cart[2];
    vo_resolve_by_project(robot1, angleb1, angleb2, centre, newvel_cart);
    float newvel_polar[2];
    cart2polar(newvel_cart, newvel_polar);
    robot1.head = newvel_polar[1];
    robot1.vel = min(max(newvel_polar[0], -3.0), 3.0);
  }
  else {
    robot1.vel  = robot1.oldvel;
    robot1.head = robot1.oldhead;
  }

}

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

}

void vo_periodic(void) {
  // your periodic code here.
  // freq = 10.0 Hz
  vo_simulate_loop();
  detect();
}