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
#ifndef PERCEVITE_VO_H
#define PERCEVITE_VO_H

#define RR  1.0
#define MAX_VEL 20
#define MAX_DIST 25
#define max(a,b)  ((a)>=(b)?(a):(b))
#define min(a,b)  ((a)<=(b)?(a):(b))

#define D2R (3.142/180.0)
#define R2D (180.0/3.142)


typedef struct {
  float pos[2];
  float vel;
  float head;
  float oldvel;
  float oldhead;
} robot_t;

void polar2cart(float mag, float directn, float *cart);
void cart2polar(float *vel_vec, float *vel_polar);
void calc_proj_matrix(const float *a, const float *b, float **c);
void vo_resolve_by_project(robot_t robot_a, float angle1, float angle2, float *centre, float *newvela);
void detect(void);


extern void vo_init(void);


extern void vo_evade(void);

#endif  // PERCEVITE_VO_H
