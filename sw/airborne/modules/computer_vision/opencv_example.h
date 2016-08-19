/*
 * Copyright (C) C. De Wagter
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
/**
 * @file "modules/computer_vision/cv_opencvdemo.h"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */

#ifndef OPENCV_EXAMPLE_H
#define OPENCV_EXAMPLE_H
#include "state.h"
#ifdef __cplusplus
extern "C" {
#endif
extern float stddev_colors;
extern int mean_u;
extern int mean_v;
extern int loc_y;
extern int super_roll;
extern int too_close;
extern int16_t distance_pixels;
extern int16_t center_pixels;
int opencv_example(char *img, int width, int height);
void opencv_init_rects(void);
#ifdef __cplusplus
}
#endif

#endif

