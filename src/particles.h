/* particles.h 
 *
 * Copyright 2025 Alan Crispin <crispinalan@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef PARTICLES_H
#define PARTICLES_H

#define MAX_PARTICLES 4096

typedef struct {
    float x, y, z;
    float vx, vy, vz;
    float life;
} Particle;

void particles_init(void);
void particles_emit_vel(float x, float y, float z,
                        float vx, float vy, float vz,
                        float intensity);
void particles_update(float dt);
void draw_particles(void);

#endif
