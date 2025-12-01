/* particles.c 
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
#include <GL/glut.h>
#include <stdlib.h>
#include <math.h>
#include "particles.h"

static Particle particles[MAX_PARTICLES];

void particles_init(void)
{
    for (int i = 0; i < MAX_PARTICLES; i++)
        particles[i].life = 0.0f;
}

void particles_emit_vel(float x, float y, float z,
                        float vx, float vy, float vz,
                        float intensity)
{
    if (intensity <= 0.0f) return;

    int nspawn = 6 + (int)(20.0f * intensity);
    float base_life = 0.6f + 0.6f * intensity;

    for (int n = 0; n < nspawn; n++) {

        // find a free particle slot
        int i;
        for (i = 0; i < MAX_PARTICLES; i++)
            if (particles[i].life <= 0.0f) break;

        if (i == MAX_PARTICLES) return; // no free particles

        Particle *p = &particles[i];

        p->life = base_life;

        // spawn jitter
        p->x = x + ((float)rand() / RAND_MAX - 0.5f) * 0.10f;
        p->y = y + ((float)rand() / RAND_MAX - 0.5f) * 0.10f;
        p->z = z + ((float)rand() / RAND_MAX - 0.5f) * 0.10f;

        // Velocity jitter
        float spread = 0.5f;
        float jx = ((float)rand() / RAND_MAX - 0.5f) * spread;
        float jy = ((float)rand() / RAND_MAX - 0.5f) * spread;
        float jz = ((float)rand() / RAND_MAX - 0.5f) * spread;

        p->vx = vx + jx;
        p->vy = vy + jy;
        p->vz = vz + jz;
    }
}

void particles_update(float dt)
{
    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle *p = &particles[i];
        if (p->life > 0.0f) {
            p->life -= dt;
            p->vy -= 0.5f * dt;      // gravity for effect
            p->x += p->vx * dt;
            p->y += p->vy * dt;
            p->z += p->vz * dt;
        }
    }
}

void draw_particles(void)
{
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);

    glPointSize(4.0f);
    glBegin(GL_POINTS);
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (particles[i].life > 0.0f) {
            //glColor4f(1.0f, 0.6f, 0.1f, particles[i].life);
            glColor4f(1.0f, 0.3f, 0.1f, particles[i].life);
            glVertex3f(particles[i].x, particles[i].y, particles[i].z);
        }
    }
    glEnd();

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
}
