/* main.c
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
#include <stdio.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include "lander3d.h"

Lander3D lander;
float target_x = 0.0f, target_y = 0.0f, target_z = 0.0f;

// === Camera globals ===
int cam_mode = 0;         // 3 = chase, 0 = orbit
float cam_distance = 1.0f; // default zoom factor (1.0 = normal)

// === Telemetry ===
static FILE *telemetry_fp = NULL;
static double sim_time = 0.0;


// ===Telemetry handling ===

void telemetry_open(void)
{
    telemetry_fp = fopen("telemetry.csv", "w");
    if (telemetry_fp) {
        fprintf(telemetry_fp, "time,altitude,vy,thrust,pitch,roll,is_landed\n");
        fflush(telemetry_fp);
        printf("[TELEM] Logging to telemetry.csv\n");
    } else {
        perror("[TELEM] Failed to open telemetry.csv");
    }
}

void telemetry_log(const Lander3D *L)
{
    if (!telemetry_fp || !L) return;
    fprintf(telemetry_fp, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d\n",
            sim_time, L->pos.y, L->vel.y, L->thrust_level,
            L->pitch, L->roll, L->is_landed ? 1 : 0);
    fflush(telemetry_fp);
}

void telemetry_close(void)
{
    if (telemetry_fp) {
        fclose(telemetry_fp);
        telemetry_fp = NULL;
        printf("[TELEM] Telemetry file closed.\n");
    }
}

// ==== Timing ====
static double now_seconds(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// ==== Physics loop constants ====
#define PHYS_DT   0.01      // 10 ms timestep
#define MAX_FRAME 0.25      // clamp 250 ms

static double accumulator = 0.0;
static double t_prev = 0.0;


// === Restart Simulation (R key) ===
void restart_simulation(void)
{
    printf("[SYSTEM] Restarting simulation...\n");
    telemetry_close();
    init_lander3d(&lander);
    sim_time = 0.0;
    accumulator = 0.0;
    t_prev = now_seconds();
    telemetry_open();
}

// === Keyboard input ===
void key_callback(unsigned char key, int x, int y)
{
    switch (key)
    {
        case 27: // ESC
            printf("[SYSTEM] Exiting simulation.\n");
            exit(0);
            break;
        case 'r':
        case 'R':
            restart_simulation();
            break;
        case 'c':
        case 'C':
            cam_mode = (cam_mode + 1) % 3;  // cycle 0–2
            {
                const char *modestr[] = {"ORBIT", "OVERHEAD", "CHASE"};
                printf("Camera mode: %s\n", modestr[cam_mode]);
            }
            break;
        case 73: // PAGEUP (ASCII 73 for GLUT special key simulation)
        case 105: // lowercase 'i' alternative if PageUp not captured
            cam_distance -= 0.1f;
            if (cam_distance < 0.3f) cam_distance = 0.3f;
            printf("Camera distance: %.2f\n", cam_distance);
            break;
        case 81: // PAGEDOWN (ASCII 81)
        case 107: // lowercase 'k' fallback
            cam_distance += 0.1f;
            if (cam_distance > 3.0f) cam_distance = 3.0f;
            printf("Camera distance: %.2f\n", cam_distance);
            break;
        default:
            break;
    }
}

// === Render and physics loop ===
void renderScene(void)
{
    double t_now = now_seconds();
    double frame_dt = t_now - t_prev;
    if (frame_dt > MAX_FRAME) frame_dt = MAX_FRAME;
    t_prev = t_now;
    accumulator += frame_dt;

    // ---- Fixed-step physics ----   
    while (accumulator >= PHYS_DT) {   
    autopilot_test_harness(&lander, (float)PHYS_DT); 
    autopilot_guided_full(&lander, (float)PHYS_DT); 
    update_lander3d(&lander, (float)PHYS_DT);
    sim_time += PHYS_DT;
    telemetry_log(&lander);
    accumulator -= PHYS_DT;
	}	

    // ---- Rendering ----
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();  
    update_camera(&lander, (float)(accumulator / PHYS_DT),cam_mode,cam_distance);
    draw_sky();
    draw_terrain();   
    draw_lander(&lander);
    draw_target_marker(target_x, target_z);
    // ---- HUD ----
    char hud[256];
    snprintf(hud, sizeof(hud),
             "ALT: %.1f  VY: %.2f  THR: %.2f  PITCH: %.1f  ROLL: %.1f YAW: %.1f STATUS: %s",
             lander.pos.y, lander.vel.y, lander.thrust_level * 100.0f,
             lander.pitch * 180.0f / M_PI, lander.roll * 180.0f / M_PI, lander.yaw * 180.0f / M_PI,
             lander.is_landed ? "LANDED" : "FLYING");
    glDisable(GL_LIGHTING);
	draw_text(10, 20, hud);
	glEnable(GL_LIGHTING);   
    glutSwapBuffers();
    glutPostRedisplay();
}

// ---------------------------------------------------------
// Initialization
// ---------------------------------------------------------
int main(int argc, char** argv)
{
    //printf("Starting 3D lunar lander simulation\n");
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1280, 720);
    glutCreateWindow("3D Lunar Lander Simulation");
    // --- GL Setup ---
    glClearColor(0.02f, 0.02f, 0.05f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);    
    glCullFace(GL_BACK);  
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0f, 1.0f);  
    // --- Projection ---
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //gluPerspective(60.0, 16.0 / 9.0, 1.0, 50000.0); //old
    gluPerspective(60.0, 16.0/9.0, 0.1, 200000.0); //new
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();   
    // --- Simulation ---
    init_lander3d(&lander);
    t_prev = now_seconds();
    init_lighting();    
	atexit(telemetry_close);
	telemetry_open();
    // --- Register callbacks ---
    glutDisplayFunc(renderScene);
    glutKeyboardFunc(key_callback);
    glutMainLoop();
    return 0;
}
