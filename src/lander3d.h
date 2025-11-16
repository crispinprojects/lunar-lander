/* lander3d.h 
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

#ifndef LANDER3D_H
#define LANDER3D_H

#include <stdbool.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <math.h>

// ----------------------------------------------------
// Constants
// ----------------------------------------------------
#define GRAVITY_LUNAR   1.62f        // m/s^2
#define WIN_W           1280
#define WIN_H           720

#define LANDER_MASS_DEFAULT   1500.0f   // kg (choose appropriate mass)
#define ENGINE_THRUST         22000.0f  // N (main engine max thrust) — example
#define RCS_THRUST_MAX        2000.0f   // N (max vertical RCS thrust per thruster)

// ----------------------------------------------------
// 3D vector struct
// ----------------------------------------------------
typedef struct {
    float x, y, z;
} Vec3;

// Utility vector ops 
static inline Vec3 vec3_add(Vec3 a, Vec3 b) { return (Vec3){a.x+b.x, a.y+b.y, a.z+b.z}; }
static inline Vec3 vec3_scale(Vec3 a, float s) { return (Vec3){a.x*s, a.y*s, a.z*s}; }

//==== Lander Structure ========
typedef struct {
    Vec3 pos;
    Vec3 vel;
    Vec3 acc;
    float pitch, roll, yaw;          // radians
    float pitch_rate, roll_rate,yaw_rate;     // rad/s 
    float mass; //mass     
	float Ixx, Iyy, Izz;   // principal moments of inertia (kg·m^2) 
    // Main engine
    float thrust_level;              // 0..1
    // RCS (reaction control system) thruster levels
    float rcs_front;                 // pitch down
    float rcs_back;                  // pitch up
    float rcs_left;                  // roll right
    float rcs_right;                 // roll left
    // Torques computed from RCS thrusters
    float torque_pitch;              // about X-axis (pitch)
    float torque_roll;               // about Z-axis (roll)  
    float torque_yaw; 				// about Y -axis (yaw)        
	// RCS yaw thruster levels
	float rcs_yaw_left;  // produces positive yaw torque (ccw around +Y)
	float rcs_yaw_right; // produces negative yaw torque	
    bool is_landed;
    bool autopilot_enabled;
} Lander3D;


// ==== Function Prototypes======

// --- Core simulation ---
void init_lander3d(Lander3D *L);
void update_lander3d(Lander3D *L, float dt);
// -----Autopilot ---
void autopilot_guided_full(Lander3D *L, float dt);
void autopilot_test_harness(Lander3D *L, float dt);
void apply_attitude_thrusters_from_torques(Lander3D *L, float torque_pitch, float torque_roll);
void apply_yaw_thrusters_from_torque(Lander3D *L, float torque_yaw);
// --- Rendering ---
void draw_lander(Lander3D *L);
void draw_sky(void);
void draw_terrain(void);
void draw_target_marker(float tx, float tz);
void draw_text(int x, int y, const char *text);
void init_lighting();
// --- Camera ---
void update_camera(Lander3D *L, float alpha, int cam_mode, float cam_distance);
// --- Math helper (rotation) ---
void rotate_by_euler(const float v[3], float roll, float pitch, float yaw, float out[3]);

#endif // LANDER3D_H
