/* lander3d.c 
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

#include "lander3d.h"
#include "particles.h"
#include <stdio.h>
#include <stdlib.h>
#include <GL/glut.h>
#include <math.h>

float cam_orbit_speed = 5.0f; // degrees per second
float cam_orbit_angle = 0.0f;
// Rotational inertia (approximate for an Apollo LM-like mass distribution)
const float I_pitch = 2500.0f;   // kg·m²   adjust after testing
const float I_roll  = 2500.0f;   // kg·m²
extern float target_x; //globals exist in main.c
extern float target_z;

//static Particle particles[MAX_PARTICLES];
//static int    first_free = 0;        /* next free slot in the array */

//=====================================================================

void init_lander3d(Lander3D *L)
{
    if (!L) return;
    // --- initial state ---
    L->pos = (Vec3){0.0f, 15000.0f, 0.0f};
    L->vel = (Vec3){0.0f, 0.0f, 0.0f};
    L->acc = (Vec3){0.0f, 0.0f, 0.0f};   
    L->pitch = 0.0f;   
    L->roll  = 0.0f;       
    L->pitch_rate = L->roll_rate = 0.0f;   
    // controls
    L->thrust_level = 0.0f;    
    L->rcs_front = L->rcs_back = L->rcs_left = L->rcs_right = 0.0f;
	L->torque_pitch = L->torque_roll = 0.0f;
    L->is_landed = false;
    L->autopilot_enabled = true;    
    //yaw
    L->yaw   = 0.0f;
    L->yaw_rate = 0.0f;    
	L->rcs_yaw_left=0.0f;  // produces positive yaw torque (ccw around +Y)
	L->rcs_yaw_right=0.0f; // produces negative yaw torque	
	L->torque_yaw=0.0f; // yaw torque storage     
    // --- mass & inertia setup ---
    L->mass = LANDER_MASS_DEFAULT; // mass
    // Approximating the main descent stage as a rectangular cuboid (width=descent_w,
    // depth=descent_d, height=descent_h). Using scaled geometry consistent with draw code.
    const float w = 4.0f;  // meters
    const float d = 4.0f;
    const float h = 2.5f;
    // moment of inertia of a cuboid about its center:
    // Ixx = (1/12) * m * (h^2 + d^2)
    // Iyy = (1/12) * m * (w^2 + d^2)
    // Izz = (1/12) * m * (w^2 + h^2)
    L->Ixx = (1.0f/12.0f) * L->mass * (h*h + d*d);
    L->Iyy = (1.0f/12.0f) * L->mass * (w*w + d*d);
    L->Izz = (1.0f/12.0f) * L->mass * (w*w + h*h);    
}

// clamp helper
static float clampf(float v, float a, float b) {
    if (v < a) return a;
    if (v > b) return b;
    return v;
}

// ================================================================
// Convert desired yaw torque → RCS yaw thruster levels
// ================================================================
void apply_yaw_thrusters_from_torque(Lander3D *L, float torque_yaw)
{
    if (!L) return;

    // Geometry: lever arm for yaw thrusters
    const float r_yaw = 1.5f;         // meters (tune if lander wider/narrower)

    // Convert requested torque → forces
    float req_F_left  = 0.0f;
    float req_F_right = 0.0f;

    if (torque_yaw > 0.0f) {
        // positive yaw torque → use LEFT yaw thruster
        req_F_left = torque_yaw / r_yaw;
    } else if (torque_yaw < 0.0f) {
        req_F_right = -torque_yaw / r_yaw;
    }

    // Convert forces → RCS levels
    L->rcs_yaw_left  = clampf(req_F_left  / RCS_THRUST_MAX, 0.0f, 1.0f);
    L->rcs_yaw_right = clampf(req_F_right / RCS_THRUST_MAX, 0.0f, 1.0f);

    // Save for telemetry
    L->torque_yaw = torque_yaw;
}



void apply_attitude_thrusters_from_torques(Lander3D *L, float desired_pitch_torque, float desired_roll_torque)
{
	// desired_pitch_torque: positive -> nose-up, negative -> nose-down (Nm)
	// desired_roll_torque : positive -> roll-right,  negative -> roll-left (Nm)
	// lever arm distances (use same geometry as update_lander3d)
	const float half_w = 2.0f;
	const float half_d = 2.0f;

    // For pitch torque: use front/back RCS (they are offset in Z)
    // positive pitch torque (nose-up): produce upward force at front
    float req_F_front = 0.0f, req_F_back = 0.0f;
    if (desired_pitch_torque > 0.0f) {
        // torque = r_z * F  => F = torque / r_z
        req_F_front = desired_pitch_torque / half_d;
    } else {
        req_F_back  = -desired_pitch_torque / half_d;
    }

    // For roll torque: use left/right RCS (they are offset in X)
    float req_F_left = 0.0f, req_F_right = 0.0f;
    if (desired_roll_torque > 0.0f) {
        // positive roll torque -> push up on right thruster (creates positive roll)
        req_F_right = desired_roll_torque / half_w;
    } else {
        req_F_left  = -desired_roll_torque / half_w;
    }

    // convert requested forces to levels (0..1) using RCS_THRUST_MAX
    float lvl_front = clampf(req_F_front / RCS_THRUST_MAX, 0.0f, 1.0f);
    float lvl_back  = clampf(req_F_back  / RCS_THRUST_MAX, 0.0f, 1.0f);
    float lvl_left  = clampf(req_F_left  / RCS_THRUST_MAX, 0.0f, 1.0f);
    float lvl_right = clampf(req_F_right / RCS_THRUST_MAX, 0.0f, 1.0f);

    // assign to lander (these will be used next physics step)
    L->rcs_front = lvl_front;
    L->rcs_back  = lvl_back;
    L->rcs_left  = lvl_left;
    L->rcs_right = lvl_right;
        
    //printf("[RCS_REQ] Ff=%.3f Fb=%.3f Fl=%.3f Fr=%.3f => lvl F=%.3f B=%.3f L=%.3f R=%.3f\n",
       //req_F_front, req_F_back, req_F_left, req_F_right,
       //lvl_front, lvl_back, lvl_left, lvl_right);
    
}

void update_lander3d(Lander3D *L, float dt)
{
    if (!L || L->is_landed) return;

    // --------------------------------------------------------
    // 1) ATTITUDE: compute torques from RCS levels -> angular accel
    // --------------------------------------------------------
    // RCS geometry (must match apply_attitude_thrusters_from_torques)
    const float half_w = 2.0f;   // x offset (m) for left/right RCS
    const float half_d = 2.0f;   // z offset (m) for front/back RCS
    const float r_yaw   = 1.5f;  // yaw lever arm for yaw RCS pair

    // Convert RCS levels (0..1) -> forces (N)
    float Ff = L->rcs_front * RCS_THRUST_MAX;   // front (nose) upward force
    float Fb = L->rcs_back  * RCS_THRUST_MAX;   // back (tail) upward force
    float Fl = L->rcs_left  * RCS_THRUST_MAX;   // left side upward force
    float Fr = L->rcs_right * RCS_THRUST_MAX;   // right side upward force

    // yaw thrusters (separate pair) -> forces
    float Fy_left  = L->rcs_yaw_left  * RCS_THRUST_MAX;
    float Fy_right = L->rcs_yaw_right * RCS_THRUST_MAX;

    // Torques from RCS (signed)
    // pitch torque: positive = nose-up (front pushes up creates positive pitch)
    float torque_pitch = (Ff - Fb) * half_d;

    // roll torque: positive = roll-right (right thruster pushes up => positive roll)
    float torque_roll  = (Fr - Fl) * half_w;

    // yaw torque: positive = rotate CCW (around +Y) ; sign convention consistent later
    float torque_yaw   = (Fy_left - Fy_right) * r_yaw;

    // store torques for telemetry/debug (optional fields on L)
    L->torque_pitch = torque_pitch;
    L->torque_roll  = torque_roll;
    L->torque_yaw   = torque_yaw;

    // Angular accelerations (τ = I * α -> α = τ / I)
    float ang_acc_pitch = torque_pitch / L->Ixx;   // pitch about X-like axis (your convention)
    float ang_acc_roll  = torque_roll  / L->Izz;   // roll about Z-like axis
    float ang_acc_yaw   = torque_yaw   / L->Iyy;   // yaw about Y axis

    // Integrate angular rates
    L->pitch_rate += ang_acc_pitch * dt;
    L->roll_rate  += ang_acc_roll  * dt;
    L->yaw_rate   += ang_acc_yaw   * dt;

    // Damping (critical to avoid runaway oscillation)
    const float ANGULAR_DAMPING = 2.0f;   // tune 0.5..5.0
    L->pitch_rate -= L->pitch_rate * ANGULAR_DAMPING * dt;
    L->roll_rate  -= L->roll_rate  * ANGULAR_DAMPING * dt;
    L->yaw_rate   -= L->yaw_rate  * ANGULAR_DAMPING * dt;

    // Integrate angles
    L->pitch += L->pitch_rate * dt;
    L->roll  += L->roll_rate  * dt;
    L->yaw   += L->yaw_rate   * dt;

    // normalize yaw into -pi..+pi for stability/readability
    if (L->yaw >  M_PI) L->yaw -= 2.0f * M_PI;
    if (L->yaw < -M_PI) L->yaw += 2.0f * M_PI;

    // clamp angles (safety)
    const float ANGLE_LIMIT = 1.5f; // ~= 86 degrees
    if (L->pitch >  ANGLE_LIMIT) L->pitch =  ANGLE_LIMIT;
    if (L->pitch < -ANGLE_LIMIT) L->pitch = -ANGLE_LIMIT;
    if (L->roll  >  ANGLE_LIMIT) L->roll  =  ANGLE_LIMIT;
    if (L->roll  < -ANGLE_LIMIT) L->roll  = -ANGLE_LIMIT;

    // --------------------------------------------------------
    // 2) TRANSLATION: main engine acceleration oriented by attitude
    // --------------------------------------------------------
    // thrust force (N) and accel (m/s^2)
    float thrust_force = ENGINE_THRUST * L->thrust_level;   // N
    float thrust_acc   = thrust_force / L->mass;            // m/s^2

    // local thrust direction (local +Y = engine vector)
    float local_thrust[3] = { 0.0f, 1.0f, 0.0f };
    float world_thrust[3];
    rotate_by_euler(local_thrust, L->roll, L->pitch, L->yaw, world_thrust);
    
    // -----------------------------------
	// MAIN ENGINE PARTICLES
	// -----------------------------------
	float engine_x = L->pos.x;
	float engine_y = L->pos.y - 2.4f;   // adjust visually
	float engine_z = L->pos.z;

	// exhaust moves opposite the thrust
	particles_emit_vel(
		engine_x, engine_y, engine_z,
		-world_thrust[0] * 8.0f,
		-world_thrust[1] * 8.0f,
		-world_thrust[2] * 8.0f,
		L->thrust_level
	);
   
	// ---------------------------------------------------------------------
	// RCS THRUSTERS PARTICLES (4 vertical for pitch/roll + 2 lateral for yaw)
	// ---------------------------------------------------------------------

	const float rcs_offset = 2.075f;      // matches draw_lander
	const float rcs_height = 1.0f;        

	// ========================================================
	// LEFT RCS  (creates roll-left → right side comes down)
	// ========================================================
	if (L->rcs_left > 0.01f)
	{
		float px = L->pos.x - rcs_offset;
		float py = L->pos.y + rcs_height;
		float pz = L->pos.z;

		// jet shoots upward visually
		particles_emit_vel(
			px, py, pz,
			0.0f, 5.0f, 0.0f,
			L->rcs_left
		);
	}

	// ========================================================
	// RIGHT RCS
	// ========================================================
	if (L->rcs_right > 0.01f)
	{
		float px = L->pos.x + rcs_offset;
		float py = L->pos.y + rcs_height;
		float pz = L->pos.z;

		particles_emit_vel(
			px, py, pz,
			0.0f, 5.0f, 0.0f,
			L->rcs_right
		);
	}

	// ========================================================
	// FRONT RCS (nose) — positive pitch torque = nose-up
	// ========================================================
	if (L->rcs_front > 0.01f)
	{
		float px = L->pos.x;
		float py = L->pos.y + rcs_height;
		float pz = L->pos.z - rcs_offset;

		particles_emit_vel(
			px, py, pz,
			0.0f, 5.0f, 0.0f,
			L->rcs_front
		);
	}

	// ========================================================
	// BACK RCS (tail)
	// ========================================================
	if (L->rcs_back > 0.01f)
	{
		float px = L->pos.x;
		float py = L->pos.y + rcs_height;
		float pz = L->pos.z + rcs_offset;

		particles_emit_vel(
			px, py, pz,
			0.0f, 5.0f, 0.0f,
			L->rcs_back
		);
	}

	// ========================================================
	// YAW LEFT  (positive yaw torque → CCW rotation)
	// Jet points +X direction
	// ========================================================
	if (L->rcs_yaw_left > 0.01f)
	{
		float px = L->pos.x;
		float py = L->pos.y + rcs_height;
		float pz = L->pos.z - rcs_offset;

		particles_emit_vel(
			px, py, pz,
			8.0f, 0.0f, 0.0f,     // sideways jet
			L->rcs_yaw_left
		);
	}

	// ========================================================
	// YAW RIGHT (negative yaw torque → CW rotation)
	// Jet points −X direction
	// ========================================================
	if (L->rcs_yaw_right > 0.01f)
	{
		float px = L->pos.x;
		float py = L->pos.y + rcs_height;
		float pz = L->pos.z + rcs_offset;

		particles_emit_vel(
			px, py, pz,
			-8.0f, 0.0f, 0.0f,
			L->rcs_yaw_right
		);
	}
		
		// world acceleration = thrust_acc * world_thrust + gravity
		L->acc.x = thrust_acc * world_thrust[0];
		L->acc.y = thrust_acc * world_thrust[1] - GRAVITY_LUNAR;
		L->acc.z = thrust_acc * world_thrust[2];

		// integrate linear velocities
		L->vel.x += L->acc.x * dt;
		L->vel.y += L->acc.y * dt;
		L->vel.z += L->acc.z * dt;

		// integrate positions
		L->pos.x += L->vel.x * dt;
		L->pos.y += L->vel.y * dt;
		L->pos.z += L->vel.z * dt;

		// --------------------------------------------------------
		// 3) GROUND COLLISION / TOUCHDOWN
		// --------------------------------------------------------
		if (L->pos.y <= 0.0f) {
			L->pos.y = 1.0f;
			L->vel.x = L->vel.y = L->vel.z = 0.0f;
			L->pitch = 0.0f;
			L->roll  = 0.0f;
			L->pitch_rate = L->roll_rate = L->yaw_rate = 0.0f;
			L->thrust_level = 0.0f;
			L->rcs_front = L->rcs_back = L->rcs_left = L->rcs_right = 0.0f;
			L->rcs_yaw_left = L->rcs_yaw_right = 0.0f;
			L->is_landed = true;
			printf("[LANDER] Touchdown detected.\n");
		}
		particles_update(dt);
	}

// ===== Autopilot (3 loops) ===
void autopilot_guided_full(Lander3D *L, float dt)
{
    if (!L || L->is_landed) return;

    // ============================================
    // 1. VERTICAL DESCENT CONTROL
    // ============================================

    const float Kp_v  = 0.0045f;      // your tuned value
    const float target_v = -3.0f;     // desired vertical speed

    float vel_error = target_v - L->vel.y;
    float thrust_cmd = L->thrust_level + Kp_v * vel_error;

    thrust_cmd = clampf(thrust_cmd, 0.0f, 1.0f);
    L->thrust_level = thrust_cmd;

    // ============================================
    // 2. ATTITUDE HOLD (pitch, roll → 0)
    // ============================================

    const float Kp_ang = 800.0f;
    const float Kd_ang = 150.0f;
    const float max_torque = 1500.0f;

    // pitch: positive → nose up
    float pitch_err = -L->pitch;               // target = 0
    float pitch_rate_err = -L->pitch_rate;

    float torque_pitch_cmd = 
        Kp_ang * pitch_err + 
        Kd_ang * pitch_rate_err;

    torque_pitch_cmd = clampf(torque_pitch_cmd, -max_torque, max_torque);

    // roll: positive → right wing down
    float roll_err = -L->roll;
    float roll_rate_err = -L->roll_rate;

    float torque_roll_cmd =
        Kp_ang * roll_err + 
        Kd_ang * roll_rate_err;

    torque_roll_cmd = clampf(torque_roll_cmd, -max_torque, max_torque);

    // ============================================
    // APPLY PITCH & ROLL TORQUES USING RCS
    // ============================================
    apply_attitude_thrusters_from_torques(L,torque_pitch_cmd,torque_roll_cmd);
    
    // ============================================================
	// 3. YAW HOLD 
	// ============================================================
	const float Kp_yaw = 200.0f;      // Nm per rad     (tune)
	const float Kd_yaw = 50.0f;       // Nm per rad/s   (tune)
	const float max_torque_yaw = 500.0f;

	// Target yaw = 0 for now (later: face landing site)
	float yaw_err      = -L->yaw;
	float yaw_rate_err = -L->yaw_rate;

	float torque_yaw_cmd =
		Kp_yaw * yaw_err +
		Kd_yaw * yaw_rate_err;

	torque_yaw_cmd = clampf(torque_yaw_cmd,	-max_torque_yaw, max_torque_yaw);

	// Apply yaw RCS
	apply_yaw_thrusters_from_torque(L, torque_yaw_cmd);
   
    
    // ============================================
    // 3. Touchdown
    // ============================================
    
    // Safety: if thrust-level is very small and we are practically landed -> zero RCS too
    if (L->pos.y <= 1.0f && fabsf(L->vel.y) < 0.5f) {
        // Touchdown
        L->pos.y = 1.0f; // rest just above terrain
        L->vel.y = 0.0f;
        L->thrust_level = 0.0f;
        // zero attitude actuators
        L->rcs_front = L->rcs_back = L->rcs_left = L->rcs_right = 0.0f;
        L->torque_pitch = L->torque_roll = 0.0f;
        L->is_landed = true;
        printf("[AUTOPILOT_FULL] Touchdown at y=%.2f m\n", L->pos.y);
    }

    // console telemetry 
    //static float tel_timer = 0.0f;
    //tel_timer += dt;
    //if (tel_timer >= 1.0f) {
		//tel_timer = 0.0f;
		//printf("[AUTOPILOT] ALT:%.1f VY:%.2f THR:%.3f  PITCH:%.1f° ROLL:%.1f°\n",
		//L->pos.y, L->vel.y, L->thrust_level,
		//L->pitch * 180.0f / M_PI, L->roll * 180.0f / M_PI);
		//printf("[AUTOPILOT] PITCH/ROLL RCS F:%.2f B:%.2f L:%.2f R:%.2f  torqP:%.1f torqR:%.1f\n",
		//L->rcs_front, L->rcs_back, L->rcs_left, L->rcs_right,
		//L->torque_pitch, L->torque_roll);
		//printf("[AUTOPILOT] YAW:%.1f° YR:%.2f TYaw:%.1f R_L:%.2f R_R:%.2f\n",
		//L->yaw * 180.0f / M_PI, L->yaw_rate,
		//L->torque_yaw, L->rcs_yaw_left, L->rcs_yaw_right);
    //}    
}

// ===================================================================
// ATTITUDE STABILITY TEST HARNESS
// Injects a one-time disturbance at t=0
// =====================================================================

void autopilot_test_harness(Lander3D *L, float dt)
{
    static bool injected = false;

    if (!L || L->is_landed) return;

    // Inject disturbance ONCE at t=0
    if (!injected) {
        L->pitch += 5.0f * M_PI / 180.0f;   // +5° pitch disturbance
        L->roll  += -3.0f * M_PI / 180.0f;  // -3° roll disturbance
        L->yaw += 10.0f * M_PI/180.0f;		// +10° yaw disturbance
        printf("[TEST] Injected +5° pitch, -3° roll disturbance, +10° yaw disturbance.\n");
        injected = true;
    }
}


//now uses zoom factor
void update_camera(Lander3D *L, float alpha, int cam_mode, float cam_distance, float cam_zoom)
{
    if (!L) return;

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // --- Base camera geometry (metres) tuned for a ~4 m lander ---
    const float cam_back_base = 60.0f;   // baseline chase/back distance
    const float cam_up_base   = 25.0f;   // baseline camera height above craft

    // ---------- Zoom handling ----------
    // cam_zoom > 1 -> closer view (zoom in)
    // cam_zoom < 1 -> farther view (zoom out)
    // We implement zoom by scaling the camera offsets (distance & height).
    // Use division so increasing cam_zoom moves camera closer:
    //   cam_back = cam_back_base * cam_distance / cam_zoom
    // This keeps world geometry unchanged (no glScalef).
    if (cam_zoom <= 0.0f) cam_zoom = 1.0f; // safety
    float cam_back = (cam_back_base * cam_distance) / cam_zoom;
    float cam_up   = (cam_up_base   * cam_distance) / cam_zoom;

    switch (cam_mode)
    {
        case 0: // ORBIT
        {
            static float orbit_angle = 0.0f;
            orbit_angle += 0.002f; // slow rotation (radians per frame step)

            float radius = cam_back * 2.0f;
            float cx = L->pos.x + radius * sinf(orbit_angle);
            float cz = L->pos.z + radius * cosf(orbit_angle);
            float cy = L->pos.y + cam_up;

            // Standard Y-up orbit camera
            gluLookAt(cx, cy, cz,
                      L->pos.x, L->pos.y, L->pos.z,
                      0.0f, 1.0f, 0.0f);
        } break;

        case 1: // OVERHEAD (top-down)
        {
            // Put camera directly over craft and look straight down.
            // NOTE: when view vector is nearly vertical, the gluLookAt "up" vector
            // must NOT be parallel to view direction. Use (0,0,-1) so screen up
            // corresponds to -Z world (gives stable orientation).
            float cx = L->pos.x;
            float cz = L->pos.z;
            float cy = L->pos.y + cam_up * 6.0f; // higher overhead for coverage

            gluLookAt(cx, cy, cz,
                      L->pos.x, L->pos.y, L->pos.z,
                      0.0f, 0.0f, -1.0f);
        } break;

        default: // CHASE
        {
            float yaw = L->yaw;
            // Put camera behind the craft in its local -Z direction (world-space)
            float cx = L->pos.x + cam_back * sinf(yaw);
            float cz = L->pos.z + cam_back * cosf(yaw);
            float cy = L->pos.y + cam_up;

            gluLookAt(cx, cy, cz,
                      L->pos.x, L->pos.y, L->pos.z,
                      0.0f, 1.0f, 0.0f);
        } break;
    } // switch
}



// ===== Initialise Lighting ===========
void init_lighting(void)
{
    GLfloat light_pos[]  = { 0.3f, 1.0f, 0.4f, 0.0f };  // directional light from above-right
    GLfloat ambient[]    = { 0.1f, 0.1f, 0.1f, 1.0f };
    GLfloat diffuse[]    = { 0.9f, 0.9f, 0.85f, 1.0f };
    GLfloat specular[]   = { 0.5f, 0.5f, 0.5f, 1.0f };

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);

    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
}


// ===== Draw Sky ===========
void draw_sky()
{
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(-1,1,-1,1,-1,1);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glBegin(GL_QUADS);        
        glColor3f(0.5f,0.4f,0.2f); //bottom sky colour               
        glVertex2f(-1,-1); glVertex2f(1,-1);       
        glColor3f(0.2f,0.4f,0.5f);   // top sky colour    
        glVertex2f(1,1); glVertex2f(-1,1);
    glEnd();
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
}

// === compute terrian height ===========
float terrain_height(float x, float z)
{
    // small amplitude terrain 
    return 0.6f * sinf(0.2f * x) * cosf(0.2f * z);
}

// ===============================================================
// Compute surface normal (for lighting) - standard gradient method
// ===============================================================
void terrain_normal(float x, float z, float *nx, float *ny, float *nz)
{
    float eps = 0.5f; // central difference step (bigger gives smoother normals)
    float hL = terrain_height(x - eps, z);
    float hR = terrain_height(x + eps, z);
    float hD = terrain_height(x, z - eps);
    float hU = terrain_height(x, z + eps);

    // partial derivatives
    float dhdx = (hR - hL) / (2.0f * eps);
    float dhdz = (hU - hD) / (2.0f * eps);

    // normal is (-dhdx, 1, -dhdz)
    *nx = -dhdx;
    *ny = 1.0f;
    *nz = -dhdz;

    // normalize
    float len = sqrtf((*nx)*(*nx) + (*ny)*(*ny) + (*nz)*(*nz));
    if (len > 1e-6f) {
        *nx /= len; *ny /= len; *nz /= len;
    } else {
        *nx = 0.0f; *ny = 1.0f; *nz = 0.0f;
    }
}

// ==============================================================
// draw_terrain: filled triangle grid using triangle strips
// =============================================================
void draw_terrain(void)
{
    // extent and resolution
    const float SIZE = 20000.0f;  // half-extent in x and z (so total 40km x 40km)
    const float STEP = 200.0f;    // spacing in metres - tune for performance/quality

    const int N = (int)(SIZE / STEP);
    const float start = -N * STEP;
    //const float end   =  N * STEP;

    // lighting / material
    glColor3f(0.55f, 0.55f, 0.55f);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0f, 1.0f);   // push terrain slightly back to reduce z-fighting
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    // draw triangle strips row by row (x direction)
    for (int ix = 0; ix < 2*N; ++ix) {
        float x = start + ix * STEP;
        float x2 = x + STEP;

        glBegin(GL_TRIANGLE_STRIP);
        for (int iz = 0; iz <= 2*N; ++iz) {
            float z = start + iz * STEP;

            // vertex at (x2, z)
            float nx, ny, nz;
            terrain_normal(x2, z, &nx, &ny, &nz);
            glNormal3f(nx, ny, nz);
            glVertex3f(x2, terrain_height(x2, z), z);

            // vertex at (x, z)
            terrain_normal(x, z, &nx, &ny, &nz);
            glNormal3f(nx, ny, nz);
            glVertex3f(x, terrain_height(x, z), z);
        }
        glEnd();
    }

    // cleanup
    glDisable(GL_POLYGON_OFFSET_FILL);
    glDisable(GL_CULL_FACE);
}

//==== Draw Lander Craft =================
void draw_lander(Lander3D *L)
{
    if (!L) return;

    glPushMatrix();
    glTranslatef(L->pos.x, L->pos.y, L->pos.z);
    glRotatef(L->yaw  * 180.0f / M_PI, 0, 1, 0);
    glRotatef(L->pitch* 180.0f / M_PI, 1, 0, 0);
    glRotatef(L->roll * 180.0f / M_PI, 0, 0, 1);

    // ------------------------------------------------------------
    // DIMENSIONS (approximate Apollo LM proportions)
    // ------------------------------------------------------------
    const float descent_w = 4.3f;   // width (meters)
    const float descent_h = 2.0f;   // height
    const float ascent_w  = 2.5f;   // width
    const float ascent_h  = 2.0f;   // height

    // ------------------------------------------------------------
    // DESCENT STAGE (main lower box)
    // ------------------------------------------------------------
    glPushMatrix();
        //glColor3f(1.0f, 0.5f, 0.2f);
        glColor3f(0.75f, 0.75f, 0.78f);
        glScalef(descent_w, descent_h, descent_w);
        glutSolidCube(1.0f);
    glPopMatrix();

    // ------------------------------------------------------------
    // ASCENT STAGE (smaller upper box)
    // ------------------------------------------------------------
    glPushMatrix();
        glTranslatef(0.0f, (descent_h + ascent_h) * 0.5f, 0.0f);
        glColor3f(0.85f, 0.82f, 0.75f);
        //glColor3f(0.2f, 1.0f, 0.5f);
        glScalef(ascent_w, ascent_h, ascent_w * 1.2f);
        glutSolidCube(1.0f);
    glPopMatrix();

    // ------------------------------------------------------------
    // MAIN ENGINE (cone)
    // ------------------------------------------------------------
    const float thruster_h = 1.0f;
    const float thruster_r = 0.6f;
    glPushMatrix();
        glTranslatef(0.0f, -descent_h * 0.6f, 0.0f);
        glRotatef(-90.0f, 1, 0, 0);
        glColor3f(0.6f, 0.6f, 0.65f);
        glutSolidCone(thruster_r, thruster_h, 16, 4);
    glPopMatrix();
	
	
    // ------------------------------------------------------------
    // LANDING LEGS (lines at corners, ~20° angle)
    // ------------------------------------------------------------
    const float leg_len = 2.0f;
    const float leg_angle = 20.0f * M_PI / 180.0f;
    const float base_offset = descent_w * 0.5f;

    glColor3f(0.9f, 0.75f, 0.2f);
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    for (int i = 0; i < 4; ++i)
    {
        float angle = i * M_PI_2 + M_PI_4;
        float x = base_offset * cosf(angle);
        float z = base_offset * sinf(angle);

        float dx = leg_len * sinf(leg_angle) * cosf(angle);
        float dz = leg_len * sinf(leg_angle) * sinf(angle);
        float dy = -leg_len * cosf(leg_angle);

        glVertex3f(x, -descent_h * 0.5f, z);
        glVertex3f(x + dx, -descent_h * 0.5f + dy, z + dz);
    }
    glEnd();

    // ------------------------------------------------------------
    // Attitude side thrusters (small cubes)
    // ------------------------------------------------------------
     
    const float rcs_offset = ascent_w * 0.75f + 0.2f;  // pushed slightly outward
	const float rcs_height = (descent_h + ascent_h) * 0.25f;
	const float rcs_size = 0.25f;
	glColor3f(0.8f, 0.8f, 0.85f);

	// ±X
	for (int dir = -1; dir <= 1; dir += 2) {
		glPushMatrix();
			glTranslatef(dir * rcs_offset, rcs_height, 0.0f);
			glutSolidCube(rcs_size);
		glPopMatrix();
	}
	// ±Z
	for (int dir = -1; dir <= 1; dir += 2) {
		glPushMatrix();
			glTranslatef(0.0f, rcs_height, dir * rcs_offset);
			glutSolidCube(rcs_size);
		glPopMatrix();
	}   
   
    glPopMatrix();
    
    // =============================================================
	// YAW RCS (sideways pointing thrusters)
	// =============================================================
	glColor3f(0.9f, 0.9f, 0.95f);

	// yaw-left thruster (front-left side)
	glPushMatrix();
	glTranslatef(0.0f, rcs_height, -rcs_offset);
	glRotatef(90, 0,1,0);  // rotate 90° so jet points sideways
	glutSolidCube(rcs_size);
	glPopMatrix();

	// yaw-right thruster (back-right side)
	glPushMatrix();
	glTranslatef(0.0f, rcs_height, rcs_offset);
	glRotatef(90, 0,1,0);
	glutSolidCube(rcs_size);
	glPopMatrix();
    
}

//==== Landing target ===========
void draw_target_marker(float target_x, float target_z)
{
    const float radius = 10.0f;   // circle radius 10m
    const int segments = 36;
	float y = 10.0f;  // raised above terrain 
    glColor3f(0.0f, 1.0f, 0.0f);     // green
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < segments; ++i) {
        float theta = 2.0f * M_PI * (float)i / (float)segments;
        glVertex3f(target_x + radius * cosf(theta), y, target_z + radius * sinf(theta));
    }
    glEnd();
    
}

// ==== Draw Text for HUD ===========
void draw_text(int x, int y, const char *text) {
    if (!text) return;

    // Save state we will modify
    GLboolean depthWasEnabled = glIsEnabled(GL_DEPTH_TEST);
    GLboolean lightingWasEnabled = glIsEnabled(GL_LIGHTING);

    // Setup orthographic projection for 2D overlay
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, WIN_W, 0, WIN_H);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    if (depthWasEnabled) glDisable(GL_DEPTH_TEST);
    if (lightingWasEnabled) glDisable(GL_LIGHTING);

    //glColor3f(1.0f, 0.0f, 0.0f); // red text
    glColor3f(0.0f, 0.0f, 0.0f); //black text
    glRasterPos2i(x, y);
	for (const char *c = text; *c != '\0'; c++) {        
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c); 
     }

    // Restore states
    if (lightingWasEnabled) glEnable(GL_LIGHTING);
    if (depthWasEnabled) glEnable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}


// ======= Rotate by Euler ===========
// Lander orientation uses intrinsic Euler rotations in the order:
// R = Ry(yaw) * Rx(pitch) * Rz(roll)
// Applies that rotation to vector v -> out
void rotate_by_euler(const float v[3], float roll, float pitch, float yaw, float out[3])
{
    // Step 1: v1 = Rz(roll) * v
    float cr = cosf(roll), sr = sinf(roll);
    float v1x =  cr * v[0] - sr * v[1];
    float v1y =  sr * v[0] + cr * v[1];
    float v1z =  v[2];

    // Step 2: v2 = Rx(pitch) * v1
    float cp = cosf(pitch), sp = sinf(pitch);
    float v2x = v1x;
    float v2y =  cp * v1y - sp * v1z;
    float v2z =  sp * v1y + cp * v1z;

    // Step 3: out = Ry(yaw) * v2
    float cy = cosf(yaw), sy = sinf(yaw);
    out[0] =  cy * v2x + sy * v2z;
    out[1] =  v2y;
    out[2] = -sy * v2x + cy * v2z;
}

