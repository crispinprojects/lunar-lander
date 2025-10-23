/**
 * main.c
 * Lunar Lander simulation — v1.0.0
 *
 * Author: Alan Crispin (crispinalan)
 * License: GPL-3.0 
 *
 * This file provides a self-contained 2D lunar lander simulation and autopilot (AutoPilot_Update).
 * The autopilot code uses named global constants at the top the file making it easier to experiment
 * with tuning constants
 */

#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include <string.h>
//#include <stdio.h>

#define HAZARD_CLEAR     0
#define HAZARD_WARNING   1
#define HAZARD_DANGER    2

typedef struct {
    int status;
    float dist_to_hazard;
    float scan_angle;
} HazardResult;


#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600

// World
#define WORLD_WIDTH 8000.0f
#define SURFACE_Y 520.0f
#define CEILING_Y 20.0f

// Physics/time
const double TIME_SLICE = 1.0 / 60.0;
const double SIM_SPEED = 1.0;
const int MAX_PHYS_STEPS = 5;

// ============================================================================
//  Global Constants & Tuning Parameters
// ============================================================================

// ---------------------------
// Physical environment
// ---------------------------
const float GRAVITY            = 1.62f;      // lunar gravity acceleration (m/s²)
const float MAIN_THRUST_ACCEL  = 30.0f;      // maximum upward acceleration at full main throttle (m/s²)
const float SIDE_THRUST_ACCEL  = 2.5f;       // maximum horizontal acceleration (side jets) m/s² 
const float FUEL_UNIT_MASS_KG  = 0.25f;      // mass per unit of onboard fuel
const float DRY_MASS_KG        = 1200.0f;    // lander’s dry mass (no fuel)
const float NOMINAL_MASS_KG    = 1500.0f;    // reference mass for thrust normalization
const float MIN_MASS_KG        = 800.0f;     // prevents instability when fuel is nearly empty (lower safety bound)

// ---------------------------
// Vertical descent tuning
// ---------------------------

// Vertical descent target profiles
#define ALT_VY_HIGH1           300.0f  //first stage height (stage 1)
#define ALT_VY_HIGH2           150.0f  //second stage height (stage 2)
#define ALT_VY_HIGH3            80.0f  //third stage height (stage 3)
#define ALT_VY_HIGH4            30.0f  //fourth stage height (stage 4)
#define VY_TARGET1               4.0f  //desired descent velocity (m/s) for stage 1 (higher target_vy for higher altitude: efficient glide)
#define VY_TARGET2               3.0f  //desired descent velocity (m/s) for stage 2
#define VY_TARGET3               2.0f  //desired descent velocity (m/s) for stage 3
#define VY_TARGET4               1.0f  //desired descent velocity (m/s) for stage 4
#define VY_TARGET_FINAL          0.55f //desired descent velocity (m/s) for landing


// === Autopilot Global Constants ===

// Target acquisition
#define ALT_TARGET_LOCK        	200.0f
#define SCAN_DISTANCE           300.0f
#define SCAN_ANGLE_DEG           60.0f
#define SCAN_RAYS                11
#define SCAN_MODE                 1

// Lateral PD control (horizontal stability)
#define KP_LAT                  0.0025f //position gain (how strongly the craft turns toward the target landing site)
#define KD_LAT                  0.06f  //velocity damping (counteracts overshoot or drift)
#define SIDE_MAX                0.36f  //max allowable side thrust (fraction of full power)
#define SIDE_BLEND_A            0.85f
#define SIDE_BLEND_B            0.15f

// Vertical PD gain
// higher value increases throttle aggressiveness
// lower value  smooths but may cause drift from target rate
#define KV_VY                   0.27f

// Landing flare
#define FLARE_ALT               25.0f //height (m) where soft-landing flare begins
#define FLARE_BASE               1.10f
#define FLARE_GRADIENT          -0.30f


// Anti-hover bleed
#define BLEED_ALT_MIN            2.0f //hover height (m) 
#define BLEED_VY_MAX             0.2f
#define BLEED_DELTA_THR          0.03f //small downward bias if hovering too long

// Pitch damping
#define KP_THETA                0.25f //proportional gain to return craft to upright
#define KD_THETA                0.40f //damping term to resist oscillation (angular drag)
#define PITCH_BLEND             0.15f
#define PITCH_LIMIT_MAX         (M_PI/6.0f) //maximum pitch angle allowed (30 degrees)

// Landing criteria
#define ALT_TOUCHDOWN            1.9f   //switch off engines below 1.9m   
#define LAND_VY_MAX              1.5f  //safe vertical velocity threshold for landing
#define LAND_VX_MAX              2.5f  //safe horizontal velocity threshold for landing


//// Camera tuning
const float CAMERA_FOLLOW_RATE = 6.0f;
const float CAMERA_LEAD_PIXELS = (float)WINDOW_WIDTH / 6.0f;
const float CAMERA_VERTICAL_BIAS = 100.0f;

//// Roll / attitude tuning
const float ROLL_LEVER_ARM = 0.6f;     // m (effective distance from center-line to thruster force line)
const float ROLL_KP = 1.2f;            // P gain (Nm/rad) — start moderate
const float ROLL_KD = 0.20f;           // D gain (Nm⋅s/rad)
const float ROLL_MAX_THR = 0.6f;       // max side thruster command used for roll control
const float ROLL_ANGLE_TOL = 0.05f;    // radians (~2.9°): alignment tolerance
const float ROLL_RATE_TOL = 0.12f;     // rad/s: rate tolerance

////Fuel
#define FUEL_FLOW_MAIN       0.4f        // fuel units per sec at throttle = 1.0
#define FUEL_FLOW_SIDE       0.05f       // fuel units per sec at side_thrust = 1.0

////Thrust
#define MAX_LANDING_VY       4.0f        // safe vertical velocity
#define MAX_LANDING_VX       2.0f        // safe horizontal velocity

// Debug and bleep
#define DEBUG_FRAME_INTERVAL    120 //number of frames between telemetry printouts e.g. 60
#define BLEEP_INTERVAL			1200 //number of frames between audio bleeps 60 x10
bool is_bleeping =true;				//bleeping to indicate landing in progress
//----------------------------------------------------------------------
// Structures
//----------------------------------------------------------------------

typedef struct {
    float x, y;
    float vx, vy;
    float thrust_level;        // 0.0–1.0 main engine (used by physics)
    float side_thrust_level;   // -1.0 left, +1.0 right
    float fuel;                // abstract fuel units (not kg) - preserved as you requested
    float angle;    
    float dry_mass;            // dry mass (kg) of the lander **without** fuel     
    float theta;      // orientation radians (0 = upright engines down)
	float omega;      // angular velocity radians/sec    
    bool thrust;  
    bool is_landed;
    bool is_crashed;
    bool is_hovering;
    bool hazard_ahead;  
    bool left_thruster;
    bool right_thruster;
} Lander;

typedef struct {
    float x;        // world x of site centre
    float y;        // surface y at site centre (optional)
    float score;    // 0..1 fraction clear
    float width;    // effective clear width (m)
    double timestamp;
} CandidateSite;

// AutopilotState (add)
typedef struct {
	bool enabled;
    int stage;
    float target_theta; // desired attitude for roll-align stage (radians)
    float theta_cmd;
    double burn_timer;
    double last_throttle; 
    float target_x;             // selected landing site world x (NaN if none)
    float target_y;             // optional: expected surface y at target (for display)
    float target_score;         // safety score (0..1) of selected site
    float stage_timer;
    bool has_target;            // true when a target is locked    
    double scan_last_time;      // last time we did full predictive scan   
    int site_mem_count;  // site memory (ring buffer)
    int site_mem_head;
    CandidateSite site_mem[16]; // small ring buffer of previously-scored sites
} AutopilotState;


typedef struct {
	float x; 
	float y; 
} Camera;

typedef struct {
	float x;
	float y; 
	float w; 
	float h; 
} Hazard;

//----------------------------------------------------------------------
//Declarations
//----------------------------------------------------------------------
void PredictiveScan(const Lander *l, const Hazard *hazards, int hazard_count,
                    AutopilotState *ap, float scan_ahead, float grid_spacing,
                    int grid_cols, int grid_rows);

CandidateSite ScoreSite(const Lander *l, const Hazard *hazards, int hazard_count,
                        float centre_x, float centre_y, float site_width, float site_depth);

void AddSiteMemory(AutopilotState *ap, const CandidateSite *site);
CandidateSite SelectBestSite(const AutopilotState *ap, const Lander *l);

float ComputeLandingZoneSafetyAtX(const Lander* l,
                                 const Hazard* hazards, int hazardCount,
                                 float x_center,
                                 float site_width,
                                 float site_depth);


//======================================================================
//  world to screen 
static inline void WorldToScreen(const Camera* cam, float wx, float wy, SDL_FRect* out) {
    out->x = wx - (cam->x - (float)WINDOW_WIDTH / 2.0f);
    out->y = wy - (cam->y - (float)WINDOW_HEIGHT / 2.0f);
    out->w = out->h = 0.0f;
}

//======================================================================
void InitLander(Lander* l) {   
    l->x = 2000.0f;    
    //l->y = SURFACE_Y - 200.0f;
    l->y = 100.0f; 
    l->vx =10.0f; //try vx =20m/s faster entry to moon atmosphere
    l->vy = 0.0f;
    l->fuel = 300.0f;           // abstract units (mass calculations)
    l->dry_mass = DRY_MASS_KG;   // fuel units
    //l->theta = 0.0f;    // upright
    l->theta = 16.0f * M_PI / 180.0f;  //16 degree on entry to moon atmosphere
	l->omega = 0.0f;	 
    // visual flags
    l->is_landed = false;
    l->is_crashed = false;
    l->is_hovering = false;
    l->hazard_ahead = false;  
    l->thrust = false;
    l->left_thruster = false;
    l->right_thruster = false;
}


//======================================================================

static bool RectsOverlap(float ax, float ay, float aw, float ah,
                         float bx, float by, float bw, float bh) {
    return (ax < bx + bw) && (ax + aw > bx) && (ay < by + bh) && (ay + ah > by);
}

//======================================================================

// Compute roll PD torque and map to side_thrust_level [-ROLL_MAX_THR..ROLL_MAX_THR]
// target_theta in radians, positive = clockwise rotation desired
void ComputeRollControl(Lander *l, AutopilotState *ap, float target_theta, float dt)
{
    if (!l || dt <= 0.0f) return;

    // normalize shortest-angle error (-pi..pi)
    float err = target_theta - l->theta;
    while (err > M_PI) err -= 2.0f * M_PI;
    while (err < -M_PI) err += 2.0f * M_PI;

    // PD torque demand
    float torque_cmd = ROLL_KP * err + ROLL_KD * (-l->omega);

    // map torque -> side_thrust_level:
    // torque = force * lever_arm; force = side_thrust_level * SIDE_THRUST_ACCEL * mass_scale * mass_kg
    float mass_kg = fmaxf(MIN_MASS_KG, l->dry_mass + l->fuel * FUEL_UNIT_MASS_KG);
    float mass_scale = NOMINAL_MASS_KG / mass_kg;
    float denom = ROLL_LEVER_ARM * SIDE_THRUST_ACCEL * mass_scale * mass_kg;
    float side_cmd = 0.0f;
    if (fabsf(denom) > 1e-6f) side_cmd = torque_cmd / denom;

    // clamp to allowable side thruster authority
    if (side_cmd > ROLL_MAX_THR) side_cmd = ROLL_MAX_THR;
    if (side_cmd < -ROLL_MAX_THR) side_cmd = -ROLL_MAX_THR;

    // set side thrust (note: this will also apply to lateral control if called elsewhere)
    l->side_thrust_level = side_cmd;
}

//======================================================================
void StepLander(Lander *lander, double dt)
{
    if (!lander) return;
    if (dt <= 0.0) return;

    // Compute current mass in kg using abstract fuel units -> mass mapping.
    float fuel_mass_kg = lander->fuel * FUEL_UNIT_MASS_KG;
    float mass_kg = lander->dry_mass + fuel_mass_kg;
    if (mass_kg < MIN_MASS_KG) mass_kg = MIN_MASS_KG;

    // Nominal acceleration constant
    // Preserve scaling by using  nominal mass / current mass:
    // effective_accel = MAIN_THRUST_ACCEL * throttle * (NOMINAL_MASS_KG / mass_kg)
    float thrust_accel_at_full = MAIN_THRUST_ACCEL; // existing constant (m/s^2) at nominal mass
    float mass_scale = NOMINAL_MASS_KG / mass_kg;

	// ---- rotational integration (add near other physics in StepLander) ----
	// Choose approximate lander width/height (meters) consistent with draw size
	const float LANDER_W = 0.6f; // meters (visual scale -> tune to feel)
	const float LANDER_H = 0.6f; // meters

	// compute moment of inertia (rectangle approx)
	float I = (1.0f / 12.0f) * mass_kg * (LANDER_W * LANDER_W + LANDER_H * LANDER_H);

	// compute torque produced by side thrusters (positive = rotate clockwise)
	float side_force = SIDE_THRUST_ACCEL * mass_scale * lander->side_thrust_level * mass_kg; 
	// side_force is in N-equivalent units; torque = force * lever arm
	float torque = side_force * ROLL_LEVER_ARM; // Nm (abstract units consistent with mass_scale)

	// simple rotational damping to prevent runaway (small viscous damping)
	float rotational_damping = 0.05f * lander->omega; // tune

	// angular acceleration α = torque / I  (I in kg·m^2)
	float alpha = 0.0f;
	if (I > 1e-6f) alpha = (torque - rotational_damping) / I;

	// integrate angular velocity and angle
	lander->omega += alpha * (float)dt;
	lander->theta += lander->omega * (float)dt;

	// clamp omega to safe limits
	const float MAX_OMEGA = 6.0f; // rad/s
	if (lander->omega > MAX_OMEGA) lander->omega = MAX_OMEGA;
	if (lander->omega < -MAX_OMEGA) lander->omega = -MAX_OMEGA;

	// Normalize theta to -pi..pi if desired
	if (lander->theta > M_PI) lander->theta -= 2.0f * M_PI;
	if (lander->theta < -M_PI) lander->theta += 2.0f * M_PI;
        
    // vertical accel: gravity downwards plus main engine (upwards)
    float ay = GRAVITY; // positive down in your sim
    float main_throttle = lander->thrust_level;
    float effective_main_accel = thrust_accel_at_full * mass_scale * main_throttle;
    ay -= effective_main_accel;
  
    float side_accel_base = SIDE_THRUST_ACCEL;  
    float ax = side_accel_base * mass_scale * lander->side_thrust_level;

    // Integrate velocity and position (Euler)
    lander->vx += ax * (float)dt;
    lander->vy += ay * (float)dt;
    lander->x  += lander->vx * (float)dt;
    lander->y  += lander->vy * (float)dt;

    // Fuel consumption model (abstract units)   
    float fuel_used = (FUEL_FLOW_MAIN * main_throttle + FUEL_FLOW_SIDE * fabsf(lander->side_thrust_level)) * (float)dt;
    if (fuel_used > lander->fuel) fuel_used = lander->fuel;
    lander->fuel -= fuel_used;
    if (lander->fuel < 0.0f) lander->fuel = 0.0f;

    // Update visual flags (existing behaviour)
    // thrust/left/right flags will be recomputed by autopilot or input, but keep safety clamp
    if (lander->fuel <= 0.0f) {
        lander->thrust_level = 0.0f;
        lander->side_thrust_level = 0.0f;
        lander->thrust = lander->left_thruster = lander->right_thruster = false;
    }  
    //printf("StepLander: mass=%.1f kg, thrust=%.2f, eff_accel=%.2f, net_accel=%.2f\n",
       //mass_kg, main_throttle,
       //effective_main_accel,
       //effective_main_accel - GRAVITY);
}


//======================================================================
static void CheckCollisions(Lander* l, Hazard hazards[], int count) {
    float w = 20.0f, h = 20.0f;
    float left = l->x - w * 0.5f;
    float top = l->y - h;
    float bottom = l->y;

    for (int i = 0; i < count; ++i) {
        if (RectsOverlap(left, top, w, h, hazards[i].x, hazards[i].y, hazards[i].w, hazards[i].h)) {
            l->is_crashed = true; l->is_landed = false; l->vx = l->vy = 0.0f;
            return;
        }
    }
    if (bottom >= SURFACE_Y) {
        if (fabsf(l->vy) <= MAX_LANDING_VY && fabsf(l->vx) <= MAX_LANDING_VX)
            l->is_landed = true;
        else
            l->is_crashed = true;
        l->vx = l->vy = 0.0f;
        l->y = SURFACE_Y;
    }
}
//=====================================================================
// Camera follow: horizontal follows lander, vertical fixed near surface
static void UpdateCamera(const Lander* lander, Camera* cam) {
    float target_x = lander->x + CAMERA_LEAD_PIXELS;
    float target_y = SURFACE_Y - (float)WINDOW_HEIGHT / 2.0f + CAMERA_VERTICAL_BIAS;

    float alpha = CAMERA_FOLLOW_RATE * (float)TIME_SLICE;
    if (alpha > 1.0f) alpha = 1.0f;

    cam->x += (target_x - cam->x) * alpha;
    cam->y += (target_y - cam->y) * alpha;

    float half_w = (float)WINDOW_WIDTH / 2.0f;
    if (cam->x < half_w) cam->x = half_w;
    if (cam->x > WORLD_WIDTH - half_w) cam->x = WORLD_WIDTH - half_w;

    float min_cam_y = (float)WINDOW_HEIGHT / 2.0f - CAMERA_VERTICAL_BIAS;
    float max_cam_y = SURFACE_Y - (float)WINDOW_HEIGHT / 2.0f + CAMERA_VERTICAL_BIAS;
    if (cam->y < min_cam_y) cam->y = min_cam_y;
    if (cam->y > max_cam_y) cam->y = max_cam_y;
}

//======================================================================

//DrawLander: draws a filled rotated rectangle and flames 
static void DrawLander(SDL_Renderer* ren, const Lander* l, const Camera* c)
{
    if (!ren || !l || !c) return;

    // local visual sizing (pixels)
    const float size_px = 20.0f;      // square lander visual size
    const float half = size_px * 0.5f;

    // get screen-space center for the lander (world point treated as the center)
    SDL_FRect centerScreen;
    WorldToScreen(c, l->x, l->y - 10.0f, &centerScreen); // choose vertical offset so rect sits above surface
    // WorldToScreen gives top-left for input point: adjust to center
    float cx = centerScreen.x;
    float cy = centerScreen.y;

    // position the rectangle so (cx,cy) is the center.
    // compute four rectangle corners (local coordinates) then rotate by l->theta and translate to screen.
    float theta = l->theta; // radians, 0 = upright
    float cs = cosf(theta);
    float sn = sinf(theta);

    // local corners relative to center (clockwise or whatever)
    // define rectangle in local coordinates (centered)
    float lx[4], ly[4];
    lx[0] = -half; ly[0] = -half;   // top-left
    lx[1] =  half; ly[1] = -half;   // top-right
    lx[2] =  half; ly[2] =  half;   // bottom-right
    lx[3] = -half; ly[3] =  half;   // bottom-left

    // rotate & translate to screen coords
    SDL_FPoint pts[4];
    for (int i = 0; i < 4; ++i) {
        float rx = lx[i] * cs - ly[i] * sn;
        float ry = lx[i] * sn + ly[i] * cs;
        pts[i].x = cx + rx;
        pts[i].y = cy + ry;
    }

    // draw filled rotated rectangle using two triangles (SDL_Vertex)
    // Note: SDL_RenderGeometry takes an array of vertices and triangle indices.
    // Build vertex color (light grey)
    SDL_Vertex verts[4];
    for (int i = 0; i < 4; ++i) {
        verts[i].position.x = pts[i].x;
        verts[i].position.y = pts[i].y;
        verts[i].tex_coord.x = 0.0f;
        verts[i].tex_coord.y = 0.0f;
        verts[i].color.r = 230;
        verts[i].color.g = 230;
        verts[i].color.b = 230;
        verts[i].color.a = 255;
    }

    // indices for two triangles (0,1,2) and (0,2,3)
    int indices[6] = { 0, 1, 2, 0, 2, 3 };

    // render filled geometry (no texture)
    SDL_RenderGeometry(ren, NULL, verts, 4, indices, 6);

    // -----------------------
    // draw main flame (if thrust > 0)
    // place flame below the center in lander-local coords, pointing "down" (local +y).
    if (l->thrust_level > 0.0001f && l->fuel > 0.0f) {
        // flame length scales with thrust
        float flame_len = 8.0f + 14.0f * l->thrust_level; // pixels
        float flame_w = half * 0.7f;

        // Define flame triangle in local coordinates (tip points downwards)
        // local base points (left/right) near bottom of lander, tip further down
        float base_y = half + 1.5f;        // just below rectangle bottom (local)
        float base_lx = -flame_w * 0.5f, base_rx = flame_w * 0.5f;
        float tip_x = 0.0f, tip_y = base_y + flame_len;

        // rotate each of these to screen coords
        SDL_FPoint fpts[3];
        float local_xs[3] = { base_lx, base_rx, tip_x };
        float local_ys[3] = { base_y,   base_y,  tip_y };
        for (int i = 0; i < 3; ++i) {
            float rx = local_xs[i] * cs - local_ys[i] * sn;
            float ry = local_xs[i] * sn + local_ys[i] * cs;
            fpts[i].x = cx + rx;
            fpts[i].y = cy + ry;
        }

        // flame gradient color: inner bright (yellow) to outer orange - approximate by single color fill
        SDL_Vertex fverts[3];
        for (int i = 0; i < 3; ++i) {
            fverts[i].position.x = fpts[i].x;
            fverts[i].position.y = fpts[i].y;
            fverts[i].tex_coord.x = 0; fverts[i].tex_coord.y = 0;
            // tip is more yellow, base more orange: blend via index
            if (i == 2) { // tip
                fverts[i].color.r = 255; fverts[i].color.g = 210; fverts[i].color.b = 0; fverts[i].color.a = 255;
            } else {
                fverts[i].color.r = 255; fverts[i].color.g = 140; fverts[i].color.b = 0; fverts[i].color.a = 220;
            }
        }
        int findices[3] = {0, 1, 2};
        SDL_RenderGeometry(ren, NULL, fverts, 3, findices, 3);
    }

    // -----------------------
    // draw side flames (if side_thrust nonzero) at left/right side of lander
    if (l->side_thrust_level < -0.001f && l->fuel > 0.0f) {
        // left-side thruster (positive side_thrust_level usually means right; adjust if needed)
        float thr_len = 6.0f + 8.0f * fabsf(l->side_thrust_level);
        float thr_x = -half - 2.0f; // left of rectangle
        float thr_y = 0.0f;         // mid-height local

        // triangle pointing left in local coordinates
        float base_lx = thr_x - thr_len, base_rx = thr_x;
        float base_y_top = thr_y - 3.0f, base_y_bot = thr_y + 3.0f;
        SDL_FPoint tpts[3];
        float local_xs[3] = { base_rx, base_rx, base_lx };
        float local_ys[3] = { base_y_top, base_y_bot, thr_y };
        for (int i = 0; i < 3; ++i) {
            float rx = local_xs[i] * cs - local_ys[i] * sn;
            float ry = local_xs[i] * sn + local_ys[i] * cs;
            tpts[i].x = cx + rx;
            tpts[i].y = cy + ry;
        }
        SDL_Vertex tverts[3];
        for (int i = 0; i < 3; ++i) {
            tverts[i].position.x = tpts[i].x; tverts[i].position.y = tpts[i].y;
            tverts[i].tex_coord.x = 0; tverts[i].tex_coord.y = 0;
            tverts[i].color.r = 255; tverts[i].color.g = 160; tverts[i].color.b = 0; tverts[i].color.a = 220;
        }
        int tind[3] = {0,1,2};
        SDL_RenderGeometry(ren, NULL, tverts, 3, tind, 3);
    }

    if (l->side_thrust_level > 0.001f && l->fuel > 0.0f) {
        // right-side thruster
        float thr_len = 6.0f + 8.0f * fabsf(l->side_thrust_level);
        float thr_x = half + 2.0f; // right of rectangle
        float thr_y = 0.0f;

        float base_lx = thr_x, base_rx = thr_x + thr_len;
        float base_y_top = thr_y - 3.0f, base_y_bot = thr_y + 3.0f;
        SDL_FPoint tpts[3];
        float local_xs[3] = { base_lx, base_lx, base_rx };
        float local_ys[3] = { base_y_top, base_y_bot, thr_y };
        for (int i = 0; i < 3; ++i) {
            float rx = local_xs[i] * cs - local_ys[i] * sn;
            float ry = local_xs[i] * sn + local_ys[i] * cs;
            tpts[i].x = cx + rx;
            tpts[i].y = cy + ry;
        }
        SDL_Vertex tverts[3];
        for (int i = 0; i < 3; ++i) {
            tverts[i].position.x = tpts[i].x; tverts[i].position.y = tpts[i].y;
            tverts[i].tex_coord.x = 0; tverts[i].tex_coord.y = 0;
            tverts[i].color.r = 255; tverts[i].color.g = 160; tverts[i].color.b = 0; tverts[i].color.a = 220;
        }
        int tind[3] = {0,1,2};
        SDL_RenderGeometry(ren, NULL, tverts, 3, tind, 3);
    }
}

//=====================================================================
// Draw landscape (uses WorldToScreen for consistency)
static void DrawLandscape(SDL_Renderer* r, const Camera* c, Hazard hazards[], int hc) {
    SDL_FRect surfScreen;
    WorldToScreen(c, 0.0f, SURFACE_Y, &surfScreen);

    float ground_y = surfScreen.y;
    float ground_h = (float)WINDOW_HEIGHT - ground_y;
    if (ground_h > 0.0f) {
        SDL_FRect groundRect = { 0.0f, ground_y, (float)WINDOW_WIDTH, ground_h };
        SDL_SetRenderDrawColor(r, 180, 30, 30, 255);
        SDL_RenderFillRect(r, &groundRect);
    }

    SDL_SetRenderDrawColor(r, 160, 60, 60, 255);
    for (int i = 0; i < hc; ++i) {
        SDL_FRect hr;
        WorldToScreen(c, hazards[i].x, hazards[i].y, &hr);
        hr.w = hazards[i].w; hr.h = hazards[i].h;
        if (hr.x + hr.w < 0 || hr.x > WINDOW_WIDTH || hr.y + hr.h < 0 || hr.y > WINDOW_HEIGHT)
            continue;
        SDL_RenderFillRect(r, &hr);
    }
}
//======================================================================

static void DrawPredictiveScanOverlay(SDL_Renderer *r, const Camera *c,
                                      const AutopilotState *ap,
                                      const Hazard *hazards, int hazard_count)
{
    if (!r || !c || !ap) return;

    // --- (A) Draw translucent hazard boxes so you can see alignment ---
    SDL_SetRenderDrawColor(r, 255, 0, 0, 80);
    for (int i = 0; i < hazard_count; ++i) {
        SDL_FRect hr;
        WorldToScreen(c, hazards[i].x, hazards[i].y, &hr);
        hr.w = hazards[i].w;
        hr.h = hazards[i].h;
        SDL_RenderRect(r, &hr);
    }

    // --- (B) Draw all scanned candidate sites (white dots) ---
    SDL_SetRenderDrawColor(r, 255, 255, 255, 255);
    for (int i = 0; i < ap->site_mem_count; ++i) {
        const CandidateSite *s = &ap->site_mem[i];

        // Place the dot near surface level (use surface Y minus small offset)
        SDL_FRect sr;
        WorldToScreen(c, s->x, SURFACE_Y - 5.0f, &sr);
        sr.w = 4.0f;
        sr.h = 4.0f;
        sr.x -= 2.0f;
        sr.y -= 2.0f;
        SDL_RenderFillRect(r, &sr);
    }

    // --- (C) Draw the current best target site (green) ---
    if (ap->has_target) {
        SDL_SetRenderDrawColor(r, 0, 255, 0, 255);
        SDL_FRect best;
        WorldToScreen(c, ap->target_x, SURFACE_Y - 6.0f, &best);
        best.w = 8.0f;
        best.h = 8.0f;
        best.x -= 4.0f;
        best.y -= 4.0f;
        SDL_RenderFillRect(r, &best);
    }
}

//======================================================================
// render text for  HUD 
static void render_text(SDL_Renderer *ren, TTF_Font *font, const char *text, float x, float y)
{
    if (!ren || !font || !text) return;
    SDL_Color white = {255,255,255,255};
    size_t len = strlen(text);
    SDL_Surface *surf = TTF_RenderText_Blended(font, text, len, white);
    if (!surf) return;
    SDL_Texture *tex = SDL_CreateTextureFromSurface(ren, surf);
    if (!tex) { SDL_DestroySurface(surf); return; }
    SDL_FRect dst = { x, y, (float)surf->w, (float)surf->h };
    SDL_RenderTexture(ren, tex, NULL, &dst);
    SDL_DestroyTexture(tex);
    SDL_DestroySurface(surf);
}
//=======================================================================
static void DrawHUD(SDL_Renderer *ren, TTF_Font *font, const Lander *l,
                    const AutopilotState *ap, float kp_display)
{
    if (!ren || !font || !l || !ap) return;

    char buf[128];
    float x = 8.0f;
    float line_h = (float)TTF_GetFontHeight(font);
    float y = 6.0f;

    float alt = SURFACE_Y - l->y; //altitude
    if (alt < 0.0f) alt = 0.0f;

    snprintf(buf, sizeof(buf), "ALT: %.1f", alt);
    render_text(ren, font, buf, x, y); y += line_h + 2;
   
    // Fuel
    snprintf(buf, sizeof(buf), "FUEL: %.1f", l->fuel);
    render_text(ren, font, buf, x, y); y += line_h + 2;

    // Status string (CRASHED / LANDED / HOVER / FLYING)
    snprintf(buf, sizeof(buf), "STATUS: %s",
             l->is_crashed ? "CRASHED" :
             (l->is_landed ? "LANDED" :
             (l->is_hovering ? "HOVER" : "FLYING")));
    render_text(ren, font, buf, x, y); y += line_h + 6;
    
     // Autopilot on/off
    snprintf(buf, sizeof(buf), "AUTOPILOT: %s", ap->enabled ? "ON" : "OFF");
    render_text(ren, font, buf, x, y); y += line_h + 2;
   
    snprintf(buf, sizeof(buf),"AUTOPILOT STAGE: %d", ap->stage);
    render_text(ren, font, buf, x, y); y += line_h + 2;
    
    snprintf(buf, sizeof(buf),"POSITION & TARGET: x=%.1f Target_x=%.1f ",l->x, ap->target_x);
    render_text(ren, font, buf, x, y); y += line_h + 2;
    snprintf(buf, sizeof(buf),"VELOCITY: vx=%.2f vy=%.2f ", l->vx, l->vy);
    render_text(ren, font, buf, x, y); y += line_h + 2;
    //snprintf(buf, sizeof(buf),"PITCH ANGLE(degrees): theta=%.2f ", l->theta* M_PI / 180.0f);
    snprintf(buf, sizeof(buf),"PITCH ANGLE: θ=%.2f° ", l->theta * 180.0f / M_PI);
    render_text(ren, font, buf, x, y); y += line_h + 2;
    snprintf(buf, sizeof(buf),"THRUST: main=%.3f side=%.3f ",l->thrust_level, l->side_thrust_level);
    render_text(ren, font, buf, x, y); y += line_h + 2;
}



//======================================================================
float ComputeLandingZoneSafetyAtX(const Lander* l,
                                  const Hazard* hazards,
                                  int hazardCount,
                                  float x_center,
                                  float site_width,
                                  float site_depth)
{
    float safety = 1.0f;

    // Compute sample region below lander
    float site_left  = x_center - site_width * 0.5f;
    float site_right = x_center + site_width * 0.5f;
  
    // Check against all hazards
    for (int i = 0; i < hazardCount; ++i) {
        float hx = hazards[i].x;
        float hw = hazards[i].w;
        float hx_left  = hx - hw * 0.5f;
        float hx_right = hx + hw * 0.5f;

        // if overlap between site zone and hazard footprint (unsafe)
        bool overlap = !(site_right < hx_left || site_left > hx_right);
        if (overlap) {
            safety = 0.0f;
            //printf("[DEBUG HAZARD] SiteX=%.1f overlaps hazard %d (hx=%.1f w=%.1f)\n",  x_center, i, hx, hw);
            break;
        }
    }

    // optionally bias score by proximity to hazards
    if (safety > 0.0f) {
        // e.g. check for near hazards within a margin zone
        float margin = site_width;
        for (int i = 0; i < hazardCount; ++i) {
            float hx = hazards[i].x;
            float hw = hazards[i].w;
            float dx = fabsf(hx - x_center);
            if (dx < hw * 0.5f + margin) {
                safety *= 0.8f;
                //printf("[DEBUG SAFE-BIAS] SiteX=%.1f near hazard %d (dx=%.1f)\n", x_center, i, dx);
            }
        }
    }

    // Clamp between 0 and 1
    if (safety < 0.0f) safety = 0.0f;
    if (safety > 1.0f) safety = 1.0f;

    //printf("[SCAN2] x=%.1f score=%.2f\n", x_center, safety);
    return safety;
}

// -----------------------------------------------------------------------------
// PredictiveScan()
//   Scans a grid of candidate landing sites below and ahead of the lander.
//   Fills ap->site_mem ring buffer with scored CandidateSite entries.
//   Uses ComputeLandingZoneSafetyAtX() for each column center.
// -----------------------------------------------------------------------------
void PredictiveScan(const Lander *l, const Hazard *hazards, int hazard_count,
                    AutopilotState *ap, float scan_ahead, float grid_spacing,
                    int grid_cols, int grid_rows)
{
    if (!l || !ap || !hazards || hazard_count <= 0) return;
    if (grid_cols <= 0) grid_cols = 5;
    if (grid_rows <= 0) grid_rows = 1;
    if (grid_spacing <= 0.0f) grid_spacing = 40.0f;
    if (scan_ahead <= 0.0f) scan_ahead = 200.0f;

    float start_x = l->x - (grid_cols * 0.5f) * grid_spacing;
    float y_base = l->y + 40.0f; // look somewhat below the lander

    for (int c = 0; c < grid_cols; ++c) {
        float cx = start_x + c * grid_spacing;
        float score = ComputeLandingZoneSafetyAtX(l, hazards, hazard_count, cx, 40.0f, 80.0f);

        CandidateSite site = {
            .x = cx,
            .y = y_base,
            .score = score,
            .width = 40.0f,
            .timestamp = SDL_GetTicks() / 1000.0
        };

        AddSiteMemory(ap, &site);
        //printf("[SCAN2] x=%.1f score=%.2f\n", cx, score);
    }
}

// -----------------------------------------------------------------------------
// AddSiteMemory()
//   Adds a new CandidateSite to the ring buffer inside AutopilotState.
//   Keeps up to 16 entries (overwrites oldest).
// -----------------------------------------------------------------------------
void AddSiteMemory(AutopilotState *ap, const CandidateSite *site)
{
    if (!ap || !site) return;
    int idx = ap->site_mem_head % 16;
    ap->site_mem[idx] = *site;
    ap->site_mem_head = (ap->site_mem_head + 1) % 16;
    if (ap->site_mem_count < 16) ap->site_mem_count++;
}


// -----------------------------------------------------------------------------
// SelectBestSite()  — improved "mid-gap" heuristic
//   Chooses the best landing site from autopilot memory, preferring:
//     (1) highest safety score
//     (2) site closest to midpoint of largest clear gap between hazards
//     (3) if tie, nearest to current lander position
// -----------------------------------------------------------------------------
CandidateSite SelectBestSite(const AutopilotState *ap, const Lander *l)
{
    CandidateSite best = { .score = 0.0f, .x = l ? l->x : 0.0f };
    if (!ap || ap->site_mem_count <= 0 || !l) return best;

    // --- Step 1: find global best score ---
    float max_score = 0.0f;
    for (int i = 0; i < ap->site_mem_count; ++i) {
        if (ap->site_mem[i].score > max_score)
            max_score = ap->site_mem[i].score;
    }

    // --- Step 2: identify contiguous "clear" zones (score ≈ max_score) ---
    float cluster_start = 0.0f, cluster_end = 0.0f;
    bool in_cluster = false;
    float widest_gap = 0.0f, gap_mid = l->x;

    for (int i = 0; i < ap->site_mem_count; ++i) {
        const CandidateSite *s = &ap->site_mem[i];
        bool clear = (s->score >= max_score * 0.95f);

        if (clear && !in_cluster) {
            in_cluster = true;
            cluster_start = s->x;
        } else if (!clear && in_cluster) {
            // cluster ended
            cluster_end = ap->site_mem[i - 1].x;
            float gap = cluster_end - cluster_start;
            if (gap > widest_gap) {
                widest_gap = gap;
                gap_mid = 0.5f * (cluster_start + cluster_end);
            }
            in_cluster = false;
        }
    }
    // Handle case where the last sites are clear
    if (in_cluster) {
        cluster_end = ap->site_mem[ap->site_mem_count - 1].x;
        float gap = cluster_end - cluster_start;
        if (gap > widest_gap) {
            widest_gap = gap;
            gap_mid = 0.5f * (cluster_start + cluster_end);
        }
    }

    // --- Step 3: choose the candidate closest to the gap midpoint ---
    float best_dist = 1e9f;
    for (int i = 0; i < ap->site_mem_count; ++i) {
        const CandidateSite *s = &ap->site_mem[i];
        if (s->score >= max_score * 0.95f) {
            float dist = fabsf(s->x - gap_mid);
            if (dist < best_dist) {
                best = *s;
                best_dist = dist;
            }
        }
    }

    // --- Step 4: small tie-breaker bias toward proximity to the lander ---
    for (int i = 0; i < ap->site_mem_count; ++i) {
        const CandidateSite *s = &ap->site_mem[i];
        if (fabsf(s->score - best.score) < 0.02f) {
            float d_curr = fabsf(best.x - l->x);
            float d_new  = fabsf(s->x - l->x);
            if (d_new < d_curr)
                best = *s;
        }
    }

    printf("[SITE] best_x=%.1f score=%.2f (gap_mid=%.1f)\n", best.x, best.score, gap_mid);
    return best;
}

//======================================================================

void AutoPilot_Update(Lander *l, AutopilotState *ap,
                             const Hazard *hazards, int hazardCount, float dt)
{
    if (!l || !ap || dt <= 0.0f) return;
    if (l->is_landed || l->is_crashed) return;
    
    static FILE *logf = NULL;
    if (!logf) {
    logf = fopen("autopilot_telemetry.csv", "w");
    if (logf) {
    fprintf(logf, "time,stage,altitude,x,target_x,vx,vy,thr,side,angle\n");
    fflush(logf);
    }
    }   
       
    float altitude = fmaxf(0.0f, (float)(SURFACE_Y - l->y));  
    
    //visual flags    
    if      (altitude > ALT_VY_HIGH1 && altitude < SURFACE_Y) ap->stage =1;  
    else if (altitude > ALT_VY_HIGH2 && altitude < ALT_VY_HIGH1) ap->stage =2;   
    else if (altitude > ALT_VY_HIGH3 && altitude < ALT_VY_HIGH2) ap->stage =3;     
    else if (altitude > ALT_VY_HIGH4 && altitude < ALT_VY_HIGH3 ) ap->stage =4;     
    else if (altitude >0  && altitude < ALT_VY_HIGH4 ) ap->stage =5; 
    else   ap->stage =0;


    static float sim_time = 0.0f;
    sim_time += dt;
    ap->stage_timer += dt;

   
    float fuel_mass = l->fuel * FUEL_UNIT_MASS_KG;
    float mass = l->dry_mass + fuel_mass;
    float mass_scale = NOMINAL_MASS_KG / fmaxf(mass, MIN_MASS_KG);
    float hover_thr = GRAVITY / (MAIN_THRUST_ACCEL * mass_scale);
    hover_thr = fminf(fmaxf(hover_thr, 0.0f), 1.0f);

    // --- Target selection ---
    if (!ap->has_target && altitude > ALT_TARGET_LOCK) {
        PredictiveScan(l, hazards, hazardCount, ap,
                       SCAN_DISTANCE, SCAN_ANGLE_DEG, SCAN_RAYS, SCAN_MODE);
        CandidateSite s = SelectBestSite(ap, l);
        if (s.score > 0.0f) {
            ap->has_target = true;
            ap->target_x = s.x;
            ap->target_score = s.score;
            printf("[AUTOPILOT] Target locked x=%.1f score=%.2f at alt=%.1f\n",
                   s.x, s.score, altitude);
        }
    }

    // --- Lateral control ---
    float desired_side = 0.0f;
    if (ap->has_target) {
        float dx = ap->target_x - l->x;
        desired_side = KP_LAT * dx - KD_LAT * l->vx;
        desired_side = fmaxf(fminf(desired_side, SIDE_MAX), -SIDE_MAX);
    }
    l->side_thrust_level = SIDE_BLEND_A * l->side_thrust_level +
                           SIDE_BLEND_B * desired_side;

    // --- Vertical descent targets ---
    float target_vy;
    if      (altitude > ALT_VY_HIGH1) target_vy = VY_TARGET1;
    else if (altitude > ALT_VY_HIGH2) target_vy = VY_TARGET2;
    else if (altitude > ALT_VY_HIGH3) target_vy = VY_TARGET3;
    else if (altitude > ALT_VY_HIGH4) target_vy = VY_TARGET4;
    else                              target_vy = VY_TARGET_FINAL;

    // --- Vertical PD controller ---
    float vy_err = target_vy - l->vy;
    float ay_des = KV_VY * vy_err;
    float thr = (GRAVITY - ay_des) / (MAIN_THRUST_ACCEL * mass_scale);
    thr = fmaxf(fminf(thr, 1.0f), 0.0f);

    // --- Gentle flare ---
    if (altitude < FLARE_ALT) {
		l->is_hovering = true;
        float frac = altitude / FLARE_ALT;
        float flare_thr = hover_thr *
                          (FLARE_BASE + FLARE_GRADIENT * frac);
        if (thr < flare_thr) thr = flare_thr;
    }

    // --- Anti-hover bleed ---
    if (altitude > BLEED_ALT_MIN && fabsf(l->vy) < BLEED_VY_MAX) {
        thr -= BLEED_DELTA_THR;
        if (thr < 0.0f) thr = 0.0f;
    }

    // --- Pitch decay ---
    float theta_err = l->theta;
    float omega_cmd = -KP_THETA * theta_err - KD_THETA * l->omega;
    l->omega += (omega_cmd - l->omega) * PITCH_BLEND;
    l->theta += l->omega * dt;
    l->theta = fmaxf(fminf(l->theta, PITCH_LIMIT_MAX), -PITCH_LIMIT_MAX);

    l->thrust_level = thr;

    // --- Touchdown detection ---
    if (altitude <= ALT_TOUCHDOWN) {
        if (fabsf(l->vy) < LAND_VY_MAX && fabsf(l->vx) < LAND_VX_MAX) {
            l->is_landed = true;
            l->thrust_level = l->side_thrust_level = 0.0f;
            printf("[AUTOPILOT] Touchdown OK\n");
        } else {
            l->is_crashed = true;
            l->thrust_level = l->side_thrust_level = 0.0f;
            printf("[AUTOPILOT] Crash (vy=%.2f vx=%.2f)\n", l->vy, l->vx);
        }
    }
    
    // --- Telemetry log file (all data) ---
    if (logf) {
        fprintf(logf, "%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
            sim_time, ap->stage, altitude, l->x, ap->target_x,
            l->vx, l->vy, l->thrust_level, l->side_thrust_level, l->theta * 180.0f / M_PI);  
        fflush(logf); // ensures data is written each frame
    }
    
    // --- Telemetry print (every 60 frames) ---
    static int cnt = 0;
    if ((cnt++ % DEBUG_FRAME_INTERVAL) == 0) {
        printf("[AUTOPILOT] alt=%.1f y=%.1f x=%.1f->%.1f vx=%.2f vy=%.2f thr=%.2f side=%.2f θ=%.1f° ω=%.3f\n",
               altitude, l->y, l->x, ap->has_target ? ap->target_x : l->x,
               l->vx, l->vy, l->thrust_level, l->side_thrust_level,
               l->theta * 180.0f / M_PI, l->omega);
    }
}



//======================================================================
// main program
int main(void) {
     
     if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS | SDL_INIT_AUDIO)) {
        SDL_Log("Couldn't initialize SDL: %s", SDL_GetError());
        return 1;
    }

    if (!TTF_Init()) {
        SDL_Log("TTF_Init failed");
        SDL_Quit();
        return 1;
    }

    TTF_Font *font = TTF_OpenFont("assets/font.ttf", 16);
    if (!font) {
        SDL_Log("Failed to load font: %s. HUD will be blank.", SDL_GetError());
    }
    
    //audio
    SDL_AudioSpec spec;
    char *wav_path = NULL;
    SDL_AudioStream *bleep_stream = NULL;
    Uint8 *wav_data = NULL;
    Uint32 wav_data_len = 0;
      
    SDL_asprintf(&wav_path, "%sassets/bleep.wav", SDL_GetBasePath());  /* allocate a string of the full file path */
    if (!SDL_LoadWAV(wav_path, &spec, &wav_data, &wav_data_len)) {
        SDL_Log("Couldn't load bleep .wav file: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
     SDL_free(wav_path);  /* done with this string. */

    /* Create our audio stream in the same format as the .wav file. It'll convert to what the audio hardware wants. */
    bleep_stream = SDL_OpenAudioDeviceStream(SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, &spec, NULL, NULL);
    if (!bleep_stream) {
        SDL_Log("Couldn't create audio stream: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }

    SDL_ResumeAudioStreamDevice(bleep_stream);
         

    SDL_Window* win = SDL_CreateWindow("Lunar Lander Simulator", WINDOW_WIDTH, WINDOW_HEIGHT, 0);
    SDL_Renderer* ren = SDL_CreateRenderer(win, NULL);
    if (!ren) { SDL_Log("Renderer error: %s", SDL_GetError()); SDL_DestroyWindow(win); SDL_Quit(); return 1; }

	SDL_Texture *lander_tex = NULL;
	int LANDER_PIX = 20; // pixels square

	// create a target texture and draw the simple rectangle into it
	lander_tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, LANDER_PIX, LANDER_PIX);
	if (lander_tex) {
		// set target to the texture
		SDL_SetRenderTarget(ren, lander_tex);
		SDL_SetRenderDrawColor(ren, 0,0,0,0);
		SDL_RenderClear(ren);
		SDL_SetRenderDrawColor(ren, 230,230,230,255);
		SDL_FRect r = { 2, 2, LANDER_PIX - 4, LANDER_PIX - 4 };
		SDL_RenderFillRect(ren, &r);		
		// reset render target to default
		SDL_SetRenderTarget(ren, NULL);
	}

	  
    // Each hazard uses { x, SURFACE_Y - height, width, height }.
    
	Hazard hazards[] = {
    // Central region hazards (near default hover area)
    {1500, SURFACE_Y - 25, 60, 25},
    {1800, SURFACE_Y - 35, 40, 35},
    {2200, SURFACE_Y - 30, 100, 30},
    {3000, SURFACE_Y - 28, 50, 28},
    {3600, SURFACE_Y - 20, 30, 20},
    {4200, SURFACE_Y - 32, 120, 32},

    // New extra hazards for testing radar triggers
    {2600, SURFACE_Y - 38, 80, 38},
    {2800, SURFACE_Y - 34, 70, 34},
    {3400, SURFACE_Y - 26, 60, 26},
    {3800, SURFACE_Y - 22, 50, 22},
    {4000, SURFACE_Y - 29, 90, 29},
    {4400, SURFACE_Y - 35, 100, 35},

    // Small scattered “rocks” around 40 m height to trigger radar intermittently
    {2400, SURFACE_Y - 10, 15, 10},   
    {2500, SURFACE_Y - 12, 18, 12},
    {2700, SURFACE_Y - 14, 20, 14},
    {3100, SURFACE_Y - 16, 22, 16},
    {3300, SURFACE_Y - 18, 25, 18},
    {3500, SURFACE_Y - 14, 18, 14}    
	};
	int hc = sizeof(hazards) / sizeof(hazards[0]); //hc =hazard count


    Lander l; Camera c;
    InitLander(&l);
    c.x = l.x + CAMERA_LEAD_PIXELS;
    c.y = SURFACE_Y - (float)WINDOW_HEIGHT / 2.0f + CAMERA_VERTICAL_BIAS;

    // autopilot state    
    AutopilotState ap = {
	 .enabled = true,
     .stage =0,
    .burn_timer = 0.0,
    .target_theta = 0.0f, // keep engines-down as default   
    .last_throttle =0.0,
    .target_x = 0.0,
    .has_target =false,
    .target_score =0.0,
    .target_y =0.0,
    .scan_last_time =0.0,
    .site_mem_count=0,
    .site_mem_head = 0
	};
	
    Uint32 prev = SDL_GetTicks();
    double acc = 0.0;
    bool run = true;
    bool prev_p = false; // For P-key edge toggle:

    while (run) {
        Uint32 now = SDL_GetTicks();
        double elapsed = ((double)now - (double)prev) / 1000.0;
        prev = now;
        if (elapsed < 0.0) elapsed = 0.0;
        elapsed *= SIM_SPEED;
        acc += elapsed;

        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_EVENT_QUIT || e.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED) run = false;
        }

        const bool* kb = SDL_GetKeyboardState(NULL);
        if (kb[SDL_SCANCODE_ESCAPE]) run = false;
        
        if (kb[SDL_SCANCODE_R]) //restart
        {
        InitLander(&l);        
        c.x = l.x + CAMERA_LEAD_PIXELS;
        c.y = SURFACE_Y - (float)WINDOW_HEIGHT / 2.0f + CAMERA_VERTICAL_BIAS;        
        // clear autopilot 
        ap.enabled = true,
        ap.stage = 0;
        ap.burn_timer = 0.0;
        ap.target_theta = 0.0f; // keep engines-down as default   
        ap.last_throttle =0.0;
        ap.target_x = 0.0;        
        ap.target_score =0.0;
        ap.target_y =0.0;
        ap.scan_last_time =0.0;        
        ap.site_mem_head = 0;        
        ap.site_mem_count = 0;
		ap.has_target = false;	
        PredictiveScan(&l, hazards, hc, &ap, 200.0f, 40.0f, 9, 1);// immediately perform a predictive scan so we get a valid target       
        prev_p = false;   // keep autopilot disabled by default after reset so user chooses to enable    
        } //restart
        
        // P key toggle (autopilot)
        bool p_down = kb[SDL_SCANCODE_P];
        
        if (p_down && !prev_p) {
		  ap.enabled = !ap.enabled;
        //printf("autopilot enabled\n");
        //printf("Autopilot %s\n", ap.enabled ? "ENABLED" : "DISABLED"); 
		}
		
	  if (!ap.enabled) {
            // main discrete binary control for now
            l.thrust = kb[SDL_SCANCODE_W] || kb[SDL_SCANCODE_UP];
            l.left_thruster = kb[SDL_SCANCODE_A] || kb[SDL_SCANCODE_LEFT];
            l.right_thruster = kb[SDL_SCANCODE_D] || kb[SDL_SCANCODE_RIGHT];

            l.thrust_level = l.thrust ? 1.0f : 0.0f;
            if (l.left_thruster) l.side_thrust_level = -1.0f;
            else if (l.right_thruster) l.side_thrust_level = 1.0f;
            else l.side_thrust_level = 0.0f;
        } else {
            // autopilot updates physics commands (we call it before stepping physics)            
        }
        
        // Fixed-step physics loop (call autopilot each sub-step if enabled)
        int steps = 0;
        double dt = TIME_SLICE;
        while (acc >= TIME_SLICE && steps++ < MAX_PHYS_STEPS) {
            if (ap.enabled) AutoPilot_Update(&l, &ap, hazards, hc, dt);            
            StepLander(&l, dt);
            acc -= TIME_SLICE;
        }
        if (steps == MAX_PHYS_STEPS) acc = 0.0;

        CheckCollisions(&l, hazards, hc);
        UpdateCamera(&l, &c);
        
        // --- Auto-refresh predictive scan once per second ---
		ap.scan_last_time += dt;
		if (ap.scan_last_time >= 1.0) {
			ap.scan_last_time = 0.0;
			PredictiveScan(&l, hazards, hc, &ap, 200.0f, 40.0f, 9, 1);
		}
				
		//bleep (landing in progress)
		static int cnt = 0;
        if ((cnt++ % BLEEP_INTERVAL) == 0 && is_bleeping && !l.is_crashed && !l.is_landed) {
		if (SDL_GetAudioStreamQueued(bleep_stream) < (int)wav_data_len) {		
		SDL_PutAudioStreamData(bleep_stream, wav_data, wav_data_len);
		}
		}//if bleep interval

        // Render
        SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
        SDL_RenderClear(ren);
        DrawLandscape(ren, &c, hazards, hc);
        DrawLander(ren, &l, &c);   
        DrawPredictiveScanOverlay(ren, &c, &ap,hazards,hc);	
        
		DrawHUD(ren, font, &l, &ap, 0.0f);
        SDL_RenderPresent(ren);
        SDL_Delay(1);
    }

    if (font) TTF_CloseFont(font);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    TTF_Quit();
    SDL_Quit();
    return 0;
}
