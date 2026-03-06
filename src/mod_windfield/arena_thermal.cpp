/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2026 Arena Thermal System for GP Training
 *
 * Arena-bounded thermal system implementation.
 */

#include "arena_thermal.h"
#include "../mod_misc/crrc_rand.h"
#include "../mod_misc/ls_constants.h"
#include "../global.h"
#include "../config.h"
#include <cmath>
#include <iostream>
#include <algorithm>

// M_TO_FT and FT_TO_M are already defined in ls_constants.h

// Global instance
ArenaThermalField* g_arenaThermalField = nullptr;

// ---------- ArenaThermalConfig ----------

ArenaThermalConfig::ArenaThermalConfig()
    : enabled(false)
    , spawn_x_min(-150.0f)
    , spawn_x_max(150.0f)
    , spawn_y_min(-150.0f)
    , spawn_y_max(150.0f)
    , count_min(0)
    , count_max(5)
    , strength_mean(2.0f)
    , strength_sigma(0.5f)
    , radius_mean(20.0f)
    , radius_sigma(5.0f)
    , lifetime_mean(180.0f)
    , lifetime_sigma(60.0f)
    , height_m(300.0f)
{
}

void ArenaThermalConfig::loadFromXML(SimpleXMLTransfer* el)
{
    if (!el) return;

    enabled = (el->getInt("enabled", 0) != 0);

    // Bounds (in meters)
    SimpleXMLTransfer* bounds = el->getChild("bounds", true);
    if (bounds) {
        spawn_x_min = bounds->getDouble("x_min", spawn_x_min);
        spawn_x_max = bounds->getDouble("x_max", spawn_x_max);
        spawn_y_min = bounds->getDouble("y_min", spawn_y_min);
        spawn_y_max = bounds->getDouble("y_max", spawn_y_max);
    }

    // Count range
    count_min = el->getInt("count_min", count_min);
    count_max = el->getInt("count_max", count_max);

    // Thermal parameters (in meters and m/s)
    strength_mean = el->getDouble("strength_mean", strength_mean);
    strength_sigma = el->getDouble("strength_sigma", strength_sigma);
    radius_mean = el->getDouble("radius_mean", radius_mean);
    radius_sigma = el->getDouble("radius_sigma", radius_sigma);
    lifetime_mean = el->getDouble("lifetime_mean", lifetime_mean);
    lifetime_sigma = el->getDouble("lifetime_sigma", lifetime_sigma);
    height_m = el->getDouble("height_m", height_m);

#ifdef DETAILED_LOGGING
    if (enabled) {
        std::cout << "Arena thermals enabled: "
                  << "bounds=[" << spawn_x_min << "," << spawn_x_max << "] x ["
                  << spawn_y_min << "," << spawn_y_max << "] m, "
                  << "count=" << count_min << "-" << count_max << ", "
                  << "strength=" << strength_mean << "+/-" << strength_sigma << " m/s, "
                  << "radius=" << radius_mean << "+/-" << radius_sigma << " m"
                  << std::endl;
    }
#endif
}

// ---------- ArenaThermal ----------

ArenaThermal::ArenaThermal()
    : center_x(0)
    , center_y(0)
    , radius(50)
    , strength(5)
    , lifetime(180)
    , initial_lifetime(180)
{
}

// Helper: Gaussian random using Box-Muller (deterministic with CRRC_Random)
static double gaussianRandom(double mean, double sigma, const char* context)
{
    // Simple Box-Muller transform
    double u1 = static_cast<double>(CRRC_Random::rand()) /
                (static_cast<double>(CRRC_Random::max()) + 1.0);
    double u2 = static_cast<double>(CRRC_Random::rand()) /
                (static_cast<double>(CRRC_Random::max()) + 1.0);

    // Avoid log(0)
    if (u1 < 1e-10) u1 = 1e-10;

    double z = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
    return mean + sigma * z;
}

// Helper: Uniform random in range
static double uniformRandom(double min, double max, const char* context)
{
    double u = static_cast<double>(CRRC_Random::rand()) /
               (static_cast<double>(CRRC_Random::max()) + 1.0);
    return min + u * (max - min);
}

void ArenaThermal::randomInit(const ArenaThermalConfig& config)
{
    // Position: uniform within spawn bounds (convert m to ft)
    double x_m = uniformRandom(config.spawn_x_min, config.spawn_x_max, "arena_thermal_x");
    double y_m = uniformRandom(config.spawn_y_min, config.spawn_y_max, "arena_thermal_y");
    center_x = static_cast<float>(x_m * M_TO_FT);
    center_y = static_cast<float>(y_m * M_TO_FT);

    // Radius: Gaussian, clamp to positive (convert m to ft)
    double r_m = gaussianRandom(config.radius_mean, config.radius_sigma, "arena_thermal_radius");
    r_m = std::max(5.0, r_m);  // minimum 5m radius
    radius = static_cast<float>(r_m * M_TO_FT);

    // Strength: Gaussian, clamp to positive (convert m/s to ft/s)
    double s_ms = gaussianRandom(config.strength_mean, config.strength_sigma, "arena_thermal_strength");
    s_ms = std::max(0.3, s_ms);  // minimum 0.3 m/s
    strength = static_cast<float>(s_ms * M_TO_FT);

    // Lifetime: Gaussian, clamp to minimum
    double lt = gaussianRandom(config.lifetime_mean, config.lifetime_sigma, "arena_thermal_lifetime");
    lt = std::max(30.0, lt);  // minimum 30 seconds
    lifetime = static_cast<float>(lt);
    initial_lifetime = lifetime;
}

bool ArenaThermal::update(float dt, float wind_vel_x, float wind_vel_y,
                          const ArenaThermalConfig& config)
{
    // Move with wind (wind_vel already in ft/s)
    center_x += wind_vel_x * dt;
    center_y += wind_vel_y * dt;

    // Age
    lifetime -= dt;

    // Check if should respawn
    if (lifetime <= 0) {
        return true;  // respawn
    }

    // Check if drifted far outside spawn bounds (2x buffer)
    double x_m = center_x * FT_TO_M;
    double y_m = center_y * FT_TO_M;
    double buffer = 2.0;  // allow 2x the arena size before respawn
    double x_range = (config.spawn_x_max - config.spawn_x_min) * buffer;
    double y_range = (config.spawn_y_max - config.spawn_y_min) * buffer;
    double x_center = (config.spawn_x_min + config.spawn_x_max) / 2.0;
    double y_center = (config.spawn_y_min + config.spawn_y_max) / 2.0;

    if (fabs(x_m - x_center) > x_range || fabs(y_m - y_center) > y_range) {
        return true;  // respawn - drifted too far
    }

    return false;  // keep going
}

double ArenaThermal::getVerticalVelocity(double x, double y, double z) const
{
    // Distance from thermal center (horizontal only)
    double dx = x - center_x;
    double dy = y - center_y;
    double dist = sqrt(dx * dx + dy * dy);

    // Outside influence radius (use 2x radius for sink ring)
    if (dist > radius * 2.0) {
        return 0.0;
    }

    // Height factor: thermals weak near ground, full strength above ~50ft
    double alt = -z;  // z is down-positive
    double height_factor = 1.0;
    if (alt < 0) {
        return 0.0;  // below ground
    } else if (alt < 50.0) {
        height_factor = alt / 50.0;  // linear ramp
    }

    // Lifetime fade-out in last 10 seconds
    double fade = 1.0;
    if (lifetime < 10.0 && initial_lifetime > 10.0) {
        fade = lifetime / 10.0;
    }

    // Velocity profile: Gaussian bell curve for core, sink ring outside
    // Core: full updraft within radius
    // Ring: sink from radius to 2*radius
    double vel;
    if (dist < radius) {
        // Inside core: Gaussian profile
        double r_norm = dist / radius;
        vel = strength * exp(-r_norm * r_norm * 2.0);  // peaks at center
    } else {
        // Sink ring: conservation of mass
        double r_norm = (dist - radius) / radius;
        double sink = -strength * 0.3 * exp(-r_norm * r_norm * 2.0);
        vel = sink;
    }

    // Return negative because vel_down is positive-down
    return -vel * height_factor * fade;
}

void ArenaThermal::getHorizontalVelocity(double x, double y, double z,
                                          double& vel_x, double& vel_y) const
{
    // Simplified horizontal inflow (vacuum cleaner effect)
    // Only significant close to ground
    double alt = -z;
    if (alt > 1000.0 || alt < 0) {
        vel_x = vel_y = 0;
        return;
    }

    double dx = center_x - x;
    double dy = center_y - y;
    double dist = sqrt(dx * dx + dy * dy);

    if (dist < 1.0 || dist > radius * 3.0) {
        vel_x = vel_y = 0;
        return;
    }

    // Inflow velocity magnitude (max at radius, drops off)
    double v_in_max = strength * radius / 100.0;
    double r_norm = dist / radius;

    double v_in;
    if (dist > radius) {
        v_in = v_in_max / (r_norm * r_norm);
    } else {
        v_in = v_in_max * r_norm;
    }

    // Height attenuation
    double h_factor = 1.0;
    if (alt > 50.0) {
        h_factor = std::max(0.0, 1.0 - (alt - 50.0) / 950.0);
    }

    // Direction toward thermal center
    double angle = atan2(dy, dx);
    vel_x = v_in * cos(angle) * h_factor;
    vel_y = v_in * sin(angle) * h_factor;
}

// ---------- ArenaThermalField ----------

ArenaThermalField::ArenaThermalField()
    : thermalCount_(0)
{
}

ArenaThermalField::~ArenaThermalField()
{
}

void ArenaThermalField::initialize(const ArenaThermalConfig& config)
{
    config_ = config;
    thermalCount_ = 0;

    if (!config_.enabled) {
        return;
    }

    spawnThermals();

#ifdef DETAILED_LOGGING
    std::cout << "Arena thermal field initialized with " << thermalCount_
              << " thermals" << std::endl;
#endif
}

void ArenaThermalField::clear()
{
    thermalCount_ = 0;
}

void ArenaThermalField::spawnThermals()
{
    // Determine count: random between min and max
    int count;
    if (config_.count_min == config_.count_max) {
        count = config_.count_min;
    } else {
        double u = static_cast<double>(CRRC_Random::rand()) /
                   (static_cast<double>(CRRC_Random::max()) + 1.0);
        count = config_.count_min +
                static_cast<int>(u * (config_.count_max - config_.count_min + 1));
    }

    count = std::min(count, MAX_THERMALS);
    thermalCount_ = count;

    for (int i = 0; i < thermalCount_; i++) {
        thermals_[i].randomInit(config_);
    }
}

void ArenaThermalField::update(float dt, float wind_vel, float wind_dir_deg)
{
    if (!config_.enabled || thermalCount_ == 0) {
        return;
    }

    // Convert wind to x/y components (ft/s)
    // wind_vel is in m/s from config, convert to ft/s
    double wind_vel_ft = wind_vel * M_TO_FT;
    double wind_rad = wind_dir_deg * M_PI / 180.0;
    float wind_x = static_cast<float>(-wind_vel_ft * cos(wind_rad));
    float wind_y = static_cast<float>(-wind_vel_ft * sin(wind_rad));

    for (int i = 0; i < thermalCount_; i++) {
        if (thermals_[i].update(dt, wind_x, wind_y, config_)) {
            // Respawn this thermal
            thermals_[i].randomInit(config_);
        }
    }
}

void ArenaThermalField::calculateThermalWind(double x, double y, double z,
                                              double& vel_north, double& vel_east,
                                              double& vel_down)
{
    vel_north = vel_east = vel_down = 0.0;

    if (!config_.enabled || thermalCount_ == 0) {
        return;
    }

    for (int i = 0; i < thermalCount_; i++) {
        // Vertical component
        vel_down += thermals_[i].getVerticalVelocity(x, y, z);

        // Horizontal inflow
        double vx, vy;
        thermals_[i].getHorizontalVelocity(x, y, z, vx, vy);
        vel_north += vx;
        vel_east += vy;
    }
}

// ---------- Global functions ----------

void initialize_arena_thermals(SimpleXMLTransfer* locCfg)
{
    // locCfg is the location-specific config node (from getCurLocCfgPtr)
    // Look for arena_thermals section directly within it
    SimpleXMLTransfer* arenaCfg = nullptr;

    if (locCfg) {
        int arenaIdx = locCfg->indexOfChild("arena_thermals");
        if (arenaIdx >= 0) {
            arenaCfg = locCfg->getChildAt(arenaIdx);
        }
    }

    // Create or reinit global instance
    if (!g_arenaThermalField) {
        g_arenaThermalField = new ArenaThermalField();
    }

    ArenaThermalConfig config;
    if (arenaCfg) {
        config.loadFromXML(arenaCfg);
    }

    g_arenaThermalField->initialize(config);
}

void clear_arena_thermals()
{
    if (g_arenaThermalField) {
        g_arenaThermalField->clear();
    }
}
