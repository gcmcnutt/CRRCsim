/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2026 Arena Thermal System for GP Training
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * Arena-bounded thermal system for efficient GP training scenarios.
 * Replaces the global 12.8km grid with a small bounded arena (~150-300m).
 */

#ifndef ARENA_THERMAL_H
#define ARENA_THERMAL_H

#include "../mod_misc/SimpleXMLTransfer.h"
#include "../mod_math/vector3.h"
#include <vector>
#include <cstdint>

/**
 * Configuration for arena-bounded thermal field.
 * All distances in meters (converted to feet internally for physics).
 */
struct ArenaThermalConfig {
    bool enabled;           // Master enable for arena thermals

    // Spawn bounds (thermals can spawn here, may drift out)
    float spawn_x_min;      // meters
    float spawn_x_max;
    float spawn_y_min;
    float spawn_y_max;

    // Thermal count range
    int count_min;
    int count_max;

    // Thermal parameters (Gaussian distributions)
    float strength_mean;    // m/s vertical velocity at core
    float strength_sigma;
    float radius_mean;      // meters
    float radius_sigma;
    float lifetime_mean;    // seconds
    float lifetime_sigma;

    // Height parameters
    float height_m;         // thermal column height in meters

    // Default constructor with sensible values for ~150m arena
    ArenaThermalConfig();

    // Load from XML element
    void loadFromXML(SimpleXMLTransfer* el);
};

/**
 * Single thermal in the arena system.
 * Simplified from global Thermal class - no grid tracking needed.
 */
class ArenaThermal {
public:
    float center_x;         // position in feet (world coords)
    float center_y;
    float radius;           // feet
    float strength;         // ft/s upward velocity
    float lifetime;         // seconds remaining
    float initial_lifetime; // for fade-out calculation

    ArenaThermal();

    /**
     * Initialize with random values within config bounds.
     * Uses deterministic PRNG for reproducibility.
     */
    void randomInit(const ArenaThermalConfig& config);

    /**
     * Update thermal position (wind drift) and lifetime.
     * Returns true if thermal should be respawned.
     */
    bool update(float dt, float wind_vel_x, float wind_vel_y,
                const ArenaThermalConfig& config);

    /**
     * Calculate vertical velocity contribution at given position.
     * Uses simplified bell-curve profile (no v3 shell model for efficiency).
     * Position in feet, returns ft/s.
     */
    double getVerticalVelocity(double x, double y, double z) const;

    /**
     * Calculate horizontal inflow velocity (vacuum cleaner effect).
     * Position in feet, returns components in ft/s.
     */
    void getHorizontalVelocity(double x, double y, double z,
                               double& vel_x, double& vel_y) const;
};

/**
 * Arena-bounded thermal field manager.
 * Efficient replacement for global thermal grid when arena mode is enabled.
 */
class ArenaThermalField {
public:
    static constexpr int MAX_THERMALS = 8;

    ArenaThermalField();
    ~ArenaThermalField();

    /**
     * Initialize from config. Call on simulation reset.
     */
    void initialize(const ArenaThermalConfig& config);

    /**
     * Clear all thermals.
     */
    void clear();

    /**
     * Update all thermals (drift, aging, respawn).
     * wind_vel in m/s, dt in seconds.
     */
    void update(float dt, float wind_vel, float wind_dir_deg);

    /**
     * Calculate total thermal contribution at position.
     * Position in feet, velocities returned in ft/s.
     */
    void calculateThermalWind(double x, double y, double z,
                              double& vel_north, double& vel_east, double& vel_down);

    /**
     * Check if arena thermal system is active.
     */
    bool isEnabled() const { return config_.enabled && thermalCount_ > 0; }

    /**
     * Get current thermal count (for debugging/logging).
     */
    int getThermalCount() const { return thermalCount_; }

    /**
     * Get config (for debugging).
     */
    const ArenaThermalConfig& getConfig() const { return config_; }

private:
    ArenaThermalConfig config_;
    ArenaThermal thermals_[MAX_THERMALS];
    int thermalCount_;

    /**
     * Spawn thermals according to config.
     */
    void spawnThermals();
};

/**
 * Global arena thermal field instance.
 * Accessed by windfield.cpp's calculate_wind().
 */
extern ArenaThermalField* g_arenaThermalField;

/**
 * Initialize arena thermal system from config XML.
 * Called from initialize_wind_field() if arena_thermals section present.
 */
void initialize_arena_thermals(SimpleXMLTransfer* cfgfile);

/**
 * Clear arena thermal system.
 */
void clear_arena_thermals();

#endif // ARENA_THERMAL_H
