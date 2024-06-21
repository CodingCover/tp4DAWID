#pragma once

#include <memory>

#include <SDL.h>
#include <SDL2_gfx/SDL2_gfxPrimitives.h>

#include "planar_quadrotor.h"

class PlanarQuadrotorVisualizer {
private:
    PlanarQuadrotor *quadrotor_ptr;
    const float screenWidthMeters;
    const float screenHeightMeters;
    const int screenWidthPixels;
    const int screenHeightPixels;
public:
    PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr, float screenWidthMeters, float screenHeightMeters, int screenWidthPixels, int screenHeightPixels);
    void render(std::shared_ptr<SDL_Renderer> &gRenderer);
};