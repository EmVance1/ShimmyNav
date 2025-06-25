#pragma once
#include "types.h"


namespace nav {

using Predicate = bool(*)(const u8*);

enum class Method {
    MARCHING_SQUARES,
    FLOODFILL,
};

/*
std::vector<IntTriangle> generate_earclip(
        const uint8_t* data,
        size_t width,
        size_t height,
        size_t stride,
        Predicate P,
        Method method,
        float epsilon
    );
*/

NavMesh generate_delauney(
        const u8* grid,
        usize width,
        usize height,
        usize stride,
        Predicate P,
        Method method,
        f32 epsilon
    );

NavMesh generate_delauney(
        const u8* grid,
        usize width,
        usize height,
        usize stride,
        usize index,
        Method method,
        f32 epsilon
    );

NavMesh generate_from_shapes(
        const std::vector<std::vector<Vector2f>>& polys,
        const std::vector<FloatCircle>& circles,
        const std::vector<Vector2f>& fillers,
        u32 circle_res = 8,
        f32 epsilon = 0.f
    );

}
