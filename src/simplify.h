#pragma once
#include "shapes.h"


namespace nav {

std::vector<Vector2i> douglas_peucker(const std::vector<Vector2i>& segment, f32 epsilon);
std::vector<Vector2f> douglas_peucker(const std::vector<Vector2f>& segment, f32 epsilon);

}

