#pragma once
#include "gen.h"


namespace nav {

using Polygon = std::vector<Vector2i>;

std::vector<Polygon> marching_squares(const uint8_t* data, size_t width, size_t height, size_t stride, Predicate P);
std::vector<Polygon> marching_squares(const uint8_t* data, size_t width, size_t height, size_t stride, size_t index);
std::vector<Polygon> floodfill(const uint8_t* data, size_t width, size_t height, size_t stride, Predicate P);
std::vector<Polygon> floodfill(const uint8_t* data, size_t width, size_t height, size_t stride, size_t index);
std::vector<Polygon> floodfill_threaded(const u8* data, size_t width, size_t height, size_t stride, size_t index);

}
