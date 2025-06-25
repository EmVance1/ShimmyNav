#pragma once

#ifdef DEBUG
#include <iostream>
#include <chrono>
#define BENCH_BEGIN \
    auto _bench_end = std::chrono::time_point<std::chrono::high_resolution_clock>{}; \
    auto _bench_begin = std::chrono::high_resolution_clock::now()
#define BENCH_STEP(name) \
    _bench_end = std::chrono::high_resolution_clock::now(); \
    std::cout << "BENCH STEP \"" name "\" took: " << std::chrono::duration_cast<std::chrono::microseconds>(_bench_end - _bench_begin).count() << " microseconds\n"; \
    _bench_begin = std::chrono::high_resolution_clock::now()
#else
#define BENCH_BEGIN
#define BENCH_STEP(name)
#endif

