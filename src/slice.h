#pragma once
#include <vector>


template<typename T>
struct Slice {
    const T* ptr;
    size_t len;

    static Slice from_vec(const std::vector<T>& vec) { return Slice{ vec.data(), vec.size() }; }
    std::vector<T> into_vec() const {
        auto result = std::vector<T>();
        for (size_t i = 0; i < len; i++) { result.push_back(ptr[i]); }
        return result;
    }

    const T& operator[](size_t idx) const { return ptr[idx]; }

    Slice slice(size_t begin, size_t end) const { return Slice{ ptr + begin, end - begin }; }
};

