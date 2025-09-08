// ─────────────────────────────────────────────────────────────
// range_utils.h — header-only helpers for std::vector sequences
// David-friendly Allman style (braces on their own lines)
// ─────────────────────────────────────────────────────────────
#pragma once
#include <vector>
#include <type_traits>
#include <stdexcept>
#include <cstddef>
#include <limits>

// Internal helper: safe size computation for exclusive range [start, end)
// with arbitrary (non-zero) step. Handles ascending and descending.
template <typename T>
inline std::size_t _ru_count_exclusive(T start, T end, T step)
{
    if (step == T(0))
    {
        throw std::invalid_argument("step must not be zero");
    }

    // If we’re already past end in the movement direction → empty.
    if ((step > T(0) && start >= end) || (step < T(0) && start <= end))
    {
        return 0;
    }

    // Number of steps: ceil((end - start) / step) for positive step
    // For integers, do a division that rounds toward +infinity for positive step,
    // and toward +infinity in magnitude for negative step; easiest is to do the
    // math in “distance along step” terms.
    // We avoid floating-point here for determinism.
    const auto dist = (step > T(0)) ? (end - start) : (start - end);
    const auto s    = (step > T(0)) ? step : -step;

    // ceil(dist / s) for non-negative integers:
    return static_cast<std::size_t>((dist + (s - T(1))) / s);
}

// Internal helper: safe size computation for inclusive range [start, end]
// with arbitrary (non-zero) step. Handles ascending and descending.
template <typename T>
inline std::size_t _ru_count_inclusive(T start, T end, T step)
{
    if (step == T(0))
    {
        throw std::invalid_argument("step must not be zero");
    }

    // Direction mismatch → at most one element if start == end, else empty.
    if ((step > T(0) && start > end) || (step < T(0) && start < end))
    {
        return (start == end) ? 1u : 0u;
    }

    // Example: [0, 10] step 2 → 0,2,4,6,8,10 → ((10-0)/2)+1 = 6
    // For descending: [10, 0] step -2 → 10,8,6,4,2,0 → ((10-0)/2)+1 = 6
    const auto dist = (step > T(0)) ? (end - start) : (start - end);
    const auto s    = (step > T(0)) ? step : -step;
    return static_cast<std::size_t>(dist / s) + 1u;
}

// Make an exclusive arithmetic sequence: [start, end) with stride `step`.
// Works for ascending (step > 0) and descending (step < 0).
template <typename T>
inline std::vector<T> make_range_exclusive(T start, T end, T step = T(1))
{
    static_assert(std::is_arithmetic<T>::value, "T must be arithmetic");
    const std::size_t n = _ru_count_exclusive(start, end, step);
    std::vector<T> out;
    out.reserve(n);

    for (T v = start; ((step > T(0)) ? (v < end) : (v > end)); v += step)
    {
        out.push_back(v);
    }
    return out;
}

// Make an inclusive arithmetic sequence: [start, end] with stride `step`.
// Works for ascending (step > 0) and descending (step < 0).
template <typename T>
inline std::vector<T> make_range_inclusive(T start, T end, T step = T(1))
{
    static_assert(std::is_arithmetic<T>::value, "T must be arithmetic");
    const std::size_t n = _ru_count_inclusive(start, end, step);
    std::vector<T> out;
    out.reserve(n);

    if (n == 0)
    {
        return out;
    }

    for (T v = start;; v += step)
    {
        out.push_back(v);
        if (v == end)
        {
            break;
        }
        // Safety bail-out if step overshoots due to non-integer arithmetic
        if ((step > T(0) && v > end) || (step < T(0) && v < end))
        {
            break;
        }
    }
    return out;
}

// In-place filler for an existing vector: write consecutive values starting at `start`.
template <typename T>
inline void fill_iota(std::vector<T> &v, T start = T(0))
{
    static_assert(std::is_arithmetic<T>::value, "T must be arithmetic");
    T current = start;
    for (auto &x : v)
    {
        x = current;
        current = static_cast<T>(current + T(1));
    }
}

// In-place filler with stride: v[i] = start + i*step
template <typename T>
inline void fill_stride(std::vector<T> &v, T start, T step)
{
    static_assert(std::is_arithmetic<T>::value, "T must be arithmetic");
    T current = start;
    for (auto &x : v)
    {
        x = current;
        current = static_cast<T>(current + step);
    }
}

// Build a vector of size N with v[i] = start + i*step (handy when you know N)
template <typename T>
inline std::vector<T> make_by_count(std::size_t N, T start = T(0), T step = T(1))
{
    std::vector<T> out;
    out.resize(N);
    fill_stride(out, start, step);
    return out;
}

// Map an input vector to a new vector via a unary function (useful for LUTs)
template <typename InT, typename Fn>
inline auto map_to_vector(const std::vector<InT> &in, Fn f)
    -> std::vector<decltype(f(std::declval<InT>()))>
{
    using OutT = decltype(f(std::declval<InT>()));
    std::vector<OutT> out;
    out.resize(in.size());
    for (std::size_t i = 0; i < in.size(); ++i)
    {
        out[i] = f(in[i]);
    }
    return out;
}


/*
// ─────────────────────────────────────────────────────────────
// examples_ranges.cpp — usage examples tailored to your LUTs
// ─────────────────────────────────────────────────────────────
#include <cstdio>
#include <vector>
#include "range_utils.h"

int main()
{
    // 1) Duty = 0..1023 inclusive (exactly 1024 elements)
    auto duty_0_1023 = make_range_inclusive<int>(0, 1023);
    std::printf("duty_0_1023.size() = %zu, first=%d, last=%d\n",
                duty_0_1023.size(), duty_0_1023.front(), duty_0_1023.back());

    // 2) Duty = [0, 1024) exclusive — same as 0..1023
    auto duty_excl = make_range_exclusive<int>(0, 1024);
    std::printf("duty_excl.size()   = %zu, first=%d, last=%d\n",
                duty_excl.size(), duty_excl.front(), duty_excl.back());

    // 3) Subrange with stride: 376..1023 inclusive, step 1
    auto duty_376_1023 = make_range_inclusive<int>(376, 1023, 1);

    // 4) Descending: 1023..0 inclusive, step -1
    auto duty_desc = make_range_inclusive<int>(1023, 0, -1);

    // 5) Fixed count builder: exactly 1024 values, start=0, step=1
    auto duty_by_count = make_by_count<int>(1024, 0, 1);

    // 6) Fill a preallocated vector in place
    std::vector<int> duty_fixed(1024);
    fill_iota(duty_fixed, 0);               // 0..1023
    fill_stride(duty_fixed, 376, 3);        // 376,379,382,...

    // 7) Build a float LUT by mapping duty values
    //    Example: normalize duty to [0,1] using 10-bit max 1023
    auto lut_norm = map_to_vector(duty_0_1023,
                                  [](int d)
                                  {
                                      return static_cast<float>(d) / 1023.0f;
                                  });

    // 8) Another LUT: piecewise (toy example)
    auto lut_piecewise = map_to_vector(duty_0_1023,
                                       [](int d)
                                       {
                                           if (d < 400)      return 0.1f * d;         // region 1
                                           else if (d < 800) return 40.0f + 0.05f*d;  // region 2
                                           else              return 80.0f + 0.02f*d;  // region 3
                                       });

    // 9) Typical coil-winder: ticks per 50ms model from duty (placeholder)
    //    You’d replace the function with your empirical or fitted model.
    auto ticks50ms_model = map_to_vector(duty_0_1023,
                                         [](int duty)
                                         {
                                             // example: nonlinear-ish curve
                                             // Avoid division by zero; clamp inputs as needed
                                             const float x = static_cast<float>(duty) / 1023.0f;
                                             return 1500.0f * x + 800.0f * x * x; // fake model
                                         });

    std::printf("Example: lut_norm[0]=%.3f, lut_norm[1023]=%.3f\n",
                lut_norm.front(), lut_norm.back());

    return 0;
}
*/
