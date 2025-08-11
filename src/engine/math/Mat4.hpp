#pragma once

#include <cmath>
#include <array>

namespace Engine::Math
{

    struct Vec3
    {
        float x{0.0f};
        float y{0.0f};
        float z{0.0f};

        Vec3() = default;
        Vec3(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}
    };

    inline Vec3 operator+(const Vec3 &a, const Vec3 &b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
    inline Vec3 operator-(const Vec3 &a, const Vec3 &b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
    inline Vec3 operator*(const Vec3 &a, float s) { return {a.x * s, a.y * s, a.z * s}; }
    inline Vec3 operator/(const Vec3 &a, float s) { return {a.x / s, a.y / s, a.z / s}; }

    inline float dot(const Vec3 &a, const Vec3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    inline Vec3 cross(const Vec3 &a, const Vec3 &b)
    {
        return {a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x};
    }
    inline float length(const Vec3 &v) { return std::sqrt(dot(v, v)); }
    inline Vec3 normalize(const Vec3 &v)
    {
        float len = length(v);
        if (len <= 0.0f)
            return {0.0f, 0.0f, 0.0f};
        return v / len;
    }

    // Column-major 4x4 matrix (graphics standard). m[col*4 + row]
    struct Mat4
    {
        std::array<float, 16> m{1, 0, 0, 0,
                                0, 1, 0, 0,
                                0, 0, 1, 0,
                                0, 0, 0, 1};
    };

    inline Mat4 identity() { return Mat4{}; }

    inline Mat4 multiply(const Mat4 &a, const Mat4 &b)
    {
        Mat4 r;
        for (int c = 0; c < 4; ++c)
        {
            for (int rIdx = 0; rIdx < 4; ++rIdx)
            {
                r.m[c * 4 + rIdx] = a.m[0 * 4 + rIdx] * b.m[c * 4 + 0] + a.m[1 * 4 + rIdx] * b.m[c * 4 + 1] + a.m[2 * 4 + rIdx] * b.m[c * 4 + 2] + a.m[3 * 4 + rIdx] * b.m[c * 4 + 3];
            }
        }
        return r;
    }

    inline Mat4 translate(const Vec3 &t)
    {
        Mat4 r = identity();
        r.m[12] = t.x; // 3rd column, row 0
        r.m[13] = t.y; // row 1
        r.m[14] = t.z; // row 2
        return r;
    }

    inline Mat4 scale(const Vec3 &s)
    {
        Mat4 r{};
        r.m = {s.x, 0, 0, 0,
               0, s.y, 0, 0,
               0, 0, s.z, 0,
               0, 0, 0, 1};
        return r;
    }

    inline Mat4 rotateX(float rad)
    {
        float c = std::cos(rad), s = std::sin(rad);
        Mat4 r{};
        r.m = {1, 0, 0, 0,
               0, c, s, 0,
               0, -s, c, 0,
               0, 0, 0, 1};
        return r;
    }

    inline Mat4 rotateY(float rad)
    {
        float c = std::cos(rad), s = std::sin(rad);
        Mat4 r{};
        r.m = {c, 0, -s, 0,
               0, 1, 0, 0,
               s, 0, c, 0,
               0, 0, 0, 1};
        return r;
    }

    inline Mat4 rotateZ(float rad)
    {
        float c = std::cos(rad), s = std::sin(rad);
        Mat4 r{};
        r.m = {c, s, 0, 0,
               -s, c, 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1};
        return r;
    }

    inline Mat4 lookAt(const Vec3 &eye, const Vec3 &target, const Vec3 &up)
    {
        Vec3 f = normalize(target - eye);
        Vec3 s = normalize(cross(f, up));
        Vec3 u = cross(s, f);

        Mat4 r{};
        r.m = {
            s.x, u.x, -f.x, 0.0f,
            s.y, u.y, -f.y, 0.0f,
            s.z, u.z, -f.z, 0.0f,
            -dot(s, eye), -dot(u, eye), dot(f, eye), 1.0f};
        return r;
    }

    inline Mat4 perspective(float fovYRadians, float aspect, float zNear, float zFar)
    {
        float f = 1.0f / std::tan(fovYRadians * 0.5f);
        Mat4 r{};
        r.m = {
            f / aspect, 0, 0, 0,
            0, f, 0, 0,
            0, 0, (zFar + zNear) / (zNear - zFar), -1,
            0, 0, (2 * zFar * zNear) / (zNear - zFar), 0};
        return r;
    }

    inline Mat4 eulerXYZ(const Vec3 &rad)
    {
        return multiply(multiply(rotateX(rad.x), rotateY(rad.y)), rotateZ(rad.z));
    }

    inline Mat4 composeTRS(const Vec3 &t, const Vec3 &radEuler, const Vec3 &s)
    {
        return multiply(multiply(translate(t), eulerXYZ(radEuler)), scale(s));
    }

} // namespace Engine::Math
