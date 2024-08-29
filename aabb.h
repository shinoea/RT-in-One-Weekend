#pragma once
#ifndef AABB_H
#define AABB_H

#include "rtweekend.h"

class aabb {
public:
    //三个对面形成一个包围盒
    interval x, y, z;

    aabb() {}

    aabb(const interval& x, const interval& y, const interval& z)
        : x(x), y(y), z(z)
    {
        pad_to_minimums();
    }

    //用两个点代表三个对面
    aabb(const point3& a, const point3& b) {
        x = (a[0] <= b[0]) ? interval(a[0], b[0]) : interval(b[0], a[0]);
        y = (a[1] <= b[1]) ? interval(a[1], b[1]) : interval(b[1], a[1]);
        z = (a[2] <= b[2]) ? interval(a[2], b[2]) : interval(b[2], a[2]);

        pad_to_minimums();
    }

    //求两个包围盒的并集
    aabb(const aabb& box0, const aabb& box1) {
        x = interval(box0.x, box1.x);
        y = interval(box0.y, box1.y);
        z = interval(box0.z, box1.z);
    }

    const interval& axis_interval(int n) const {
        if (n == 1) return y;
        if (n == 2) return z;
        return x;
    }

    //光线和包围盒求交
    bool hit(const ray& r, interval ray_t) const {
        const point3& ray_orig = r.origin();
        const vec3& ray_dir = r.direction();

        for (int axis = 0; axis < 3; axis++) {
            //取得两个对面
            const interval& ax = axis_interval(axis);
            const double adinv = 1.0 / ray_dir[axis];

            auto t0 = (ax.min - ray_orig[axis]) * adinv;
            auto t1 = (ax.max - ray_orig[axis]) * adinv;

            //tenter=max(tmin) texit=min(tmax)
            //相交条件：tenter<texit && texit>=0
            if (t0 < t1) {
                if (t0 > ray_t.min) ray_t.min = t0;
                if (t1 < ray_t.max) ray_t.max = t1;
            }
            else {
                if (t1 > ray_t.min) ray_t.min = t1;
                if (t0 < ray_t.max) ray_t.max = t0;
            }

            if (ray_t.max <= ray_t.min)
                return false;
        }
        return true;
    }

    int longest_axis() const {
        if (x.size() > y.size())
            return x.size() > z.size() ? 0 : 2;
        else
            return y.size() > z.size() ? 1 : 2;
    }

    static const aabb empty, universe;

private:
    //阻止包围盒体积为0的情况，对包围盒进行微小的填充
    void pad_to_minimums() {
        double delta = 0.0001;
        if (x.size() < delta) x = x.expand(delta);
        if (y.size() < delta) y = y.expand(delta);
        if (z.size() < delta) z = z.expand(delta);
    }
};

const aabb aabb::empty = aabb(interval::empty, interval::empty, interval::empty);
const aabb aabb::universe = aabb(interval::universe, interval::universe, interval::universe);

aabb operator+(const aabb& bbox, const vec3& offset) {
    return aabb(bbox.x + offset.x(), bbox.y + offset.y(), bbox.z + offset.z());
}

aabb operator+(const vec3& offset, const aabb& bbox) {
    return bbox + offset;
}

#endif
