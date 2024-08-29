#pragma once
#ifndef HITTABLE_H
#define HITTABLE_H
#include "rtweekend.h"

#include "aabb.h"

class material;

class hit_record {
public:
    point3 p;//击中点
    vec3 normal;//击中点的法线
    shared_ptr<material> mat;//材质
    double t;//击中时间
    double u;//纹理坐标
    double v;
    bool front_face;//true代表光线从外部射入

    void set_face_normal(const ray& r, const vec3& outward_normal) {
        //判断光线是从内部还是外部射入
        front_face = dot(r.direction(), outward_normal) < 0;
        //如果从外部射入，则法线朝外，否则朝内
        normal = front_face ? outward_normal : -outward_normal;
    }
};

class hittable {
public:
    virtual ~hittable() = default;

    virtual bool hit(const ray& r, interval ray_t, hit_record& rec) const = 0;

    virtual aabb bounding_box() const = 0;

    virtual double pdf_value(const point3& origin, const vec3& direction) const {
        return 0.0;
    }

    virtual vec3 random(const point3& origin) const {
        return vec3(1, 0, 0);
    }
};

class translate : public hittable {
public:
    translate(shared_ptr<hittable> object, const vec3& offset)
        : object(object), offset(offset)
    {
        bbox = object->bounding_box() + offset;
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        //对光线进行偏移
        ray offset_r(r.origin() - offset, r.direction(), r.time());

        //检查光线偏移后是否能击中物体
        if (!object->hit(offset_r, ray_t, rec))
            return false;

        //移动交叉点
        rec.p += offset;

        return true;
    }

    aabb bounding_box() const override { return bbox; }

private:
    shared_ptr<hittable> object;
    vec3 offset;
    aabb bbox;
};

//沿着y轴旋转
class rotate_y : public hittable {
public:
    rotate_y(shared_ptr<hittable> object, double angle) : object(object) {
        auto radians = degrees_to_radians(angle);
        sin_theta = std::sin(radians);
        cos_theta = std::cos(radians);
        bbox = object->bounding_box();

        point3 min(infinity, infinity, infinity);
        point3 max(-infinity, -infinity, -infinity);

        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                for (int k = 0; k < 2; k++) {
                    //对于包围盒的每个点
                    auto x = i * bbox.x.max + (1 - i) * bbox.x.min;
                    auto y = j * bbox.y.max + (1 - j) * bbox.y.min;
                    auto z = k * bbox.z.max + (1 - k) * bbox.z.min;
                    //进行旋转
                    auto newx = cos_theta * x + sin_theta * z;
                    auto newz = -sin_theta * x + cos_theta * z;

                    vec3 tester(newx, y, newz);

                    //重新记录包围盒
                    for (int c = 0; c < 3; c++) {
                        min[c] = std::fmin(min[c], tester[c]);
                        max[c] = std::fmax(max[c], tester[c]);
                    }
                }
            }
        }

        bbox = aabb(min, max);
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        //把光线从世界坐标系转换到物体坐标系
        auto origin = r.origin();
        auto direction = r.direction();

        //把光线逆时针旋转-theta度
        origin[0] = cos_theta * r.origin()[0] - sin_theta * r.origin()[2];
        origin[2] = sin_theta * r.origin()[0] + cos_theta * r.origin()[2];

        direction[0] = cos_theta * r.direction()[0] - sin_theta * r.direction()[2];
        direction[2] = sin_theta * r.direction()[0] + cos_theta * r.direction()[2];

        ray rotated_r(origin, direction, r.time());

        // Determine whether an intersection exists in object space (and if so, where)
        if (!object->hit(rotated_r, ray_t, rec))
            return false;

        //把相交点从物体坐标系转换到世界坐标系
        auto p = rec.p;
        p[0] = cos_theta * rec.p[0] + sin_theta * rec.p[2];
        p[2] = -sin_theta * rec.p[0] + cos_theta * rec.p[2];

        //把法线从物体坐标系转换到世界坐标系
        auto normal = rec.normal;
        normal[0] = cos_theta * rec.normal[0] + sin_theta * rec.normal[2];
        normal[2] = -sin_theta * rec.normal[0] + cos_theta * rec.normal[2];

        rec.p = p;
        rec.normal = normal;

        return true;
    }

    aabb bounding_box() const override { return bbox; }

private:
    shared_ptr<hittable> object;
    double sin_theta;
    double cos_theta;
    aabb bbox;
};

#endif
