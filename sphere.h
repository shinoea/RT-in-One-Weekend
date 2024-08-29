#pragma once
#ifndef SPHERE_H
#define SPHERE_H

#include "hittable.h"
#include "onb.h"

class sphere :public hittable
{
public:
    //静态球
    sphere(const point3& center, double radius, shared_ptr<material> mat)
        : center1(center), radius(std::fmax(0, radius)), mat(mat), is_moving(false)
    {
        auto rvec = vec3(radius, radius, radius);
        //初始化球体包围盒
        bbox = aabb(center1 - rvec, center1 + rvec);
    }

    //移动中的球
    sphere(const point3& center1, const point3& center2, double radius,
        shared_ptr<material> mat)
        : center1(center1), radius(std::fmax(0, radius)), mat(mat), is_moving(true)
    {
        auto rvec = vec3(radius, radius, radius);
        aabb box1(center1 - rvec, center1 + rvec);
        aabb box2(center2 - rvec, center2 + rvec);
        bbox = aabb(box1, box2);
    }

	//判断光线是否和球相交
    bool hit(const ray& r, interval ray_t, hit_record& rec) const override 
    {
        point3 center = is_moving ? sphere_center(r.time()) : center1;
        vec3 oc = center - r.origin();
        auto a = r.direction().length_squared();
        auto h = dot(r.direction(), oc);
        auto c = oc.length_squared() - radius * radius;

        auto discriminant = h * h - a * c;
        if (discriminant < 0)
            return false;

        auto sqrtd = std::sqrt(discriminant);

        auto root = (h - sqrtd) / a;
        if (!ray_t.surrounds(root)) {
            root = (h + sqrtd) / a;
            if (!ray_t.surrounds(root))
                return false;
        }

        rec.t = root;
        rec.p = r.at(rec.t);
        vec3 outward_normal = (rec.p - center) / radius;
        rec.set_face_normal(r, outward_normal);
        get_sphere_uv(outward_normal, rec.u, rec.v);
        rec.mat = mat;

        return true;
    }

    aabb bounding_box() const override { return bbox; }

    double pdf_value(const point3& origin, const vec3& direction) const override {
        // This method only works for stationary spheres.

        hit_record rec;
        if (!this->hit(ray(origin, direction), interval(0.001, infinity), rec))
            return 0;

        auto cos_theta_max = std::sqrt(1 - radius * radius / (center1 - origin).length_squared());
        auto solid_angle = 2 * pi * (1 - cos_theta_max);

        return  1 / solid_angle;
    }

    vec3 random(const point3& origin) const override {
        vec3 direction = center1 - origin;
        auto distance_squared = direction.length_squared();
        onb uvw(direction);
        return uvw.transform(random_to_sphere(radius, distance_squared));
    }

private:
    point3 center1;
	double radius;
    shared_ptr<material> mat;
    bool is_moving;
    vec3 center_vec;//球移动方向
    aabb bbox;//包围盒

    point3 sphere_center(double time) const {
        return center1 + time * center_vec;
    }

    static void get_sphere_uv(const point3& p, double& u, double& v) {
        //把一个空间坐标映射到球坐标上
        auto theta = std::acos(-p.y());
        auto phi = std::atan2(-p.z(), p.x()) + pi;

        u = phi / (2 * pi);
        v = theta / pi;
    }

    static vec3 random_to_sphere(double radius, double distance_squared) {
        auto r1 = random_double();
        auto r2 = random_double();
        auto z = 1 + r2 * (std::sqrt(1 - radius * radius / distance_squared) - 1);

        auto phi = 2 * pi * r1;
        auto x = std::cos(phi) * std::sqrt(1 - z * z);
        auto y = std::sin(phi) * std::sqrt(1 - z * z);

        return vec3(x, y, z);
    }
};

#endif