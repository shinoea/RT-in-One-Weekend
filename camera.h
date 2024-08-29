#pragma once
#ifndef CAMERA_H
#define CAMERA_H

#include "rtweekend.h"

#include "pdf.h"
#include "hittable.h"
#include "material.h"

class camera 
{
public:
    double aspect_ratio = 1.0;  //宽高比
    int    image_width = 100;   //宽像素数
    int    samples_per_pixel = 10;   // 对每个像素随机采样的次数
    int    max_depth = 10;   // 光线在场景中弹射的最大次数
    color  background;               //背景颜色 

    double vfov = 90;  // Vertical view angle
    point3 lookfrom = point3(0, 0, 0);   //相机位置
    point3 lookat = point3(0, 0, -1);  // 相机朝向
    vec3   vup = vec3(0, 1, 0);     // 相机的上方向

    double defocus_angle = 0;  // Variation angle of rays through each pixel
    double focus_dist = 10;    // Distance from camera lookfrom point to plane of perfect focus

    void render(const hittable& world, const hittable& lights)
    {
        initialize();

        FILE* fp = fopen("binary.ppm", "wb");
        (void)fprintf(fp, "P6\n%d %d\n255\n", image_width, image_height);

        for (int j = 0; j < image_height; j++)
        {
            for (int i = 0; i < image_width; i++)
            {
                color pixel_color(0, 0, 0);
                for (int sample = 0; sample < samples_per_pixel; sample++) 
                {
                    //采样有可能命中物体，也有可能未命中，将多次采样的结果平均
                    ray r = get_ray(i, j);
                    pixel_color += ray_color(r, max_depth, world, lights);
                }
                static unsigned char color[3];
                write_color(fp, pixel_samples_scale * pixel_color , color);
                fwrite(color, 1, 3, fp);
            }
        }
        fclose(fp);
    }

private:
    int    image_height;   // 高像素数
    double pixel_samples_scale;  //颜色缩放因子
    point3 center;         // 相机中心
    point3 pixel00_loc;    // 左上角第一个像素的起点
    vec3   pixel_delta_u;  // 一个像素宽的偏移
    vec3   pixel_delta_v;  // 一个像素高的偏移
    vec3   u, v, w;              // Camera frame basis vectors
    vec3   defocus_disk_u;       // Defocus disk horizontal radius
    vec3   defocus_disk_v;       // Defocus disk vertical radius

    void initialize()
    {
        image_height = int(image_width / aspect_ratio);
        image_height = (image_height < 1) ? 1 : image_height;

        pixel_samples_scale = 1.0 / samples_per_pixel;

        center = lookfrom;

        auto theta = degrees_to_radians(vfov);//角度转弧度
        auto h = std::tan(theta / 2);
        auto viewport_height = 2 * h * focus_dist;//视口高度
        auto viewport_width = viewport_height * (double(image_width) / image_height);

        w = normalize(lookfrom - lookat);//相机观察方向
        u = normalize(cross(vup, w));//指向相机右侧的单位向量 u
        v = cross(w, u);//指向相机上方的单位向量 v

        //一个从左到右的向量和一个从上到下的向量
        vec3 viewport_u = viewport_width * u;
        vec3 viewport_v = viewport_height * -v;

        //视窗上一个像素的宽和高
        pixel_delta_u = viewport_u / image_width;
        pixel_delta_v = viewport_v / image_height;

        //计算左上角开始的像素中心位置
        auto viewport_upper_left = center - (focus_dist * w) - viewport_u / 2 - viewport_v / 2;
        pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);

        // Calculate the camera defocus disk basis vectors.
        auto defocus_radius = focus_dist * std::tan(degrees_to_radians(defocus_angle / 2));
        defocus_disk_u = u * defocus_radius;
        defocus_disk_v = v * defocus_radius;
    }

    ray get_ray(int i, int j) const {
        //给定一个点x,y,在其+-0.5的单位正方形偏移内随机采样
        auto offset = sample_square();
        auto pixel_sample = pixel00_loc
            + ((i + offset.x()) * pixel_delta_u)
            + ((j + offset.y()) * pixel_delta_v);

        auto ray_origin = (defocus_angle <= 0) ? center : defocus_disk_sample();
        auto ray_direction = pixel_sample - ray_origin;
        auto ray_time = random_double();

        return ray(ray_origin, ray_direction, ray_time);
    }

    vec3 sample_square() const {
        // 返回一个 [-0.5,-0.5]-[+0.5,+0.5] 的单位正方形.
        return vec3(random_double() - 0.5, random_double() - 0.5, 0);
    }

    point3 defocus_disk_sample() const {
        // Returns a random point in the camera defocus disk.
        auto p = random_in_unit_disk();
        return center + (p[0] * defocus_disk_u) + (p[1] * defocus_disk_v);
    }

    color ray_color(const ray& r, int depth, const hittable& world, const hittable& lights)const {
        //超过反弹次数，则没有颜色
        if (depth <= 0)
            return color(0, 0, 0);

        hit_record rec;

        //光线没有击中物体，返回背景颜色
        if (!world.hit(r, interval(0.001, infinity), rec))
            return background;

        scatter_record srec;
        color color_from_emission = rec.mat->emitted(r, rec, rec.u, rec.v, rec.p);

        if (!rec.mat->scatter(r, rec, srec))
            return color_from_emission;

        if (srec.skip_pdf) {
            return srec.attenuation * ray_color(srec.skip_pdf_ray, depth - 1, world, lights);
        }

        auto light_ptr = make_shared<hittable_pdf>(lights, rec.p);
        mixture_pdf p(light_ptr, srec.pdf_ptr);

        ray scattered = ray(rec.p, p.generate(), r.time());
        auto pdf_value = p.value(scattered.direction());

        double scattering_pdf = rec.mat->scattering_pdf(r, rec, scattered);

        color sample_color = ray_color(scattered, depth - 1, world, lights);
        color color_from_scatter =
            (srec.attenuation * scattering_pdf * sample_color) / pdf_value;

        return color_from_emission + color_from_scatter;
    }
};

#endif
