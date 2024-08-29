#pragma once
#ifndef VEC3_H
#define VEC3_H

class vec3 
{
public:
	double e[3];

	vec3() :e{ 0,0,0 }{}
	vec3(double e0,double e1,double e2):e{e0,e1,e2}{}

	double x() const { return e[0]; }
	double y() const { return e[1]; }
	double z() const { return e[2]; }

	vec3 operator-()const { return vec3(-e[0], -e[1], -e[2]); }
	double operator[](int i) const { return e[i]; }
	double& operator[](int i) { return e[i]; }

	vec3& operator+=(const vec3& v) {
		e[0] += v.e[0];
		e[1] += v.e[1];
		e[2] += v.e[2];
		return *this;
	}

	vec3& operator*=(double t) {
		e[0] *= t;
		e[1] *= t;
		e[2] *= t;
		return *this;
	}

	vec3& operator/=(double t) {
		return *this *= 1 / t;
	}

	//长度
	double length() const {
		return std::sqrt(length_squared());
	}

	//长度的平方
	double length_squared() const {
		return e[0] * e[0] + e[1] * e[1] + e[2] * e[2];
	}

	//当三维向量的每个维度都接近0时返回true
	bool near_zero() const {
		auto s = 1e-8;
		return (std::fabs(e[0]) < s) && (std::fabs(e[1]) < s) && (std::fabs(e[2]) < s);
	}

	//生成随机三维向量
	static vec3 random() {
		return vec3(random_double(), random_double(), random_double());
	}

	static vec3 random(double min, double max) {
		return vec3(random_double(min, max), random_double(min, max), random_double(min, max));
	}
};

using point3 = vec3;

inline std::ostream& operator<<(std::ostream& out, const vec3& v) {
	return out << v.e[0] << ' ' << v.e[1] << ' ' << v.e[2];
}

inline vec3 operator+(const vec3& u, const vec3& v) {
	return vec3(u.e[0] + v.e[0], u.e[1] + v.e[1], u.e[2] + v.e[2]);
}

inline vec3 operator-(const vec3& u, const vec3& v) {
	return vec3(u.e[0] - v.e[0], u.e[1] - v.e[1], u.e[2] - v.e[2]);
}

inline vec3 operator*(const vec3& u, const vec3& v) {
	return vec3(u.e[0] * v.e[0], u.e[1] * v.e[1], u.e[2] * v.e[2]);
}

inline vec3 operator*(double t, const vec3& v) {
	return vec3(t * v.e[0], t * v.e[1], t * v.e[2]);
}

inline vec3 operator*(const vec3& v, double t) {
	return t * v;
}

inline vec3 operator/(const vec3& v, double t) {
	return (1 / t) * v;
}

inline double dot(const vec3& u, const vec3& v) {
	return u.e[0] * v.e[0]
		+ u.e[1] * v.e[1]
		+ u.e[2] * v.e[2];
}

inline vec3 cross(const vec3& u, const vec3& v) {
	return vec3(u.e[1] * v.e[2] - u.e[2] * v.e[1],
		u.e[2] * v.e[0] - u.e[0] * v.e[2],
		u.e[0] * v.e[1] - u.e[1] * v.e[0]);
}

//单位向量
inline vec3 normalize(const vec3& v) {
	return v / v.length();
}

inline vec3 random_in_unit_disk() {
	while (true) {
		auto p = vec3(random_double(-1, 1), random_double(-1, 1), 0);
		if (p.length_squared() < 1)
			return p;
	}
}

//生成一个单位圆内的随机向量
inline vec3 random_in_unit_sphere() {
	while (true) {
		auto p = vec3::random(-1, 1);
		if (p.length_squared() < 1)
			return p;
	}
}

//将随机向量单位化
inline vec3 random_unit_vector() {
	return normalize(random_in_unit_sphere());
}

//单位向量和法线点乘判断角度
inline vec3 random_on_hemisphere(const vec3& normal) {
	vec3 on_unit_sphere = random_unit_vector();
	if (dot(on_unit_sphere, normal) > 0.0) //方向正确
		return on_unit_sphere;
	else
		return -on_unit_sphere;
}

inline vec3 random_cosine_direction() {
	auto r1 = random_double();
	auto r2 = random_double();

	auto phi = 2 * pi * r1;
	auto x = std::cos(phi) * std::sqrt(r2);
	auto y = std::sin(phi) * std::sqrt(r2);
	auto z = std::sqrt(1 - r2);

	return vec3(x, y, z);
}

//计算反射向量
inline vec3 reflect(const vec3& v, const vec3& n) {
	return v - 2 * dot(v, n) * n;
}

//计算折射向量
inline vec3 refract(const vec3& uv, const vec3& n, double etai_over_etat) {
	//入射光线和法线的余弦值
	auto cos_theta = std::fmin(dot(-uv, n), 1.0);
	vec3 r_out_perp = etai_over_etat * (uv + cos_theta * n);
	vec3 r_out_parallel = -std::sqrt(std::fabs(1.0 - r_out_perp.length_squared())) * n;
	return r_out_perp + r_out_parallel;
}

#endif

