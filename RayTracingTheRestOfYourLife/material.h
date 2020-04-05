#pragma once

#include "ray.h"
#include "texture.h"

class material
{
public:
	virtual bool scatter(const ray& r_in, const hit_record& rec, vec3& attenuation, ray& scattered, float& pdf) const = 0;
	virtual float scattering_pdf(const ray& r_in, const hit_record& rec, ray& scattered) const
	{
		return 1.f;
	}
	virtual vec3 emitted(float u, float v, const vec3& p) const { return vec3(0, 0, 0); }
};


class lambertian : public material
{
public:
	lambertian(texture* a) : albedo(a) {}
	virtual float scattering_pdf(const ray& r_in, const hit_record& rec, ray& scattered) const override
	{
		float cosine = dot(rec.normal, unit_vector(scattered.direction()));
		if (cosine < 0) cosine = 0;
		return cosine / M_PI;
	}

	virtual bool scatter(const ray& r_in, const hit_record& rec, vec3& attenuation, ray& scattered, float& pdf) const override
	{
		vec3 target = rec.p + rec.normal + random_in_unit_sphere();
		scattered = ray(rec.p, target - rec.p, r_in.time());
		attenuation = albedo->value(rec.u, rec.v, rec.p);
		pdf = dot(rec.normal, scattered.direction()) / M_PI;
		return true;
	}

	texture*  albedo;
};

class metal : public material
{
public:
	metal(const vec3& a, float f) : albedo(a) { if (f < 1) fuzz = f; else fuzz = 1; }
	virtual bool scatter(const ray& r_in, const hit_record& rec, vec3& attenuation, ray& scattered, float& pdf) const
	{
		vec3 reflected = reflect(unit_vector(r_in.direction()), rec.normal);
		scattered = ray(rec.p, reflected + fuzz*random_in_unit_sphere(), r_in.time());
		attenuation = albedo;
		return (dot(scattered.direction(), rec.normal) > 0);
	}

	vec3 albedo;
	float fuzz;
};

class dielectric : public material
{
public:
	dielectric(float ri) : ref_idx(ri) {}
	virtual bool scatter(const ray& r_in, const hit_record& rec, vec3& attenuation, ray& scattered, float& pdf) const
	{
		vec3 outward_normal;
		vec3 reflected = reflect(r_in.direction(), rec.normal);
		float ni_over_nt;
		attenuation = vec3(1.f, 1.f, 1.f);
		vec3 refracted;
		float reflect_prob;
		float cosine;
		if (dot(r_in.direction(), rec.normal) > 0)
		{
			outward_normal = -rec.normal;
			ni_over_nt = ref_idx;
			cosine = ref_idx * dot(r_in.direction(), rec.normal) / r_in.direction().length();
		}
		else
		{
			outward_normal = rec.normal;
			ni_over_nt = 1.f / ref_idx;
			cosine = -dot(r_in.direction(), rec.normal) / r_in.direction().length();
		}
		if (refract(r_in.direction(), outward_normal, ni_over_nt, refracted))
		{
			reflect_prob = schlick(cosine, ref_idx);
		}
		else
		{
			scattered = ray(rec.p, reflected, r_in.time());
			reflect_prob = 1.f;
		}

		if (drand48() < reflect_prob)
		{
			scattered = ray(rec.p, reflected, r_in.time());
		}
		else
		{
			scattered = ray(rec.p, refracted, r_in.time());
		}
		return true;
	}

	float ref_idx;
};

class diffuse_light : public material
{
public:
	diffuse_light(texture* a) : emit(a) {}
	virtual bool scatter(const ray& r_in, const hit_record& rec, vec3& attenuation, ray& scattered, float& pdf) const { return false; }
	virtual vec3 emitted(float u, float v, const vec3& p) const { return emit->value(u, v, p); }

	texture* emit;
};

class isotropic : public material
{
public:
	isotropic(texture* a) : albedo(a) {}
	virtual bool scatter(const ray& r_in, const hit_record& rec, vec3& attenuation, ray& scattered, float& pdf) const
	{
		scattered = ray(rec.p, random_in_unit_sphere());
		attenuation = albedo->value(rec.u, rec.v, rec.p);
		return true;
	}

	texture* albedo;
};