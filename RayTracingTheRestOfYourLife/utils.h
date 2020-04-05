#pragma once

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "vec3.h"
#include <random>

std::random_device rd;  //Will be used to obtain a seed for the random number engine
std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
std::uniform_real_distribution<float> dis(0.f, 1.f);

float drand48()
{
	return dis(gen);
}

vec3 random_in_unit_disk()
{
	vec3 p(0, 0, 0);
	do
	{
		p = 2.f*vec3(drand48(), drand48(), 0) - vec3(1, 1, 0);
	} while (dot(p,p) >= 1.f);
	return p;
}

vec3 random_in_unit_sphere()
{
	vec3 p(0, 0, 0);
	do
	{
		p = 2.f*vec3(drand48(), drand48(), drand48()) - vec3(1, 1, 1);
	} while (p.squared_length() >= 1.f);
	return p;
}

vec3 random_on_unit_sphere()
{
	vec3 p(0, 0, 0);
	do
	{
		p = 2.f * vec3(drand48(), drand48(), drand48()) - vec3(1, 1, 1);
	} while (dot(p,p) >= 1.f);
	return unit_vector(p);
}


vec3 reflect(const vec3& v, const vec3& n)
{
	return v - 2 * dot(v, n)*n;
}

bool refract(const vec3& v, const vec3& n, float ni_over_nt, vec3& refracted)
{
	vec3 uv = unit_vector(v);
	float dt = dot(uv, n);
	float discriminant = 1.f - ni_over_nt * ni_over_nt * (1 - dt * dt);
	if (discriminant > 0)
	{
		refracted = ni_over_nt * (uv - n * dt) - n * sqrt(discriminant);
		return true;
	}
	else
	{
		return false;
	}
}

float schlick(float cosine, float ref_idx)
{
	float r0 = (1 - ref_idx) / (1 + ref_idx);
	r0 = r0 * r0;
	return r0 + (1 - r0)*pow(1 - cosine, 5);
}
