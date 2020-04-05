#include "pch.h"

#define _USE_MATH_DEFINES
#include <cmath>

#include <iostream>
#include <iomanip>
#include <iostream>
#include <atomic>
#include "parallel_for.h"

#include "utils.h"

#include "sphere.h"
#include "hitable.h"

#include "camera.h"
#include "material.h"
#include "texture.h"

vec3 color(const ray& r, hitable* world, int depth)
{
	hit_record rec;
	if (world->hit(r, 0.001f, std::numeric_limits<float>::max(), rec))
	{
		ray scattered;
		vec3 attenuation;
		vec3 emitted = rec.mat_ptr->emitted(rec.u, rec.v, rec.p);
		float pdf;
		if (depth < 50 && rec.mat_ptr->scatter(r, rec, attenuation, scattered, pdf))
		{
			return emitted + attenuation * rec.mat_ptr->scattering_pdf(r, rec, scattered) * color(scattered, world, depth + 1) / pdf;
		}
		else
		{
			return emitted;
		}
	}
	else
	{
		return vec3(0, 0, 0);
	}
}

inline float pdf(const vec3& p)
{
	return 1 / (4 * M_PI);
}

void cornell_box(hitable** scene, camera** cam, float aspect)
{
	int i = 0;
	hitable** list = new hitable * [9];
	material* red = new lambertian(new constant_texture(vec3(0.65, 0.05, 0.05)));
	material* white = new lambertian(new constant_texture(vec3(0.73, 0.73, 0.73)));
	material* green = new lambertian(new constant_texture(vec3(0.12, 0.45, 0.15)));
	material* light = new diffuse_light(new constant_texture(vec3(15, 15, 15)));
	list[i++] = new flip_normals(new yz_rect(0, 555, 0, 555, 555, green));
	list[i++] = new yz_rect(0, 555, 0, 555, 0, red);
	list[i++] = new xz_rect(213, 343, 226, 332, 554, light);
	list[i++] = new flip_normals(new xz_rect(0, 555, 0, 555, 555, white));
	list[i++] = new xz_rect(0, 555, 0, 555, 0, white);
	list[i++] = new flip_normals(new xy_rect(0, 555, 0, 555, 555, white));
	list[i++] = new translate(new rotate_y(new box(vec3(0, 0, 0), vec3(165, 165, 165), white), -18), vec3(130, 0, 65));
	list[i++] = new translate(new rotate_y(new box(vec3(0, 0, 0), vec3(165, 330, 165), white), 15), vec3(265, 0, 265));
	*scene = new hitable_list(list, i);
	vec3 lookfrom(278, 278, -800);
	vec3 lookat(278, 278, 0);
	float dist_to_focus = 10;
	float aperture = 0;
	float vfov = 40;
	*cam = new camera(lookfrom, lookat, vec3(0, 1, 0), vfov, aspect, aperture, dist_to_focus, 0, 1);
}

int main()
{

	int nx = 256;
	int ny = 256;
	int ns = 1000;
	std::cout << "P3" << std::endl << nx << " " << ny << std::endl << "255" << std::endl;

	hitable* scene;
	camera* cam;
	cornell_box(&scene, &cam, nx/ny);

	for (int j = ny - 1; j >= 0; j--)
	{
		for (int i = 0; i < nx; i++)
		{
			std::atomic<vec3> col(vec3(0, 0, 0));
			parallel_for(ns, [&](int start, int end) {
				for (int s = start; s < end; s++)
				{
					float u = float(i + drand48()) / float(nx);
					float v = float(j + drand48()) / float(ny);
					ray r = cam->get_ray(u, v);
					vec3 tmp_color = color(r, scene, 0);
					col = col + tmp_color;
				}
				}, true);

			col = col / float(ns);

			vec3 col_val = col.load();

			col_val = vec3(sqrt(col_val[0]), sqrt(col_val[1]), sqrt(col_val[2]));

			if (col_val[0] > 1)
				col_val[0] = 1;
			if (col_val[1] > 1)
				col_val[1] = 1;
			if (col_val[2] > 1)
				col_val[2] = 1;
			int ir = int(255.99f * col_val.r());
			int ig = int(255.99f * col_val.g());
			int ib = int(255.99f * col_val.b());
			std::cout << ir << " " << ig << " " << ib << std::endl;
		}
	}
}