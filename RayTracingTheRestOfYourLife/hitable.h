#pragma once

#include "ray.h"
#include "aabb.h"

class material;
class isotropic;
class texture;

struct hit_record 
{
	float t;
	vec3 p;
	vec3 normal;
	material* mat_ptr;
	float u, v;
};

class hitable
{
public:
	virtual bool hit(const ray& r, float tmin, float tmax, hit_record& rec) const = 0;
	virtual bool bounding_box(float t0, float t1, aabb& box) const = 0;
};

class hitable_list : public hitable
{
public:
	hitable_list() {}
	hitable_list(hitable** l, int n) { list = l; list_size = n; }
	virtual bool hit(const ray& r, float tmin, float tmax, hit_record& rec) const;
	virtual bool bounding_box(float t0, float t1, aabb& box) const;
	hitable** list;
	int list_size;
};

bool hitable_list::hit(const ray& r, float tmin, float tmax, hit_record& rec) const
{
	hit_record temp_rec;
	bool hit_anything = false;
	double closest_so_far = tmax;
	for (int i = 0; i < list_size; i++)
	{
		if (list[i]->hit(r, tmin, closest_so_far, temp_rec))
		{
			hit_anything = true;
			closest_so_far = temp_rec.t;
			rec = temp_rec;
		}
	}
	return hit_anything;
}

bool hitable_list::bounding_box(float t0, float t1, aabb& box) const
{
	if (list_size < 1) return false;
	aabb temp_box;
	bool first_true = list[0]->bounding_box(t0, t1, temp_box);
	if (!first_true)
		return false;
	else
		box = temp_box;
	for (int i = 1; i < list_size; i++)
	{
		if (list[0]->bounding_box(t0, t1, temp_box))
		{
			box = surrounding_box(box, temp_box);
		}
		else
			return false;
	}
	return true;
}

class bvh_node : public hitable
{
public:
	bvh_node() {}
	bvh_node(hitable** l, int n, float time0, float time1);
	virtual bool hit(const ray& r, float tmin, float tmax, hit_record& rec) const;
	virtual bool bounding_box(float t0, float t1, aabb& box) const;
	hitable* left;
	hitable* right;
	aabb box;
};

int box_x_compare(const void* a, const void* b)
{
	aabb box_left, box_right;
	hitable* ah = *(hitable * *)a;
	hitable* bh = *(hitable * *)b;
	if (!ah->bounding_box(0, 0, box_left) || !bh->bounding_box(0, 0, box_right))
		std::cerr << "No bounding box in bvh_node constructor" << std::endl;
	if (box_left.min().x() - box_right.min().x() < 0.f)
		return -1;
	else
		return 1;
}

int box_y_compare(const void* a, const void* b)
{
	aabb box_left, box_right;
	hitable* ah = *(hitable * *)a;
	hitable* bh = *(hitable * *)b;
	if (!ah->bounding_box(0, 0, box_left) || !bh->bounding_box(0, 0, box_right))
		std::cerr << "No bounding box in bvh_node constructor" << std::endl;
	if (box_left.min().y() - box_right.min().y() < 0.f)
		return -1;
	else
		return 1;
}

int box_z_compare(const void* a, const void* b)
{
	aabb box_left, box_right;
	hitable* ah = *(hitable * *)a;
	hitable* bh = *(hitable * *)b;
	if (!ah->bounding_box(0, 0, box_left) || !bh->bounding_box(0, 0, box_right))
		std::cerr << "No bounding box in bvh_node constructor" << std::endl;
	if (box_left.min().z() - box_right.min().z() < 0.f)
		return -1;
	else
		return 1;
}

bvh_node::bvh_node(hitable** l, int n, float time0, float time1)
{
	int axis = int(3 * drand48());
	if (axis == 0)
		qsort(l, n, sizeof(hitable*), box_x_compare);
	else if (axis == 1)
		qsort(l, n, sizeof(hitable*), box_y_compare);
	else 
		qsort(l, n, sizeof(hitable*), box_z_compare);
	if (n == 1)
		left = right = l[0];
	else if (n == 2)
	{
		left = l[0];
		right = l[1];
	}
	else
	{
		left = new bvh_node(l, n / 2, time0, time1);
		right = new bvh_node(l + n / 2, n - n / 2, time0, time1);
	}
	aabb box_left, box_right;
	if (!left->bounding_box(time0, time1, box_left) || !right->bounding_box(time0, time1, box_right))
		std::cerr << "No bounding box in bvh_node constructor" << std::endl;
	box = surrounding_box(box_left, box_right);
}

bool bvh_node::hit(const ray& r, float tmin, float tmax, hit_record& rec) const
{
	if (box.hit(r, tmin, tmax))
	{
		hit_record left_rec, right_rec;
		bool hit_left = left->hit(r, tmin, tmax, left_rec);
		bool hit_right = right->hit(r, tmin, tmax, right_rec);
		if (hit_left && hit_right)
		{
			if (left_rec.t < right_rec.t)
				rec = left_rec;
			else
				rec = right_rec;
			return true;
		}
		else if (hit_left)
		{
			rec = left_rec;
			return true;
		}
		else if (hit_right)
		{
			rec = right_rec;
			return true;
		}
	}

	return false;
}

bool bvh_node::bounding_box(float t0, float t1, aabb& b) const
{
	b = box;
	return true;
}

class xy_rect : public hitable
{
public:
	xy_rect() {}
	xy_rect(float _x0, float _x1, float _y0, float _y1, float _k, material* mat)
		: x0(_x0), x1(_x1), y0(_y0), y1(_y1), k(_k), mp(mat) {}
	virtual bool hit(const ray& r, float t0, float t1, hit_record& rec) const;
	virtual bool bounding_box(float t0, float t1, aabb& box) const
	{
		box = aabb(vec3(x0, y0, k - 0.0001f), vec3(x1, y1, k + 0.0001f));
		return true;
	}


	material* mp;
	float x0, x1, y0, y1, k;
};

class xz_rect : public hitable
{
public:
	xz_rect() {}
	xz_rect(float _x0, float _x1, float _z0, float _z1, float _k, material* mat)
		: x0(_x0), x1(_x1), z0(_z0), z1(_z1), k(_k), mp(mat) {}
	virtual bool hit(const ray& r, float t0, float t1, hit_record& rec) const;
	virtual bool bounding_box(float t0, float t1, aabb& box) const
	{
		box = aabb(vec3(x0, k - 0.0001f, z0), vec3(x1, k + 0.0001f, z1));
		return true;
	}


	material * mp;
	float x0, x1, z0, z1, k;
};

class yz_rect : public hitable
{
public:
	yz_rect() {}
	yz_rect(float _y0, float _y1, float _z0, float _z1, float _k, material* mat)
		: y0(_y0), y1(_y1), z0(_z0), z1(_z1), k(_k), mp(mat) {}
	virtual bool hit(const ray& r, float t0, float t1, hit_record& rec) const;
	virtual bool bounding_box(float t0, float t1, aabb& box) const
	{
		box = aabb(vec3(k - 0.0001f, y0, z0), vec3(k + 0.0001f, y1, z1));
		return true;
	}


	material * mp;
	float y0, y1, z0, z1, k;
};


bool xy_rect::hit(const ray& r, float t0, float t1, hit_record& rec) const
{
	float t = (k - r.origin().z()) / r.direction().z();
	if (t < t0 || t > t1)
		return false;
	float x = r.origin().x() + t * r.direction().x();
	float y = r.origin().y() + t * r.direction().y();
	if (x < x0 || x > x1 || y < y0 || y > y1)
		return false;
	rec.u = (x - x0) / (x1 - x0);
	rec.v = (y - y0) / (y1 - y0);
	rec.t = t;
	rec.mat_ptr = mp;
	rec.p = r.point_at_parameter(t);
	rec.normal = vec3(0, 0, 1);
	return true;
}

bool xz_rect::hit(const ray& r, float t0, float t1, hit_record& rec) const
{
	float t = (k - r.origin().y()) / r.direction().y();
	if (t < t0 || t > t1)
		return false;
	float x = r.origin().x() + t * r.direction().x();
	float z = r.origin().z() + t * r.direction().z();
	if (x < x0 || x > x1 || z < z0 || z > z1)
		return false;
	rec.u = (x - x0) / (x1 - x0);
	rec.v = (z - z0) / (z1 - z0);
	rec.t = t;
	rec.mat_ptr = mp;
	rec.p = r.point_at_parameter(t);
	rec.normal = vec3(0, 1, 0);
	return true;
}

bool yz_rect::hit(const ray& r, float t0, float t1, hit_record& rec) const
{
	float t = (k - r.origin().x()) / r.direction().x();
	if (t < t0 || t > t1)
		return false;
	float y = r.origin().y() + t * r.direction().y();
	float z = r.origin().z() + t * r.direction().z();
	if (y < y0 || y > y1 || z < z0 || z > z1)
		return false;
	rec.u = (y - y0) / (y1 - y0);
	rec.v = (z - z0) / (z1 - z0);
	rec.t = t;
	rec.mat_ptr = mp;
	rec.p = r.point_at_parameter(t);
	rec.normal = vec3(1, 0, 0);
	return true;
}

class flip_normals : public hitable
{
public:
	flip_normals(hitable* p) : ptr(p) {}
	virtual bool hit(const ray& r, float tmin, float tmax, hit_record& rec) const
	{
		if (ptr->hit(r, tmin, tmax, rec))
		{
			rec.normal = -rec.normal;
			return true;
		}
		else
			return false;
	}
	virtual bool bounding_box(float t0, float t1, aabb& box) const
	{
		return ptr->bounding_box(t0, t1, box);
	}
	hitable* ptr;
};

class box : public hitable
{
public:
	box() {}
	box(const vec3& p0, const vec3& p1, material* ptr);
	virtual bool hit(const ray& r, float tmin, float tmax, hit_record& rec) const;
	virtual bool bounding_box(float t0, float t1, aabb& box) const
	{
		box = aabb(pmin, pmax);
		return true;
	}
	vec3 pmin, pmax;
	hitable* list_ptr;
};

box::box(const vec3& p0, const vec3& p1, material* ptr)
{
	pmin = p0;
	pmax = p1;
	hitable** list = new hitable * [6];
	list[0] = new xy_rect(p0.x(), p1.x(), p0.y(), p1.y(), p1.z(), ptr);
	list[1] = new flip_normals(new xy_rect(p0.x(), p1.x(), p0.y(), p1.y(), p0.z(), ptr));
	list[2] = new xz_rect(p0.x(), p1.x(), p0.z(), p1.z(), p1.y(), ptr);
	list[3] = new flip_normals(new xz_rect(p0.x(), p1.x(), p0.z(), p1.z(), p0.y(), ptr));
	list[4] = new yz_rect(p0.y(), p1.y(), p0.z(), p1.z(), p1.x(), ptr);
	list[5] = new flip_normals(new yz_rect(p0.y(), p1.y(), p0.z(), p1.z(), p0.x(), ptr));
	list_ptr = new hitable_list(list, 6);
}

bool box::hit(const ray& r, float tmin, float tmax, hit_record& rec) const
{
	return list_ptr->hit(r, tmin, tmax, rec);
}

class translate : public hitable
{
public:
	translate(hitable* p, const vec3& displacement) : ptr(p), offset(displacement) {}
	virtual bool hit(const ray& r, float tmin, float tmax, hit_record& rec) const;
	virtual bool bounding_box(float t0, float t1, aabb& box) const;

	hitable* ptr;
	vec3 offset;
};

bool translate::hit(const ray& r, float tmin, float tmax, hit_record& rec) const
{
	ray moved_r(r.origin() - offset, r.direction(), r.time());
	if (ptr->hit(moved_r, tmin, tmax, rec))
	{
		rec.p += offset;
		return true;
	}
	else
		return false;
}

bool translate::bounding_box(float t0, float t1, aabb& box) const
{
	if (ptr->bounding_box(t0, t1, box))
	{
		box = aabb(box.min() + offset, box.max() + offset);
		return true;
	}
	else
	{
		return false;
	}
}

class rotate_y : public hitable
{
public:
	rotate_y(hitable* p, float angle) : ptr(p)
	{
		float radians = (M_PI / 180) * angle;
		sin_theta = sin(radians);
		cos_theta = cos(radians);
		hasbox = ptr->bounding_box(0, 1, bbox);
		vec3 min(FLT_MAX, FLT_MAX, FLT_MAX);
		vec3 max(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		for (int i = 0; i < 2; ++i)
		{
			for (int j = 0; j < 2; ++j)
			{
				for (int k = 0; k < 2; ++k)
				{
					float x = i * bbox.max().x() + (1 - i) * bbox.min().x();
					float y = j * bbox.max().y() + (1 - j) * bbox.min().y();
					float z = k * bbox.max().z() + (1 - k) * bbox.min().z();
					float newx = cos_theta * x + sin_theta * z;
					float newz = -sin_theta * x + cos_theta * z;
					vec3 tester(newx, y, newz);
					for (int c = 0; c < 3; ++c)
					{
						if (tester[c] < min[c])
							min[c] = tester[c];
						if (tester[c] > max[c])
							max[c] = tester[c];
					}
				}
			}
		}
		bbox = aabb(min, max);
	}
	virtual bool hit(const ray& r, float tmin, float tmax, hit_record& rec) const;
	virtual bool bounding_box(float t0, float t1, aabb& box) const
	{
		box = bbox;
		return hasbox;
	}

	hitable* ptr;
	float sin_theta;
	float cos_theta;
	bool hasbox;
	aabb bbox;
};

bool rotate_y::hit(const ray& r, float tmin, float tmax, hit_record& rec) const
{
	vec3 origin = r.origin();
	vec3 direction = r.direction();
	origin[0] = cos_theta * r.origin()[0] - sin_theta * r.origin()[2];
	origin[2] = sin_theta * r.origin()[0] + cos_theta * r.origin()[2];
	direction[0] = cos_theta * r.direction()[0] - sin_theta * r.direction()[2];
	direction[2] = sin_theta * r.direction()[0] + cos_theta * r.direction()[2];
	ray rotated_r(origin, direction, r.time());
	if (ptr->hit(rotated_r, tmin, tmax, rec))
	{
		vec3 p = rec.p;
		vec3 normal = rec.normal;
		p[0] = cos_theta * rec.p[0] + sin_theta * rec.p[2];
		p[2] = -sin_theta * rec.p[0] + cos_theta * rec.p[2];
		normal[0] = cos_theta * rec.normal[0] + sin_theta * rec.normal[2];
		normal[2] = -sin_theta * rec.normal[0] + cos_theta * rec.normal[2];
		rec.p = p;
		rec.normal = normal;
		return true;
	}
	else
		return false;
}

class constant_medium : public hitable
{
public:
	constant_medium(hitable* b, float d, isotropic* a) : boundary(b), density(d) { phase_function = (material*)a; }
	virtual bool hit(const ray& r, float tmin, float tmax, hit_record& rec) const;
	virtual bool bounding_box(float t0, float t1, aabb& box) const
	{
		return boundary->bounding_box(t0, t1, box);
	}

	hitable* boundary;
	float density;
	material* phase_function;
};

bool constant_medium::hit(const ray& r, float tmin, float tmax, hit_record& rec) const
{
	bool db = (drand48() < 0.00001f);
	db = false;
	hit_record rec1, rec2;
	if (boundary->hit(r, -FLT_MAX, FLT_MAX, rec1))
	{
		if (boundary->hit(r, rec1.t + 0.0001f, FLT_MAX, rec2))
		{
			if (db) std::cerr << std::endl << "t0 t1 " << rec1.t << " " << rec2.t << std::endl;
			if (rec1.t < tmin)
				rec1.t = tmin;
			if (rec2.t > tmax)
				rec2.t = tmax;
			if (rec1.t >= rec2.t)
				return false;
			if (rec1.t < 0)
				rec1.t = 0;
			float distance_inside_boundary = (rec2.t - rec1.t) * r.direction().length();
			float hit_distance = -(1 / density) * log(drand48());
			if (hit_distance < distance_inside_boundary)
			{
				if (db) std::cerr << "hit_distance = " << hit_distance << std::endl;
				rec.t = rec1.t + hit_distance / r.direction().length();
				if (db) std::cerr << "rec.t = " << rec.t << std::endl;
				rec.p = r.point_at_parameter(rec.t);
				if (db) std::cerr << "rec.p = " << rec.p << std::endl;
				rec.normal = vec3(1, 0, 0);
				rec.mat_ptr = phase_function;
				return true;
			}
		}
	}
	return false;
}