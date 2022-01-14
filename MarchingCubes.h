#pragma once

#include <cgv/render/render_types.h>
#include <cgv/render/shader_program.h>
#include <3rd/glew/GL/glew.h>
#include "rgbd_pointcloud.h"
#include <cgv/render/render_types.h>
#include "PCBoundingbox.h"
#include <cgv_gl/sphere_renderer.h>




class MarchingCubes : public cgv::render::render_types {




public:
	struct Triangle
	{
		vec3 a, b, c;
	};
	MarchingCubes() :// int width, int height, int depth, float _cubeSize
		Vertices_tex("flt32[R]")
		//cubeSize(_cubeSize),
		//densityGrid(width + 1, height + 1, depth + 1),
		//cubeGrid(width, height, depth)
	{
		Vertices_tex.set_min_filter(cgv::render::TF_LINEAR_MIPMAP_LINEAR);
		Vertices_tex.set_mag_filter(cgv::render::TF_LINEAR);
		Vertices_tex.set_wrap_s(cgv::render::TW_CLAMP_TO_BORDER);
		Vertices_tex.set_wrap_t(cgv::render::TW_CLAMP_TO_BORDER);
		Vertices_tex.set_wrap_r(cgv::render::TW_CLAMP_TO_BORDER);
		Vertices_tex.set_border_color(0.0f, 0.0f, 0.0f, 0.0f);
	};
	bool get_signed_distance_func();
	bool set_signed_weight( std::vector<int> Voxelid, std::vector<float> Voxel, uvec3 Vox_resolution);
	bool init_MC(cgv::render::context& ctx);
	void resize();
	bool resize(int width, int height, int depth, float _cubeSize);
	bool generate(cgv::render::context& ctx);
	std::vector<float> get_vertices() { return Vertices; }
	int* flattenTriTable();

	bool draw(cgv::render::context& ctx);


	bool ge(cgv::render::context& ctx);//, std::vector<int> Voxelid


	
	struct {
		float x, y, z, max;
	} size;

	int numTriangles = 0;
	//std::vector<Triangle> triangles;

	vec3 min_pos = vec3(0.83623, -0.728815, 2.24123);
	vec3 max_pos = vec3(2.83623, 1.271185, 4.24123);


protected:
	GLuint tables_buffer;
	GLuint cubes_buffer;
	GLuint vertices_buffer;
	GLuint triangles_buffer;

	float surfaceLevel = 0;
	std::vector<float> Vertices;
	std::vector<vec3> triangles;
	ivec3 Vertices_size;
	ivec3 Voxel_size;
	GLuint triCount;
	cgv::render::texture Vertices_tex;

	cgv::render::shader_program marchingcubes_prog;

	int maxNumVertices;
	int maxNumTriangles;

private:

	float cubeSize = 0.1f;
};