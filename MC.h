#pragma once

#include <cgv/render/render_types.h>
#include <cgv/render/shader_program.h>
#include <3rd/glew/GL/glew.h>
#include "rgbd_pointcloud.h"
#include <cgv/render/render_types.h>
#include "PCBoundingbox.h"


struct Volume
{
	int x = 0, y = 0, z = 0, count = 0;
	Volume();
	Volume(int _x, int _y, int _z);
};


class MarchingCubes : public cgv::render::render_types {

private:




public:

	MarchingCubes(int width, int height, int depth, float _cubeSize) : 
		Vertices_tex("flt32[R]"),
		cubeSize(_cubeSize),
		//densityGrid(width + 1, height + 1, depth + 1),
		cubeGrid(width, height, depth)
	{
		Vertices_tex.set_min_filter(cgv::render::TF_LINEAR_MIPMAP_LINEAR);
		Vertices_tex.set_mag_filter(cgv::render::TF_LINEAR);
		Vertices_tex.set_wrap_s(cgv::render::TW_CLAMP_TO_BORDER);
		Vertices_tex.set_wrap_t(cgv::render::TW_CLAMP_TO_BORDER);
		Vertices_tex.set_wrap_r(cgv::render::TW_CLAMP_TO_BORDER);
		Vertices_tex.set_border_color(0.0f, 0.0f, 0.0f, 0.0f);
	};
	bool get_signed_distance_func();
	bool set_signed_weight( std::vector<int> Voxelid, std::vector<float> Voxel, uvec3 Vox_size);
	bool init_MC(cgv::render::context& ctx);
	bool generate(cgv::render::context& ctx);
	int* flattenTriTable();

	bool draw();

protected:
	GLuint tables_buffer;
	GLuint cubes_buffer;
	/*GLuint Voxelid_buffer;*/

	std::vector<float> Vertices;
	ivec3 Vertices_size;
	cgv::render::texture Vertices_tex;
	cgv::render::shader_program marchingcubes_prog;

	

private:
	Volume cubeGrid;
	float cubeSize = 0.1f;
};