#pragma once

#include <cgv/render/render_types.h>
#include <cgv/render/shader_program.h>
#include <3rd/glew/GL/glew.h>
#include "rgbd_pointcloud.h"
#include <cgv/render/render_types.h>
#include "PCBoundingbox.h"
#include <cgv_gl/sphere_renderer.h>
#include "Buffer.h"



class MarchingCubes : public cgv::render::render_types {




public:
	struct Triangle
	{
		vec3 a, b, c;
	};
	MarchingCubes() {
		min_pos = vec3(0, 0, 0);
		max_pos = vec3(2, 2, 2);
		hasBuffers = false;
	};
	//bool get_signed_distance_func();
	bool set_signed_weight( std::vector<int> Voxelid, std::vector<float> Voxel, uvec3 Vox_resolution);
	bool initialize(cgv::render::context& ctx);
	bool initMC(std::vector<float> result_voxelization, ivec3 resolution, vec3 min, vec3 max);
	bool initMC(std::vector<float> result_voxelization);
	void resize();
	bool resize(ivec3 resolution, vec3 min, vec3 max);

	bool generate(cgv::render::context& ctx);

	//std::vector<float> get_GridPoints() { return GridPoints; }
	
	int* flattenTriTable();
	void bindbuffer();
	bool draw(cgv::render::context& ctx);
	void deleteBuffers();
	void createBuffers();
	//bool ge(cgv::render::context& ctx);//, std::vector<int> Voxelid

	int get_numVertices() { return numVertices; }
	
	struct {
		float x, y, z, max;
	} size;

	
	//std::vector<Triangle> triangles;

	vec3 min_pos ;
	vec3 max_pos ;


protected:
	

	float surfaceLevel = 0.5;
	//std::vector<float> GridPoints;
	std::vector<float> Cubes;
	std::vector<vec3> Triangles;
	ivec3 GridPoints_size;
	ivec3 Voxel_size;
	GLuint triCount;
	//cgv::render::texture Vertices_tex;

	cgv::render::shader_program marchingcubes_prog,triangle_prog,normal_prog;
	//cgv::render::shader_program Normals_prog;
	cgv::render::shader_program GridPoints_prog;

	int maxNumVertices;
	int maxNumTriangles;

	GLuint numVertices;
	GLuint numTriangles;
	bool hasBuffers;
private:

	float cubeSize = 0.1f;
	Buffer gridpoints;
	Buffer trianglenormals;
	Buffer cubes;
	Buffer triangles;
	Buffer tables;
	Buffer cubeedges;
	Buffer vertices;
	Buffer normals;
	Buffer normalid;
};