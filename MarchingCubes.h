#pragma once

#include <cgv/render/render_types.h>
#include <cgv/render/shader_program.h>
#include <3rd/glew/GL/glew.h>
#include "rgbd_pointcloud.h"
#include <cgv/render/render_types.h>
#include "PCBoundingbox.h"
#include <cgv_gl/sphere_renderer.h>
#include "Buffer.h"

typedef cgv::math::fmat<float, 3, 3> Mat;
typedef cgv::type::uint8_type ClrComp;
typedef cgv::media::color<ClrComp, cgv::media::RGB, cgv::media::OPACITY> Rgba;

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
	/*
	bool init_voxelization(cgv::render::context& ctx);
	bool init_boundary_from_PC(std::vector<rgbd_pointcloud> pc, vec3 min, vec3 max, float voxel_length);
	bool denoising(cgv::render::context& ctx, int filter_threshold, int kernel_range);//the kernel_range should be odd 
	bool traverse_voxels(cgv::render::context& ctx, std::vector<vec3> cam_pos);
	void draw_voxels(cgv::render::context& ctx);
	vec3 get_center_gravity() { return center_gravity; }

	bool init(std::vector<rgbd_pointcloud> pc, vec3 min, vec3 max, float side);
	bool generate(cgv::render::context& ctx, std::vector<vec3> cam_pos);
	void bindbuffer();
	void createBuffers();
	void deleteBuffers();
	int get_numBoxes() { return numBoxes; }
*/


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
	
	
	cgv::render::shader_program fill_prog;
	cgv::render::shader_program denoise_prog;
	cgv::render::shader_program remove_outlier_prog;

	/*float side_length;
	vec3 min_pos;
	vec3 max_pos;
	ivec3 Voxel_size;
	*/

	std::vector<box3> boxes;
	std::vector<Rgba> box_colors;	
	vec3 center_gravity;
	GLuint numBoxes;
	
	std::vector<float> Object_Boundary;
	/*std::vector<float> render_content;
	*/

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

	float cubeSize = 0.2f;
	Buffer gridpoints;
	Buffer trianglenormals;
	Buffer cubes;
	Buffer triangles;
	Buffer tables;
	Buffer cubeedges;
	Buffer vertices;
	Buffer normals;
	Buffer normalid;

	/*Buffer object_boundary;
	Buffer filled_object;
	Buffer denoised_object;
	Buffer cubes;*/
};