#pragma once

#include <cgv/render/render_types.h>
#include <cgv/render/shader_program.h>
#include <3rd/glew/GL/glew.h>
#include "rgbd_pointcloud.h"
#include <cgv/render/render_types.h>
#include <cgv_gl/rounded_cone_renderer.h>
#include "PCBoundingbox.h"
#include "Buffer.h"

typedef cgv::math::fmat<float, 3, 3> Mat;

//c
//namespace voxel {
typedef cgv::type::uint8_type ClrComp;
typedef cgv::media::color<ClrComp, cgv::media::RGB, cgv::media::OPACITY> Rgba;
//typedef cgv::math::fvec<uint32_t, 3> uvec3;

class Voxelization: public cgv::render::render_types//:
	
{
private:
	



public:
	cgv::render::rounded_cone_render_style rcrs;


	Voxelization();
	~Voxelization();
	
	//bool init_voxelization_from_image(cgv::render::context& ctx, float myvoxel_size, vec3 min, vec3 max, std::vector<Mat> inver_r, std::vector<vec3> inver_t, std::vector< std::vector<std::vector<depthpixel>>> depthimageplane);
	
	bool init_voxelization(cgv::render::context& ctx);
	
	bool resize();
	
	
	void draw_voxels(cgv::render::context& ctx,bool showvolume);
	vec3 get_center_gravity() { return center_gravity; }
	void draw_center_mass(cgv::render::context& ctx,vec3 center_m);
	bool init(std::vector<rgbd_pointcloud> pc, ivec3 resolution, vec3 min, vec3 max,  bool showmesh);//float side,
	bool generate(cgv::render::context& ctx, std::vector<vec3> cam_pos, bool showmesh);
	void bindbuffer(bool showmesh);	
	void createBuffers(bool showmesh);

	int* flattenTriTable();
	void deleteBuffers(bool showmesh);
	bool drawmesh(cgv::render::context& ctx);
	void clear(cgv::render::context& ctx);
	void clear_center_pts() { center_pts.clear(); center_clrs.clear();
	}
	int get_numBoxes() { return numBoxes; }

	int filter_threshold_1 = 5;
	int kernel_range_1 = 3;
	int filter_threshold_2 = 13;
	int kernel_range_2 = 5;

protected:
	

	cgv::render::shader_program fill_prog;
	cgv::render::shader_program denoise_prog;
	cgv::render::shader_program remove_outlier_prog;
	cgv::render::shader_program marchingcubes_prog, triangle_prog, normal_prog;
	cgv::render::shader_program GridPoints_prog;
	float side_length;
	vec3 min_pos;
	vec3 max_pos;
	std::vector<box3> boxes;
	std::vector<Rgba> box_colors;	
	vec3 center_gravity;
	std::vector<vec3> center_pts;
	std::vector<rgb> center_clrs;
	GLuint numBoxes;
	ivec3 Voxel_size;
	std::vector<float> Object_Boundary;
	std::vector<float> render_content;
	float surfaceLevel = 0.5;
	ivec3 GridPoints_size;
	int maxNumVertices;
	int maxNumTriangles;
	GLuint numVertices=0;
	GLuint numTriangles=0;
private:
	Buffer object_boundary;
	Buffer filled_object;
	Buffer denoised_object;
	Buffer cubes;

	Buffer gridpoints;
	Buffer trianglenormals;
	Buffer triangles;
	Buffer tables;
	Buffer cubeedges;
	Buffer vertices;
	Buffer normals;
	Buffer normalid;


	//Buffer boxarray;
	//Buffer center_mass;
};


//}