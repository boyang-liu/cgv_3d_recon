#pragma once

#include <cgv/render/render_types.h>
#include <cgv/render/shader_program.h>
#include <3rd/glew/GL/glew.h>
#include "rgbd_pointcloud.h"
#include <cgv/render/render_types.h>
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
	
	Voxelization()  
	{

	};
	
	//bool init_voxelization_from_image(cgv::render::context& ctx, float myvoxel_size, vec3 min, vec3 max, std::vector<Mat> inver_r, std::vector<vec3> inver_t, std::vector< std::vector<std::vector<depthpixel>>> depthimageplane);
	
	bool init_voxelization(cgv::render::context& ctx);
	bool init_boundary_from_PC(std::vector<rgbd_pointcloud> pc, vec3 min, vec3 max, float voxel_length);
	bool denoising(cgv::render::context& ctx, int filter_threshold, int kernel_range);//the kernel_range should be odd 
	bool traverse_voxels(cgv::render::context& ctx, std::vector<vec3> cam_pos);
	void draw_voxels(cgv::render::context& ctx,bool showvolume);
	vec3 get_center_gravity() { return center_gravity; }
	void draw_center_mass(cgv::render::context& ctx,vec3 center_m);
	bool init(std::vector<rgbd_pointcloud> pc, vec3 min, vec3 max, float resolution);
	bool generate(cgv::render::context& ctx, std::vector<vec3> cam_pos);
	void bindbuffer();	
	void createBuffers();
	void deleteBuffers();

	int get_numBoxes() { return numBoxes; }
protected:
	
	cgv::render::shader_program fill_prog;
	cgv::render::shader_program denoise_prog;
	cgv::render::shader_program remove_outlier_prog;
	float side_length;
	vec3 min_pos;
	vec3 max_pos;
	std::vector<box3> boxes;
	std::vector<Rgba> box_colors;	
	vec3 center_gravity;
	GLuint numBoxes;
	ivec3 Voxel_size;
	std::vector<float> Object_Boundary;
	std::vector<float> render_content;

private:
	Buffer object_boundary;
	Buffer filled_object;
	Buffer denoised_object;
	Buffer cubes;
	//Buffer boxarray;
	//Buffer center_mass;
};


//}