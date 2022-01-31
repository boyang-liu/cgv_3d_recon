#pragma once

#include <cgv/render/render_types.h>
#include <cgv/render/shader_program.h>
#include <3rd/glew/GL/glew.h>
#include "rgbd_pointcloud.h"
#include <cgv/render/render_types.h>
#include "PCBoundingbox.h"
#include "Buffer.h"
//typedef float Crd;
//typedef cgv::math::fvec<float, 3> vec3;
//typedef cgv::math::fvec<float, 4> vec4;
//typedef cgv::math::fvec<std::int32_t, 3> ivec3;
//typedef cgv::type::uint8_type ClrComp;
//typedef cgv::media::color<ClrComp, cgv::media::RGB, cgv::media::OPACITY> Rgba;
//typedef cgv::media::color<float, cgv::media::RGB> rgb;
//typedef cgv::media::axis_aligned_box<float, 3> box3;
typedef cgv::math::fmat<float, 3, 3> Mat;

//c
//namespace voxel {


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
	void draw_voxels(cgv::render::context& ctx);
	void get_center_gravity();

	bool init(std::vector<rgbd_pointcloud> pc, vec3 min, vec3 max, float side);
	bool generate(cgv::render::context& ctx, std::vector<vec3> cam_pos);
	void bindbuffer();	
	void createBuffers();
	void deleteBuffers();

	std::vector<int> get_Voxel_id() {return V_1;}
	std::vector<float> get_V() { return V; }
	int get_numBoxes() { return numBoxes; }
protected:
	

	cgv::render::shader_program fill_prog;
	cgv::render::shader_program denoise_prog;
	cgv::render::shader_program remove_outlier_prog;


	
	std::vector<float> V;
	std::vector<int> V_1;
	std::vector<vec3> V_color;
	//std::vector<int> num_p_in_voxel;
	uvec3 V_size;



	float side_length;
	vec3 min_pos;
	vec3 max_pos;

	//PCBoundingbox voxelboundingbox;

	std::vector<box3> boxes;
	std::vector<rgb> box_colors;	

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
	Buffer boxarray;
	Buffer center_mass;
};


//}