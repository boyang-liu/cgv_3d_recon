#pragma once

#include <cgv/render/render_types.h>
#include <cgv/render/shader_program.h>
#include <3rd/glew/GL/glew.h>
#include "rgbd_pointcloud.h"
#include <cgv/render/render_types.h>
#include "PCBoundingbox.h"

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
	
	Voxelization() : pixel_depth_tex("flt32[R]"), v_id_tex("flt32[R,G,B,A]"), V_tex("flt32[R]"), V_new_tex("flt32[R,G,B,A]")
	{

		voxel_size = 0.1;
		pixel_depth_tex.set_min_filter(cgv::render::TF_LINEAR_MIPMAP_LINEAR);
		pixel_depth_tex.set_mag_filter(cgv::render::TF_LINEAR);
		pixel_depth_tex.set_wrap_s(cgv::render::TW_CLAMP_TO_BORDER);
		pixel_depth_tex.set_wrap_t(cgv::render::TW_CLAMP_TO_BORDER);
		pixel_depth_tex.set_wrap_r(cgv::render::TW_CLAMP_TO_BORDER);
		pixel_depth_tex.set_border_color(0.0f, 0.0f, 0.0f, 0.0f);

		v_id_tex.set_min_filter(cgv::render::TF_LINEAR);
		v_id_tex.set_mag_filter(cgv::render::TF_LINEAR);
		v_id_tex.set_wrap_s(cgv::render::TW_CLAMP_TO_EDGE);
		v_id_tex.set_wrap_t(cgv::render::TW_CLAMP_TO_EDGE);
		v_id_tex.set_wrap_r(cgv::render::TW_CLAMP_TO_EDGE);


		V_tex.set_min_filter(cgv::render::TF_LINEAR_MIPMAP_LINEAR);
		V_tex.set_mag_filter(cgv::render::TF_LINEAR);
		V_tex.set_wrap_s(cgv::render::TW_CLAMP_TO_BORDER);
		V_tex.set_wrap_t(cgv::render::TW_CLAMP_TO_BORDER);
		V_tex.set_wrap_r(cgv::render::TW_CLAMP_TO_BORDER);
		V_tex.set_border_color(0.0f, 0.0f, 0.0f, 0.0f);

		V_new_tex.set_min_filter(cgv::render::TF_LINEAR);
		V_new_tex.set_mag_filter(cgv::render::TF_LINEAR);
		V_new_tex.set_wrap_s(cgv::render::TW_CLAMP_TO_EDGE);
		V_new_tex.set_wrap_t(cgv::render::TW_CLAMP_TO_EDGE);
		V_new_tex.set_wrap_r(cgv::render::TW_CLAMP_TO_EDGE);

	};
	
	//bool init_voxelization(cgv::render::context& ctx);

	//uvec3 vres;
	bool init_voxelization_from_image(cgv::render::context& ctx, float myvoxel_size, vec3 min, vec3 max, std::vector<Mat> inver_r, std::vector<vec3> inver_t, std::vector< std::vector<std::vector<depthpixel>>> depthimageplane);
	
	
	bool init_surface_from_PC(std::vector<rgbd_pointcloud> pc, vec3 min, vec3 max, float voxel_length);
	bool traverse_voxels(cgv::render::context& ctx, std::vector<vec3>cam_pos);
	void draw_voxels(cgv::render::context& ctx);
	
	cgv::render::shader_program voxelize_prog;
protected:
	
	cgv::render::texture pixel_depth_tex;
	cgv::render::texture v_id_tex;
	
	cgv::render::texture V_tex;
	cgv::render::texture V_new_tex;
	float voxel_size;
	std::vector<float> V;
	uvec3 V_size;
	
	vec3 min_pos;
	vec3 max_pos;

	PCBoundingbox voxelboundingbox;

	std::vector<box3> boxes;
	std::vector<rgb> box_colors;	

};


//}