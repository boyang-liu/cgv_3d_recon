#pragma once

#include <cgv/render/render_types.h>
#include <cgv/render/shader_program.h>
#include <3rd/glew/GL/glew.h>
#include "rgbd_pointcloud.h"
#include <cgv/render/render_types.h>


typedef float Crd;
typedef cgv::math::fvec<float, 3> vec3;
typedef cgv::math::fvec<float, 4> vec4;
typedef cgv::math::fvec<std::int32_t, 3> ivec3;
typedef cgv::type::uint8_type ClrComp;
typedef cgv::media::color<ClrComp, cgv::media::RGB, cgv::media::OPACITY> Rgba;
typedef cgv::media::color<float, cgv::media::RGB> rgb;
typedef cgv::media::axis_aligned_box<float, 3> box3;
typedef cgv::math::fmat<Crd, 3, 3> Mat;

//namespace voxel {


typedef cgv::math::fvec<uint32_t, 3> uvec3;

class Voxelization: public cgv::render::render_types//:
	/*public cgv::base::node,
	public cgv::render::drawable,
	public cgv::gui::provider,
	public cgv::gui::event_handler*/
{
private:
	
protected:
	cgv::render::shader_program voxelize_prog;
	cgv::render::texture depth_tex;
	cgv::render::texture v_id_tex;
	
	/*struct depthpixel 
	{
		float depthsquare;
		rgba8 color;
	};*/
public:
	Voxelization::Voxelization();
	
	/// resolution of the volume

	//GLuint m_cntBuffer;

	uvec3 vres;
	bool init_voxelization(cgv::render::context& ctx, const float step, vec3 min, vec3 max, std::vector<Mat> inver_r, std::vector<vec3> inver_t, std::vector< std::vector<std::vector<depthpixel>>> depthimageplane);
};


//}