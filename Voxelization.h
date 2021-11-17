#pragma once

#include <cgv/render/render_types.h>
#include <cgv/render/shader_program.h>
#include <3rd/glew/GL/glew.h>
#include "rgbd_pointcloud.h"
#include <cgv/render/render_types.h>



typedef cgv::math::fvec<float, 3> vec3;
typedef cgv::math::fvec<std::int32_t, 3> ivec3;
typedef cgv::type::uint8_type ClrComp;
typedef cgv::media::color<ClrComp, cgv::media::RGB, cgv::media::OPACITY> Rgba;
typedef cgv::media::color<float, cgv::media::RGB> rgb;
typedef cgv::media::axis_aligned_box<float, 3> box3;
namespace voxel {
class Voxelization:
	public cgv::base::node,
	public cgv::render::drawable,
	public cgv::gui::provider,
	public cgv::gui::event_handler
{
private:
	
protected:
	cgv::render::shader_program voxelize_prog;
public:
	Voxelization() = default;
	~Voxelization() = default;
	/// resolution of the volume
	uvec3 vres;
	bool init_voxelize(cgv::render::context& ctx,const float& step, vec3 min, vec3 max, rgbd_pointcloud pc);
};


}