#pragma once

#include <cgv/render/render_types.h>
#include <cgv/render/shader_program.h>
#include <3rd/glew/GL/glew.h>
#include "rgbd_pointcloud.h"
#include "voxel_renderer.h"
//#include "Drawable/Drawable.h"


typedef cgv::math::fvec<float, 3> vec3;
typedef cgv::math::fvec<std::int32_t, 3> ivec3;
typedef cgv::type::uint8_type ClrComp;
typedef cgv::media::color<ClrComp, cgv::media::RGB, cgv::media::OPACITY> Rgba;
namespace voxel {
class Voxelization
{
private:
	GLuint m_cntBuffer;
protected:
	cgv::render::shader_program voxelize_prog;
public:
	Voxelization() = default;
	~Voxelization() = default;

	bool voxelize(cgv::render::context& ctx,const float& step, std::vector<vec3>& ret, vec3 min, vec3 max, rgbd_pointcloud pc);
};


}