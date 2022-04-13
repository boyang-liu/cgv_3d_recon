#pragma once
//#include <cgv/media/axis_aligned_box.h>
#include <cgv/base/node.h>
#include <cgv/signal/rebind.h>
#include <cgv/base/register.h>
#include <cgv/gui/event_handler.h>
#include <cgv/gui/trigger.h>
#include <cgv/math/ftransform.h>
#include <cgv/utils/scan.h>
#include <cgv/utils/options.h>
#include <cgv/gui/provider.h>
#include <cgv/render/drawable.h>
#include <cgv/render/shader_program.h>
#include <cgv/render/frame_buffer.h>
#include <cgv/render/attribute_array_binding.h>
#include <cgv_gl/box_renderer.h>
#include <cgv_gl/point_renderer.h>
#include <cgv_gl/sphere_renderer.h>
#include <cgv/media/mesh/simple_mesh.h>
#include <cgv_gl/gl/mesh_render_info.h>
class PCBoundingbox
{

	typedef cgv::math::fvec<float, 3> vec3;
	
public:
	PCBoundingbox() {
		pos1 = (0, 0, 0);
		pos2 = (0, 0, 0);
	}
	vec3 pos1;
	vec3 pos2;
	vec3 boxcenter;
	float height;
	float length;
	float width;
	float step;
	vec3 getpos1() { return boxcenter - vec3(boxcenter[0] - length / 2, boxcenter[1] - length / 2, boxcenter[2] - height / 2); }
	vec3 getpos2() { return boxcenter - vec3(boxcenter[0] + length / 2, boxcenter[1] + length / 2, boxcenter[2] + height / 2); }
	
protected:
	std::vector<std::vector<std::vector<int>>> voxelindex;
};

