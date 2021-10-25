#include "Voxelization.h"
#include "rgbd_pointcloud.h"
#include "voxel_renderer.h"

#include <iostream>
#include <algorithm>

namespace voxel
{
	bool Voxelization::voxelize(cgv::render::context& ctx, const float& step, std::vector<vec3>& ret,vec3 min,vec3 max, rgbd_pointcloud pc)
	{
		ret.clear();

		//shader
		/*if (!voxelize_prog.is_linked()) {
			if (!voxelize_prog.build_program(ctx, "voxelize.glpr", true)) {
				std::cerr << "could not build voxelize_prog shader program" << std::endl;
				abort();
			}
		}*/


		//bounding box info
				
		ivec3 resolution;

		vec3 range(max.x() - min.x(), max.y() - min.y(), max.z() - min.z());
		resolution.x() = static_cast<int>(range.x() / step) + 1;
		resolution.y() = static_cast<int>(range.y() / step) + 1;
		resolution.z() = static_cast<int>(range.z() / step) + 1;
		int length = static_cast<int>((resolution.x()) * (resolution.y()) * (resolution.z()));

		
		//point cloud info

		std::vector<vec3> mypc;
		std::vector<Rgba> mypc_color;
		
		mypc = pc.getPoints();
		mypc_color = pc.getColors();
		
		std::vector<box3> myboxes;
		rgb box_clr(0.3f, 0.2f, 0.0f);
		std::vector<rgb> box_colors;
		myboxes.push_back(box3(vec3(-0.5f , -0.5f, -0.5f ), vec3(0.5f , 0.5f, 0.5f )));
		box_colors.push_back(box_clr);


		cgv::render::voxel_renderer& renderer = cgv::render::ref_voxel_renderer(ctx);
		cgv::render::voxel_render_style style;
		renderer.set_render_style(style);
		renderer.set_voxel_array(ctx, myboxes);
		renderer.set_color_array(ctx, box_colors);
		if (renderer.validate_and_enable(ctx)) {
			glDrawArrays(GL_POINTS, 0, (GLsizei)myboxes.size());
		}
		renderer.disable(ctx);





		





		

		
		

		return false;






	}



}