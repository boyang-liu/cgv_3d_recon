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
		if (!voxelize_prog.is_linked()) {
			if (!voxelize_prog.build_program(ctx, "voxelize.glpr", true)) {
				std::cerr << "could not build voxelize_prog shader program" << std::endl;
				abort();
			}
		}


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
		

		cgv::render::voxel_renderer& renderer = cgv::render::ref_voxel_renderer(ctx);
		renderer.set_render_style(style);
		renderer.set_box_array(ctx, boxes);
		renderer.set_color_array(ctx, box_colors);
		if (renderer.validate_and_enable(ctx)) {
			glDrawArrays(GL_POINTS, 0, (GLsizei)boxes.size());
		}
		renderer.disable(ctx);





		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glDisable(GL_MULTISAMPLE);
		glDisable(GL_CULL_FACE);
		glDisable(GL_DEPTH_TEST);
		glClearColor(1.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		voxelize_prog.enable(ctx);
		

		ctx.push_modelview_matrix();
		ctx.mul_modelview_matrix(cgv::math::scale4<double>(
			max_scene_extent, max_scene_extent, max_scene_extent));
		ctx.tesselate_unit_cube();
		ctx.pop_modelview_matrix();



		//// generate ssbo.
		//glGenBuffers(1, &m_cntBuffer);
		//glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_cntBuffer);
		//glBufferData(GL_SHADER_STORAGE_BUFFER, length * sizeof(int), nullptr, GL_STATIC_DRAW);
		//glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, m_cntBuffer);

		//// bind shader and ssbo.
		//shader->bind();
		//shader->setVec3("boxMin", min);
		//shader->setFloat("step", step);
		//shader->setVec3("resolution", resolution);
		//shader->setVec2("halfPixel[0]", vec2(1.0f / resolution.z, 1.0f / resolution.y));
		//shader->setVec2("halfPixel[1]", vec2(1.0f / resolution.x, 1.0f / resolution.z));
		//shader->setVec2("halfPixel[2]", vec2(1.0f / resolution.x, 1.0f / resolution.y));
		//
		//
		//int* writePtr = reinterpret_cast<int*>(glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_WRITE_ONLY));
		//for (int x = 0; x < length; ++x)
		//{
		//	writePtr[x] = 0;
		//}
		//if (!glUnmapBuffer(GL_SHADER_STORAGE_BUFFER))
		//	std::cout << "unMap error\n" << std::endl;
		//
		//// draw and count.
		//

		////===

		sky_prog.disable(ctx);
		img_tex.disable(ctx);
		glEnable(GL_CULL_FACE);
		glDepthMask(GL_TRUE);
		

		return false;






	}



}