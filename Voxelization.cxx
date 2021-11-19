#include "Voxelization.h"
#include "rgbd_pointcloud.h"


#include <iostream>
#include <algorithm>

namespace voxel
{
	bool Voxelization::init_voxelize(cgv::render::context& ctx, const float step, vec3 min, vec3 max, std::vector<std::vector<depthpixel>> depthimageplane)
	{
		if (!voxelize_prog.build_program(ctx, "gradient_3d.glpr", true)) {
			std::cerr << "ERROR in building shader program "  << std::endl;
			return false;
		}

		unsigned group_size = 1.0; //0.02;
		//unsigned group_size = step;
		vec3 vre = max - min;
		


		vre[0] = abs(vre[0]);
		vre[1] = abs(vre[1]);
		vre[2] = abs(vre[2]);

		uvec3 num_groups = ceil(vec3(vre) / (float)group_size);

		voxelize_prog.enable(ctx);
		voxelize_prog.set_uniform(ctx, "min", min);
		voxelize_prog.set_uniform(ctx, "max", max);
		voxelize_prog.set_uniform(ctx, "step", step);
		//voxelize_prog.set_uniform(ctx, "cam_pos", pc.cam_pos);
		voxelize_prog.set_uniform(ctx,"step",step);
		//voxelize_prog.set_uniform(ctx, "gradient_mode", (int)gradient_mode);
		glDispatchCompute(num_groups[0], num_groups[1], num_groups[2]);

		// do something else

		glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
		voxelize_prog.disable(ctx);
		

		//bounding box info
				
		/*ivec3 resolution;

		vec3 range(max.x() - min.x(), max.y() - min.y(), max.z() - min.z());
		resolution.x() = static_cast<int>(range.x() / step) + 1;
		resolution.y() = static_cast<int>(range.y() / step) + 1;
		resolution.z() = static_cast<int>(range.z() / step) + 1;
		int length = static_cast<int>((resolution.x()) * (resolution.y()) * (resolution.z()));*/

		
			
		return false;






	}



}