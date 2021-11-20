#include "Voxelization.h"
#include "rgbd_pointcloud.h"


#include <iostream>
#include <algorithm>


	bool Voxelization::init_voxelization(cgv::render::context& ctx, const float step, vec3 min, vec3 max)//, std::vector<std::vector<depthpixel>> depthimageplane, std::vector<vec3> cam_pos)
	{
		if (!voxelize_prog.build_program(ctx, "voxel_distance.glpr", true)) {
			std::cerr << "ERROR in building shader program "  << std::endl;
			return false;
		}

		unsigned voxel_size = 1; //0.02;
		//unsigned group_size = step;
		vec3 vre = max - min;
		

		
		vre[0] = abs(vre[0]);
		vre[1] = abs(vre[1]);
		vre[2] = abs(vre[2]);

		uvec3 num_groups = ceil(vec3(vre) / (float)voxel_size);

		voxelize_prog.enable(ctx);
		voxelize_prog.set_uniform(ctx, "min", min);
		voxelize_prog.set_uniform(ctx, "max", max);
		voxelize_prog.set_uniform(ctx, "voxel_size", voxel_size);
		voxelize_prog.set_uniform(ctx, "resolution", vre);

		//voxelize_prog.set_uniform(ctx, "cam_pos1", cam_pos[0]);
		//voxelize_prog.set_uniform(ctx, "cam_pos2", cam_pos[1]);
		//voxelize_prog.set_uniform(ctx, "cam_pos3", cam_pos[2]);

		//voxelize_prog.set_uniform(ctx, "step", step);
		//voxelize_prog.set_uniform(ctx, "gradient_mode", (int)gradient_mode);

		glDispatchCompute(num_groups[0], num_groups[1], num_groups[2]);

		int length = num_groups[0] * num_groups[1] * num_groups[2];


		glGenBuffers(1, &m_cntBuffer);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_cntBuffer);
		glBufferData(GL_SHADER_STORAGE_BUFFER, length * sizeof(int), nullptr, GL_STATIC_DRAW);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, m_cntBuffer);


		int* writePtr = reinterpret_cast<int*>(glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_WRITE_ONLY));
		for (int x = 0; x < length; ++x)
		{
			writePtr[x] = 0;
		}
		if (!glUnmapBuffer(GL_SHADER_STORAGE_BUFFER))
			std::cout << "unMap error\n" << std::endl;
		// do something else

		glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
		voxelize_prog.disable(ctx);
		std::cout<< writePtr[0] <<std::endl;

		

		
			
		return false;






	}



