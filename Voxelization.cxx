#include "Voxelization.h"
#include "rgbd_pointcloud.h"


#include <iostream>
#include <algorithm>

Voxelization::Voxelization() : depth_tex("flt32[R]"), v_id_tex("flt32[R,G,B,A]") {


	depth_tex.set_min_filter(cgv::render::TF_LINEAR_MIPMAP_LINEAR);
	depth_tex.set_mag_filter(cgv::render::TF_LINEAR);
	depth_tex.set_wrap_s(cgv::render::TW_CLAMP_TO_BORDER);
	depth_tex.set_wrap_t(cgv::render::TW_CLAMP_TO_BORDER);
	depth_tex.set_wrap_r(cgv::render::TW_CLAMP_TO_BORDER);
	depth_tex.set_border_color(0.0f, 0.0f, 0.0f, 0.0f);

	v_id_tex.set_min_filter(cgv::render::TF_LINEAR);
	v_id_tex.set_mag_filter(cgv::render::TF_LINEAR);
	v_id_tex.set_wrap_s(cgv::render::TW_CLAMP_TO_EDGE);
	v_id_tex.set_wrap_t(cgv::render::TW_CLAMP_TO_EDGE);
	v_id_tex.set_wrap_r(cgv::render::TW_CLAMP_TO_EDGE);


}

	bool Voxelization::init_voxelization(cgv::render::context& ctx, const float step, vec3 min, vec3 max, std::vector<Mat> inver_r, std::vector<vec3> inver_t, std::vector< std::vector<std::vector<depthpixel>>> depthimageplane)
	{
		if (!voxelize_prog.build_program(ctx, "voxel_distance.glpr", true)) {
			std::cerr << "ERROR in building shader program "  << std::endl;
			return false;
		}

		unsigned voxel_size = 0.1; //0.02;
		//unsigned group_size = step;
		vec3 vre = max - min;
		

		depth_tex.destruct(ctx);
		v_id_tex.destruct(ctx);
		
		std::vector<float> depth_data(depthimageplane.size() * 576 * 640);
		
		int count_depthdata=0;
		for(int imageid=0; imageid< depthimageplane.size(); imageid++)
		for (int i = 0; i < 576; i++)
		{
			for (int j = 0; j < 640; j++)
			{
				depth_data[count_depthdata] = depthimageplane[imageid][i][j].depthsquare;
				
				count_depthdata++;
			}
		}

		

		cgv::data::data_format vol_df(depthimageplane.size(), 576, 640, cgv::type::info::TypeId::TI_FLT32, cgv::data::ComponentFormat::CF_R);
		cgv::data::const_data_view vol_dv(&vol_df, &depth_data.front());
		depth_tex.create(ctx, vol_dv, 0);
		depth_tex.generate_mipmaps(ctx);

		if (!v_id_tex.is_created())
			v_id_tex.create(ctx, cgv::render::TT_3D, depthimageplane.size(), 576, 640);


		const int depth_tex_handle = (const int&)depth_tex.handle - 1;
		const int v_id_tex_handle = (const int&)v_id_tex.handle - 1;
		glBindImageTexture(0, depth_tex_handle, 0, GL_TRUE, 0, GL_READ_ONLY, GL_R32F);
		glBindImageTexture(1, v_id_tex_handle, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA32F);


		uvec3 num_groups = ceil(vec3(vre) / (float)voxel_size);

		voxelize_prog.enable(ctx);
		voxelize_prog.set_uniform(ctx, "min", min);
		voxelize_prog.set_uniform(ctx, "max", max);
		voxelize_prog.set_uniform(ctx, "voxel_size", voxel_size);
		voxelize_prog.set_uniform(ctx, "resolution", vre);

		voxelize_prog.set_uniform(ctx, "inver_t1", inver_t[0]);
		voxelize_prog.set_uniform(ctx, "inver_t2", inver_t[1]);
		voxelize_prog.set_uniform(ctx, "inver_t3", inver_t[2]);
		voxelize_prog.set_uniform(ctx, "inver_r1", inver_r[0]);
		voxelize_prog.set_uniform(ctx, "inver_r2", inver_r[1]);
		voxelize_prog.set_uniform(ctx, "inver_r3", inver_r[2]);
		voxelize_prog.set_uniform(ctx, "step", step);
		//voxelize_prog.set_uniform(ctx, "gradient_mode", (int)gradient_mode);
		

		glDispatchCompute(num_groups[0], num_groups[1], num_groups[2]);

		//int length = num_groups[0] * num_groups[1] * num_groups[2];


		//glGenBuffers(1, &m_cntBuffer);
		//glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_cntBuffer);
		//glBufferData(GL_SHADER_STORAGE_BUFFER, length * sizeof(int), nullptr, GL_STATIC_DRAW);
		//glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, m_cntBuffer);


		//int* writePtr = reinterpret_cast<int*>(glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_WRITE_ONLY));
		//for (int x = 0; x < length; ++x)
		//{
		//	writePtr[x] = 0;
		//}
		//if (!glUnmapBuffer(GL_SHADER_STORAGE_BUFFER))
		//	std::cout << "unMap error\n" << std::endl;
		
		
		// do something else
		glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
		voxelize_prog.disable(ctx);
		// clear 3D image bindings
		glBindImageTexture(0, 0, 0, GL_TRUE, 0, GL_READ_ONLY, GL_R32F);
		glBindImageTexture(1, 0, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA32F);


		// read texture into memory
		std::vector<vec4> voxelID_data(num_groups[0] * num_groups[1] * num_groups[2], vec4(0.0f));

		v_id_tex.enable(ctx, 0);
		glGetTexImage(GL_TEXTURE_3D, 0, GL_RGBA, GL_FLOAT, (void*)voxelID_data.data());
		v_id_tex.disable(ctx);
		
		//std::cerr << "voxelID_data: " << voxelID_data [3]<< std::endl;

		return true;






	}



