#include "Voxelization.h"
#include "rgbd_pointcloud.h"


#include <iostream>
#include <algorithm>

Voxelization::Voxelization() : pixel_depth_tex("flt32[R]"), v_id_tex("flt32[R,G,B,A]") {


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


}

	//bool Voxelization::init_voxelization(cgv::render::context& ctx, const float voxel_size, vec3 min, vec3 max, std::vector<Mat> inver_r, std::vector<vec3> inver_t, std::vector< std::vector<std::vector<depthpixel>>> depthimageplane)
	//{
	//	if (!voxelize_prog.build_program(ctx, "voxel_distance.glpr", true)) {
	//		std::cerr << "ERROR in building shader program "  << std::endl;
	//		return false;
	//	}

	//	
	//	//unsigned group_size = step;
	//	vec3 vre = max - min;
	//	
	//	pixel_depth_tex.destruct(ctx);
	//	v_id_tex.destruct(ctx);
	//	
	//	std::vector<float> depth_data(depthimageplane.size() * 576 * 640);
	//	
	//	int count_depthdata=0;
	//	for(int imageid=0; imageid< depthimageplane.size(); imageid++)
	//	for (int i = 0; i < 576; i++)
	//	{
	//		for (int j = 0; j < 640; j++)
	//		{
	//			depth_data[count_depthdata] = depthimageplane[imageid][i][j].depthsquare;
	//			
	//			count_depthdata++;
	//		}
	//	}		
	//	

	//	cgv::data::data_format vol_df(depthimageplane.size(), 576, 640, cgv::type::info::TypeId::TI_FLT32, cgv::data::ComponentFormat::CF_R);
	//	cgv::data::const_data_view vol_dv(&vol_df, &depth_data.front());
	//	pixel_depth_tex.create(ctx, vol_dv, 0);
	//	pixel_depth_tex.generate_mipmaps(ctx);
	//
	//	uvec3 num_groups = ceil(vec3(vre) / (float)voxel_size);
	//			
	//	if (!v_id_tex.is_created())
	//		v_id_tex.create(ctx, cgv::render::TT_3D, num_groups[0], num_groups[1], num_groups[2]);


	//	const int depth_tex_handle = (const int&)pixel_depth_tex.handle - 1;
	//	const int v_id_tex_handle = (const int&)v_id_tex.handle - 1;
	//	glBindImageTexture(0, depth_tex_handle, 0, GL_TRUE, 0, GL_READ_ONLY, GL_R32F);
	//	glBindImageTexture(1, v_id_tex_handle, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA32F);

	//	voxelize_prog.enable(ctx);
	//	voxelize_prog.set_uniform(ctx, "min", min);
	//	voxelize_prog.set_uniform(ctx, "max", max);
	//	voxelize_prog.set_uniform(ctx, "voxel_size", voxel_size);
	//	voxelize_prog.set_uniform(ctx, "resolution", num_groups);

	//	voxelize_prog.set_uniform(ctx, "inver_t1", inver_t[0]);
	//	voxelize_prog.set_uniform(ctx, "inver_t2", inver_t[1]);
	//	voxelize_prog.set_uniform(ctx, "inver_t3", inver_t[2]);
	//	voxelize_prog.set_uniform(ctx, "inver_r1", inver_r[0]);
	//	voxelize_prog.set_uniform(ctx, "inver_r2", inver_r[1]);
	//	voxelize_prog.set_uniform(ctx, "inver_r3", inver_r[2]);
	//		
	//	//voxelize_prog.set_uniform_array(ctx, "inver_r", inver_r);
	//	//voxelize_prog.set_uniform_array(ctx, "inver_t", inver_t);

	//	int a1 = depthimageplane.size();
	//	int a2 = depthimageplane[0].size();
	//	int a3 = depthimageplane[0][0].size();
	//	voxelize_prog.set_uniform(ctx, "depth_x_size", a1);
	//	voxelize_prog.set_uniform(ctx, "depth_y_size", a2);
	//	voxelize_prog.set_uniform(ctx, "depth_z_size", a3);


	//	glDispatchCompute(num_groups[0], num_groups[1], num_groups[2]);

	//	int length = num_groups[0] * num_groups[1] * num_groups[2];
	//			
	//	// do something else
	//	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	//	voxelize_prog.disable(ctx);
	//	// clear 3D image bindings
	//	glBindImageTexture(0, 0, 0, GL_TRUE, 0, GL_READ_ONLY, GL_R32F);
	//	glBindImageTexture(1, 0, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA32F);

	//	// read texture into memory
	//	std::vector<vec4> voxelID_data(length, vec4(0.0f));
	//	
	//	v_id_tex.enable(ctx, 0);
	//	
	//	glGetTexImage(GL_TEXTURE_3D, 0, GL_RGBA, GL_FLOAT, (void*)voxelID_data.data());
	//			
	//	v_id_tex.disable(ctx);

	//	//std::cout << "2: " << voxelID_data[0] << std::endl;
	//	//std::cerr << "voxelID_data: " << voxelID_data [3]<< std::endl;

	//	return true;






	//}
	bool Voxelization::get_surface_from_PC(rgbd_pointcloud pc, vec3 min, vec3 max, float voxel_size) {
		
	
	
		return false;
	}
	bool Voxelization::init_surface_from_PC(std::vector<rgbd_pointcloud> pc, vec3 min, vec3 max, float voxel_length) {
		V.clear();					
		V_size = ceil((max - min) / (float)voxel_size);
		V.resize(V_size[0]* V_size[1] * V_size[2]);
		for(int i = 0; i < pc.size(); i++)
			for (int j = 0; j < pc[i].get_nr_Points(); j++)
			{
				uvec3 a = ceil((pc[i].pnt(j) - min) / (float)voxel_size);				
				V[a[0] * V_size[1] * V_size[2] + a[1] * V_size[2] + a[2] - 1].loaction = 1;
			}
		Boundingbox.pos1 = min;
		Boundingbox.pos2 = max;
		voxel_size = voxel_length;
		return true;
	}
	bool Voxelization::init_voxelization(cgv::render::context& ctx) {
			if (!voxelize_prog.build_program(ctx, "voxel_distance.glpr", true)) {
				std::cerr << "ERROR in building shader program "  << std::endl;
				return false;
			}

		//============================================================================================	
			//unsigned group_size = step;
			vec3 vre = max - min;
			
			pixel_depth_tex.destruct(ctx);
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
			pixel_depth_tex.create(ctx, vol_dv, 0);
			pixel_depth_tex.generate_mipmaps(ctx);
		
			uvec3 num_groups = ceil(vec3(vre) / (float)voxel_size);
					
			if (!v_id_tex.is_created())
				v_id_tex.create(ctx, cgv::render::TT_3D, num_groups[0], num_groups[1], num_groups[2]);


			const int depth_tex_handle = (const int&)pixel_depth_tex.handle - 1;
			const int v_id_tex_handle = (const int&)v_id_tex.handle - 1;
			glBindImageTexture(0, depth_tex_handle, 0, GL_TRUE, 0, GL_READ_ONLY, GL_R32F);
			glBindImageTexture(1, v_id_tex_handle, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA32F);

			voxelize_prog.enable(ctx);
			voxelize_prog.set_uniform(ctx, "min", min);
			voxelize_prog.set_uniform(ctx, "max", max);
			voxelize_prog.set_uniform(ctx, "voxel_size", voxel_size);
			voxelize_prog.set_uniform(ctx, "resolution", num_groups);

			voxelize_prog.set_uniform(ctx, "inver_t1", inver_t[0]);
			voxelize_prog.set_uniform(ctx, "inver_t2", inver_t[1]);
			voxelize_prog.set_uniform(ctx, "inver_t3", inver_t[2]);
			voxelize_prog.set_uniform(ctx, "inver_r1", inver_r[0]);
			voxelize_prog.set_uniform(ctx, "inver_r2", inver_r[1]);
			voxelize_prog.set_uniform(ctx, "inver_r3", inver_r[2]);
				
			//voxelize_prog.set_uniform_array(ctx, "inver_r", inver_r);
			//voxelize_prog.set_uniform_array(ctx, "inver_t", inver_t);

			int a1 = depthimageplane.size();
			int a2 = depthimageplane[0].size();
			int a3 = depthimageplane[0][0].size();
			voxelize_prog.set_uniform(ctx, "depth_x_size", a1);
			voxelize_prog.set_uniform(ctx, "depth_y_size", a2);
			voxelize_prog.set_uniform(ctx, "depth_z_size", a3);


			glDispatchCompute(num_groups[0], num_groups[1], num_groups[2]);

			int length = num_groups[0] * num_groups[1] * num_groups[2];
					
			// do something else
			glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
			voxelize_prog.disable(ctx);
			// clear 3D image bindings
			glBindImageTexture(0, 0, 0, GL_TRUE, 0, GL_READ_ONLY, GL_R32F);
			glBindImageTexture(1, 0, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA32F);

			// read texture into memory
			std::vector<vec4> voxelID_data(length, vec4(0.0f));
			
			v_id_tex.enable(ctx, 0);
			
			glGetTexImage(GL_TEXTURE_3D, 0, GL_RGBA, GL_FLOAT, (void*)voxelID_data.data());
					
			v_id_tex.disable(ctx);

			//std::cout << "2: " << voxelID_data[0] << std::endl;
			//std::cerr << "voxelID_data: " << voxelID_data [3]<< std::endl;

			return true;
	
	}
	bool Voxelization::travser_voxels(cgv::render::context& ctx,std::vector<vec3>cam_pos) {
		if (!V_size)
			return false;
		uvec3 num_groups = V_size;
		voxelize_prog.enable(ctx);
		voxelize_prog.set_uniform(ctx, "min", Boundingbox.pos1);
		voxelize_prog.set_uniform(ctx, "max", Boundingbox.pos2);
		voxelize_prog.set_uniform(ctx, "voxel_size", voxel_size);
		voxelize_prog.set_uniform(ctx, "resolution", num_groups);
		glDispatchCompute(num_groups[0], num_groups[1], num_groups[2]);
		int length = num_groups[0] * num_groups[1] * num_groups[2];
		// do something else
		glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
		voxelize_prog.disable(ctx);


		return true;
	}




