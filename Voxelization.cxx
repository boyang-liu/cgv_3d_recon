#include "Voxelization.h"



#include <iostream>
#include <algorithm>



	bool Voxelization::init_voxelization_from_image(cgv::render::context& ctx,float myvoxel_size, vec3 min, vec3 max, std::vector<Mat> inver_r, std::vector<vec3> inver_t, std::vector< std::vector<std::vector<depthpixel>>> depthimageplane)
	{
		if (!voxelize_prog.build_program(ctx, "glsl/voxel_distance.glpr", true)) {
			std::cerr << "ERROR in building shader program "  << std::endl;
			return false;
		}

		
		//unsigned group_size = step;
		vec3 vre = max - min;
		voxel_size = myvoxel_size;
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
	
		uvec3 num_groups = ceil(vec3(vre) / (float)myvoxel_size);
				
		if (!v_id_tex.is_created())
			v_id_tex.create(ctx, cgv::render::TT_3D, num_groups[0], num_groups[1], num_groups[2]);


		const int depth_tex_handle = (const int&)pixel_depth_tex.handle - 1;
		const int v_id_tex_handle = (const int&)v_id_tex.handle - 1;
		glBindImageTexture(0, depth_tex_handle, 0, GL_TRUE, 0, GL_READ_ONLY, GL_R32F);
		glBindImageTexture(1, v_id_tex_handle, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA32F);

		voxelize_prog.enable(ctx);
		voxelize_prog.set_uniform(ctx, "min", min);
		voxelize_prog.set_uniform(ctx, "max", max);
		voxelize_prog.set_uniform(ctx, "voxel_size", myvoxel_size);
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

		std::cout << "2: " << voxelID_data[0] << std::endl;
		std::cerr << "voxelID_data: " << voxelID_data [1]<< std::endl;

		return true;






	}
	








	bool Voxelization::init_voxelization(cgv::render::context& ctx) {
			if (!voxelize_prog.build_program(ctx, "glsl/voxel_d.glpr", true)) {
				std::cerr << "ERROR in building shader program "  << std::endl;
				return false;
			}	
			
			return true;
	
	}

	bool Voxelization::init_surface_from_PC(std::vector<rgbd_pointcloud> pc, vec3 min, vec3 max, float voxel_length) {
		V.clear();	
		voxel_size = voxel_length;
		V_size = ceil((max - min) / (float)voxel_size);
		int length = V_size[0] * V_size[1] * V_size[2];
		V.resize(length,0);
		for(int i = 0; i < pc.size(); i++)
			for (int j = 0; j < pc[i].get_nr_Points(); j++)
			{
				uvec3 a = ceil((pc[i].pnt(j) - min) / (float)voxel_size);	
				int temp = (a[2]-1) * V_size[1] * V_size[0] + (a[1]-1) * V_size[0] + a[0] - 1;
				V[temp] = 1;

				//color
				//V_color[temp] = pc[i].clr(j);

			}

		//voxelboundingbox.pos1 = min;
		//voxelboundingbox.pos2 = max;
		min_pos = min;
		max_pos = max;
		//====================================================================================
		

		return true;
	}


	
	bool Voxelization::traverse_voxels(cgv::render::context& ctx, std::vector<vec3> cam_pos) {
		//voxelize_prog.build_program(ctx, "glsl/voxel_d.glpr", true);
		
		if (!V_size)
			return false;
		uvec3 num_groups = V_size;

		//std::vector<float> V_data;
		//clear
		V_tex.destruct(ctx);
		V_new_tex.destruct(ctx);
		
		cgv::data::data_format vtex_df(num_groups[0], num_groups[1], num_groups[2], cgv::type::info::TypeId::TI_FLT32, cgv::data::ComponentFormat::CF_R);
		cgv::data::const_data_view vtex_dv(&vtex_df, &V.front());		
		V_tex.create(ctx, vtex_dv, 0);
		V_tex.generate_mipmaps(ctx);

		//if (!V_new_tex.is_created())
		V_new_tex.create(ctx, cgv::render::TT_3D, num_groups[0], num_groups[1], num_groups[2]);

		const int V_tex_handle = (const int&)V_tex.handle - 1;
		const int V_new_tex_handle = (const int&)V_new_tex.handle - 1;
		glBindImageTexture(2, V_tex_handle, 0, GL_TRUE, 0, GL_READ_ONLY, GL_R32F);
		glBindImageTexture(3, V_new_tex_handle, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA32F);

		voxelize_prog.enable(ctx);
		voxelize_prog.set_uniform(ctx, "min_bbox", min_pos);
		voxelize_prog.set_uniform(ctx, "max_bbox", max_pos);
		voxelize_prog.set_uniform(ctx, "voxel_size", voxel_size);
		voxelize_prog.set_uniform(ctx, "resolution", num_groups);
		voxelize_prog.set_uniform(ctx, "cam_pos1", cam_pos[0]);
		voxelize_prog.set_uniform(ctx, "cam_pos2", cam_pos[1]);
		voxelize_prog.set_uniform(ctx, "cam_pos3", cam_pos[2]);
		glDispatchCompute(num_groups[0], num_groups[1], num_groups[2]);
		int length = num_groups[0] * num_groups[1] * num_groups[2];
		// do something else
		glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
		voxelize_prog.disable(ctx);
		// clear 3D image bindings
		glBindImageTexture(2, 0, 0, GL_TRUE, 0, GL_READ_ONLY, GL_R32F);
		glBindImageTexture(3, 0, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA32F);

		// read texture into memory
		std::vector<vec4> V_new_data(length, vec4(0.0f));

		V_new_tex.enable(ctx, 0);

		glGetTexImage(GL_TEXTURE_3D, 0, GL_RGBA, GL_FLOAT, (void*)V_new_data.data());

		V_new_tex.disable(ctx);
		
		for (int i = 0; i < V_new_data.size(); i++)
		{
			
				V[i] = V_new_data[i][0];

		}
		
		//std::cout << "V_size:" << V_size << std::endl;
		//std::cout<<"V_new_data[0]:"<< V_new_data[0] <<std::endl;
		





		/*cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
		boxes.clear();
		box_colors.clear();

		rgb table_clr(0.3f, 0.2f, 0.0f);
		rgb table_clr2(0.9f, 0.9f, 0.9f);
		for (int i = 0; i <= V_size[0]-1; i++)
			for (int j = 0; j <= V_size[1] - 1; j++)
				for (int k = 0; k <= V_size[2] - 1; k++)
				{
					if (V_new_data[k  * V_size[1] * V_size[0] + j  * V_size[0] + i ][0] == 1) {
						vec3 min1 = min_pos + vec3(i  * voxel_size, j  * voxel_size, k * voxel_size);
						vec3 max1 = min_pos + vec3((i+1) * voxel_size, (j + 1) * voxel_size, (k + 1) * voxel_size);
						boxes.emplace_back(box3(min1, max1));
						box_colors.emplace_back(table_clr);
					}
					else if (V_new_data[k  * V_size[1] * V_size[0] + j * V_size[0] + i][0] == 2)
					{
						vec3 min1 = min_pos + vec3(i  * voxel_size, j * voxel_size, k  * voxel_size);
						vec3 max1 = min_pos + vec3((i+1) * voxel_size, (j+1) * voxel_size, (k + 1) * voxel_size);
						boxes.emplace_back(box3(min1, max1));
						box_colors.emplace_back(table_clr2);
					}

				}
		cgv::render::box_render_style style;
		renderer.set_render_style(style);


		renderer.set_box_array(ctx, boxes);
		renderer.set_color_array(ctx, box_colors);
		if (renderer.validate_and_enable(ctx)) {
			glDrawArrays(GL_POINTS, 0, (GLsizei)boxes.size());
		}
		renderer.disable(ctx);*/















		return true;
	}

	void Voxelization::draw_voxels(cgv::render::context& ctx){
		
		


		cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
		boxes.clear();
		box_colors.clear();  

		rgb table_clr(0.3f, 0.2f, 0.0f);
		rgb table_clr2(0.9f, 0.9f, 0.9f);
		for (int i = 1; i <= V_size[0];i++) 
			for (int j = 1; j <= V_size[1]; j++)
				for(int k = 1;k <= V_size[2]; k++)
				{
					if (V[(k - 1) * V_size[1] * V_size[0] + (j - 1) * V_size[0] + i - 1] == 1) {
					vec3 min1 = min_pos + vec3((i-1)*voxel_size, (j - 1) *voxel_size, (k - 1) *voxel_size);
					vec3 max1 = min_pos + vec3(i*voxel_size, j*voxel_size, k*voxel_size);
					boxes.emplace_back(box3(min1, max1));
					box_colors.emplace_back(table_clr);
					}else if (V[(k - 1) * V_size[1] * V_size[0] + (j - 1) * V_size[0] + i - 1] == 2)
					{
						vec3 min1 = min_pos + vec3((i - 1)*voxel_size, (j - 1) *voxel_size, (k - 1) *voxel_size);
						vec3 max1 = min_pos + vec3(i*voxel_size, j*voxel_size, k*voxel_size);
						boxes.emplace_back(box3(min1, max1));
						box_colors.emplace_back(table_clr2);
					}
					
				}
		cgv::render::box_render_style style;
		renderer.set_render_style(style);


		renderer.set_box_array(ctx, boxes);
		renderer.set_color_array(ctx, box_colors);
		if (renderer.validate_and_enable(ctx)) {
			glDrawArrays(GL_POINTS, 0, (GLsizei)boxes.size());
		}
		renderer.disable(ctx);
	
	
	}


