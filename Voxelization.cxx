#include "Voxelization.h"



#include <iostream>
#include <algorithm>


#define OBJBOUNDARY_SSB_BP        9
#define OBJINSIDE_SSB_BP         10
#define DENOISEDOBJ_SSB_BP        11
#define CUBES_SSB_BP        12

#define BOXARRAY_SSB_BP        13

	




	bool Voxelization::init_voxelization(cgv::render::context& ctx) {
			if (!fill_prog.build_program(ctx, "glsl/filling.glpr", true)) {
				std::cerr << "ERROR in building shader program "  << std::endl;
				return false;
			}	
		
			if (!denoise_prog.build_program(ctx, "glsl/denoise.glpr", true)) {
				std::cerr << "ERROR in building shader program " << std::endl;
				return false;
			}
			if (!remove_outlier_prog.build_program(ctx, "glsl/remove_outlier.glpr", true)) {
				std::cerr << "ERROR in building shader program " << std::endl;
				return false;
			}
			return true;
	
	}

	bool Voxelization::init_boundary_from_PC(std::vector<rgbd_pointcloud> pc, vec3 min, vec3 max, float side) {
		//V.clear();

		//V_1.clear();

		////V_color.clear();
		////num_p_in_voxel.clear();
		//side_length = side;
		//V_size = ceil((max - min) / (float)side_length);
		//int length = V_size[0] * V_size[1] * V_size[2];
		//V.resize(length,0);
		////V_color.resize(length, vec3(1, 1, 1));
		////num_p_in_voxel.resize(length, 0);
		//
		////float r_v, g_v, b_v;
		//for(int i = 0; i < pc.size(); i++)
		//	for (int j = 0; j < pc[i].get_nr_Points(); j++)
		//	{
		//		uvec3 v1 = ceil((pc[i].pnt(j) - min) / (float)side_length);
		//		int temp = (v1[2]-1) * V_size[1] * V_size[0] + (v1[1]-1) * V_size[0] + v1[0] - 1;

		//		/*if (V[temp] == 0)
		//		{
		//		V_1.push_back(temp);*/
		//		V[temp] = 1;
		//		//}
		//		

		//		

		//		//color
		//		/*
		//		r_v = float(pc[i].clr(j)[0]);
		//		g_v = float(pc[i].clr(j)[1]);
		//		b_v = float(pc[i].clr(j)[2]);
		//		V_color[temp] = vec3((r_v+ num_p_in_voxel[temp]* V_color[temp][0])/ (num_p_in_voxel[temp]+1), 
		//			(g_v + num_p_in_voxel[temp] * V_color[temp][1]) / (num_p_in_voxel[temp] + 1), 
		//			(b_v + num_p_in_voxel[temp] * V_color[temp][1]) / (num_p_in_voxel[temp] + 1)
		//			);
		//		num_p_in_voxel[temp] += 1;*/
		//	}

		//

		////voxelboundingbox.pos1 = min;
		////voxelboundingbox.pos2 = max;
		//min_pos = min;
		//max_pos = max;
		////====================================================================================

		//
		//V_1.clear();

		//for (int i = 0; i < V.size(); i++) {
		//	if (V[i] != 0)
		//		V_1.push_back(i);
		//}








		return true;
	}

	bool Voxelization::denoising(cgv::render::context& ctx,int filter_threshold,int kernel_range) {
		
			



		//if (!V_size)
		//	return false;
		//uvec3 num_groups = V_size;


		//int V_length = num_groups[0] * num_groups[1] * num_groups[2];
		////std::vector<int> results(V.begin(),V.end());
		//std::vector<int> results(V_length, 0);

		//P_tex.destruct(ctx);		


		//cgv::data::data_format ptex_df(num_groups[0], num_groups[1], num_groups[2], cgv::type::info::TypeId::TI_FLT32, cgv::data::ComponentFormat::CF_R);
		//cgv::data::const_data_view ptex_dv(&ptex_df, &V.front());
		//P_tex.create(ctx, ptex_dv, 0);
		//P_tex.generate_mipmaps(ctx);


		//const int P_tex_handle = (const int&)P_tex.handle - 1;
		//
		//glBindImageTexture(0, P_tex_handle, 0, GL_TRUE, 0, GL_READ_ONLY, GL_R32F);
		//


		//denoise_prog.set_uniform(ctx, "resolution", num_groups);
		//denoise_prog.set_uniform(ctx, "threshold", filter_threshold);
		//denoise_prog.set_uniform(ctx, "range", kernel_range);


		//glNamedBufferData(V_results_buffer, sizeof(int) * V_length, results.data(), GL_DYNAMIC_READ);
		//glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, V_results_buffer);
		//
		//glGenBuffers(1, &V_1_buffer);
		//glBindBuffer(GL_SHADER_STORAGE_BUFFER, V_1_buffer);//
		//glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(int)* V_1.size(), V_1.data(), GL_DYNAMIC_READ); //sizeof(data) only works for statically sized C/C++ arrays.

		//glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, V_1_buffer);

		//denoise_prog.enable(ctx);
		//
		//glDispatchCompute(V_1.size() , 1, 1);

		//// do something else
		//glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
		//denoise_prog.disable(ctx);

		//// clear 3D image bindings
		//glBindImageTexture(0, 0, 0, GL_TRUE, 0, GL_READ_ONLY, GL_R32F);		

		//glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

		//GLuint* results_ptr = static_cast<GLuint*>(glMapNamedBufferRange(
		//	V_results_buffer, 0, sizeof(int) * V_length, GL_MAP_READ_BIT));

		//std::memcpy(results.data(), results_ptr, results.size() * sizeof(int));
		//glUnmapNamedBuffer(V_results_buffer);
		//			
		//std::vector<float> V_float(results.begin(), results.end());

		//V = V_float;

		//V_1.clear();

		//for (int i = 0; i < V.size(); i++) {
		//	if (V[i] != 0)
		//		V_1.push_back(i);
		//}



		return true;


	}
	
	bool Voxelization::traverse_voxels(cgv::render::context& ctx, std::vector<vec3> cam_pos) {
		//voxelize_prog.build_program(ctx, "glsl/voxel_d.glpr", true);
		

		//if (!V_size)
		//	return false;
		//uvec3 num_groups = V_size;

		////std::vector<float> V_data;
		////clear
		//V_tex.destruct(ctx);
		//V_new_tex.destruct(ctx);
		//
		//cgv::data::data_format vtex_df(num_groups[0], num_groups[1], num_groups[2], cgv::type::info::TypeId::TI_FLT32, cgv::data::ComponentFormat::CF_R);
		//cgv::data::const_data_view vtex_dv(&vtex_df, &V.front());		
		//V_tex.create(ctx, vtex_dv, 0);
		//V_tex.generate_mipmaps(ctx);

		////if (!V_new_tex.is_created())
		//V_new_tex.create(ctx, cgv::render::TT_3D, num_groups[0], num_groups[1], num_groups[2]);

		//const int V_tex_handle = (const int&)V_tex.handle - 1;
		//const int V_new_tex_handle = (const int&)V_new_tex.handle - 1;
		//glBindImageTexture(2, V_tex_handle, 0, GL_TRUE, 0, GL_READ_ONLY, GL_R32F);
		//glBindImageTexture(3, V_new_tex_handle, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA32F);

		//fill_prog.enable(ctx);
		//fill_prog.set_uniform(ctx, "min_pos", min_pos);
		//fill_prog.set_uniform(ctx, "max_pos", max_pos);
		//fill_prog.set_uniform(ctx, "side_length", side_length);
		//fill_prog.set_uniform(ctx, "cubeGridDims", num_groups);
		//fill_prog.set_uniform(ctx, "cam_pos1", cam_pos[0]);
		//fill_prog.set_uniform(ctx, "cam_pos2", cam_pos[1]);
		//fill_prog.set_uniform(ctx, "cam_pos3", cam_pos[2]);
		//glDispatchCompute(num_groups[0], num_groups[1], num_groups[2]);
		//int length = num_groups[0] * num_groups[1] * num_groups[2];
		//// do something else
		//glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
		//fill_prog.disable(ctx);
		//// clear 3D image bindings
		//glBindImageTexture(2, 0, 0, GL_TRUE, 0, GL_READ_ONLY, GL_R32F);
		//glBindImageTexture(3, 0, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA32F);

		//// read texture into memory
		//std::vector<vec4> V_new_data(length, vec4(0.0f));

		//V_new_tex.enable(ctx, 0);

		//glGetTexImage(GL_TEXTURE_3D, 0, GL_RGBA, GL_FLOAT, (void*)V_new_data.data());

		//V_new_tex.disable(ctx);
		//
		//for (int i = 0; i < V_new_data.size(); i++)
		//{
		//	
		//		V[i] = V_new_data[i][0];

		//}
		//
		//V_1.clear();

		//for (int i = 0; i < V.size(); i++) {
		//	if (V[i] != 0)
		//		V_1.push_back(i);
		//}


		return true;
	}

	void Voxelization::draw_voxels(cgv::render::context& ctx){
		
		if (Object_Boundary.size() == 0)
			return;

		cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
		boxes.clear();
		box_colors.clear();  
		rgb8 clr(0.3,0.3, 0.3);
		rgb table_clr(0.3f, 0.2f, 0.0f);
		rgb table_clr2(0.9f, 0.0f, 0.0f);
		for (int i = 1; i <= Voxel_size[0];i++)
			for (int j = 1; j <= Voxel_size[1]; j++)
				for(int k = 1;k <= Voxel_size[2]; k++)
				{
					int index = (k - 1) * Voxel_size[1] * Voxel_size[0] + (j - 1) * Voxel_size[0] + i - 1;
					if (Object_Boundary[index] == 1) {
					vec3 min1 = min_pos + vec3((i-1)*side_length, (j - 1) *side_length, (k - 1) *side_length);
					vec3 max1 = min_pos + vec3(i*side_length, j*side_length, k*side_length);
					boxes.emplace_back(box3(min1, max1));
					//int r = int(V_color[index_V][0]);
					//int g = int(V_color[index_V][1]);
					//int b = int(V_color[index_V][2]);
					//rgb8 clr(r,g,b);
					box_colors.emplace_back(clr);
					}else if (Object_Boundary[index] == 2)
					{
						vec3 min1 = min_pos + vec3((i - 1)*side_length, (j - 1) *side_length, (k - 1) *side_length);
						vec3 max1 = min_pos + vec3(i*side_length, j*side_length, k*side_length);
						boxes.emplace_back(box3(min1, max1));
						box_colors.emplace_back(clr);
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
	void Voxelization::get_center_gravity() {
		int m = V_1.size();
		int x, y, z;
		vec3 sum = vec3(0, 0, 0);
		for (int i = 0; i < V_1.size(); i++)
		{
			z = int(floor(V_1[i] / (V_size[0] * V_size[1])));
			y = int(floor((V_1[i] - (V_size[0] * V_size[1])*z) / V_size[0]));
			x = int(V_1[i] - (V_size[0] * V_size[1])*z - V_size[0] * y);
			sum = sum + vec3(x+ side_length /2, y + side_length / 2, z + side_length / 2);


		}
		center_gravity = sum / m;
		
	}

	bool Voxelization::init(std::vector<rgbd_pointcloud> pc, vec3 min, vec3 max, float side) {

		
		min_pos = min;
		max_pos = max;
		side_length = side;
		Voxel_size = ceil((max - min) / (float)side_length);
		int length = Voxel_size[0] * Voxel_size[1] * Voxel_size[2];
		Object_Boundary.resize(length, 0);
		for (int i = 0; i < pc.size(); i++)
			for (int j = 0; j < pc[i].get_nr_Points(); j++)
			{
				ivec3 v1 = ceil((pc[i].pnt(j) - min) / (float)side_length);
				int temp = (v1[2] - 1) * Voxel_size[1] * Voxel_size[0] + (v1[1] - 1) * Voxel_size[0] + v1[0] - 1;
				Object_Boundary[temp] = 1;
			}
		deleteBuffers();
		
		return true;
	}

	bool Voxelization::generate(cgv::render::context& ctx, std::vector<vec3> cam_pos) 
	{
		

		//init_voxelization(ctx);
		

		createBuffers();	
		bindbuffer();
		//calculate the gridpoints weight
		//if it is inside the object,its value become 1
		
		int length = Voxel_size[0] * Voxel_size[1] * Voxel_size[2];
		int filter_threshold = 5;
		int kernel_range = 3;
		denoise_prog.set_uniform(ctx, "cubeGridDims", Voxel_size);
		denoise_prog.set_uniform(ctx, "threshold", filter_threshold);
		denoise_prog.set_uniform(ctx, "range", kernel_range);
		denoise_prog.enable(ctx);
		glDispatchCompute(Voxel_size[0], Voxel_size[1], Voxel_size[2]);
		glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
		denoise_prog.disable(ctx);
		

		
	
		
		fill_prog.set_uniform(ctx, "min_pos", min_pos);
		fill_prog.set_uniform(ctx, "max_pos", max_pos);
		fill_prog.set_uniform(ctx, "side_length", side_length);
		fill_prog.set_uniform(ctx, "cubeGridDims", Voxel_size);
		fill_prog.set_uniform(ctx, "cam_pos1", cam_pos[0]);
		fill_prog.set_uniform(ctx, "cam_pos2", cam_pos[1]);
		fill_prog.set_uniform(ctx, "cam_pos3", cam_pos[2]);
		fill_prog.enable(ctx);
		glDispatchCompute(Voxel_size[0], Voxel_size[1], Voxel_size[2]);
		glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
		fill_prog.disable(ctx);
		
		

		GLuint zero = 0;
		boxarray.setSubData(0, sizeof(GLuint), &zero);
		filter_threshold = 13;
		kernel_range = 5;
		remove_outlier_prog.set_uniform(ctx, "cubeGridDims", Voxel_size);
		remove_outlier_prog.set_uniform(ctx, "threshold", filter_threshold);
		remove_outlier_prog.set_uniform(ctx, "range", kernel_range);
		remove_outlier_prog.set_uniform(ctx, "side_length", side_length);	
		remove_outlier_prog.set_uniform(ctx, "min_pos", min_pos);
		remove_outlier_prog.set_uniform(ctx, "max_pos", max_pos);

		remove_outlier_prog.enable(ctx);
		glDispatchCompute(Voxel_size[0], Voxel_size[1], Voxel_size[2]);
		glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
		remove_outlier_prog.disable(ctx);










		auto start_draw = std::chrono::steady_clock::now();
		//float a[64];
		//std::vector<float> d(64);
		Object_Boundary.clear();
		Object_Boundary.resize(length);
		cubes.getSubData(0, sizeof(float) * length, static_cast<void*>(Object_Boundary.data()));

		auto stop_draw = std::chrono::steady_clock::now();
		std::chrono::duration<double> diff_draw;
		diff_draw = stop_draw - start_draw;
		std::cout << diff_draw.count() << std::endl;
		//for (int i = 0; i < 64; i++)
		//	std::cout<<d[i]<<std::endl;
		//GLuint* results_ptr = static_cast<GLuint*>(glMapNamedBufferRange(
		//	V_results_buffer, 0, sizeof(int) * V_length, GL_MAP_READ_BIT));

		//std::memcpy(results.data(), results_ptr, results.size() * sizeof(int));
		//glUnmapNamedBuffer(V_results_buffer);
		//			
		//std::vector<float> V_float(results.begin(), results.end());
		return true;
	}

	void Voxelization::bindbuffer()
	{

		object_boundary.setBindingPoint(OBJBOUNDARY_SSB_BP);
		filled_object.setBindingPoint(OBJINSIDE_SSB_BP);
		denoised_object.setBindingPoint(DENOISEDOBJ_SSB_BP);
		cubes.setBindingPoint(CUBES_SSB_BP);
		boxarray.setBindingPoint(BOXARRAY_SSB_BP);
		
	}

	void Voxelization::createBuffers() 
	{

		int length = Voxel_size[0] * Voxel_size[1] * Voxel_size[2];
		
		object_boundary = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, length * sizeof(float));
		object_boundary.setSubData(0, sizeof(float)* Object_Boundary.size(),  Object_Boundary.data());
		filled_object = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, length * sizeof(float));
		denoised_object = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, length * sizeof(float));
		cubes = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, length * sizeof(float));
		boxarray = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, length *6* sizeof(float)+ sizeof(GLuint));
		//center_mass = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, length * 6 * sizeof(float));
	}
	void Voxelization::deleteBuffers()
	{

		object_boundary.deleteBuffer();
		filled_object.deleteBuffer();
		denoised_object.deleteBuffer();
		cubes.deleteBuffer();
		boxarray.deleteBuffer();
		
		
	}
