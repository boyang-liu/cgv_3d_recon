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
		







		return true;
	}

	bool Voxelization::denoising(cgv::render::context& ctx,int filter_threshold,int kernel_range) {
		
			




		return true;


	}
	
	bool Voxelization::traverse_voxels(cgv::render::context& ctx, std::vector<vec3> cam_pos) {
		


		return true;
	}

	void Voxelization::draw_voxels(cgv::render::context& ctx){
		
		if (render_content.size() == 0)
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
					if (render_content[index] == 1) {
					vec3 min1 = min_pos + vec3((i-1)*side_length, (j - 1) *side_length, (k - 1) *side_length);
					vec3 max1 = min_pos + vec3(i*side_length, j*side_length, k*side_length);
					boxes.emplace_back(box3(min1, max1));
					//int r = int(V_color[index_V][0]);
					//int g = int(V_color[index_V][1]);
					//int b = int(V_color[index_V][2]);
					//rgb8 clr(r,g,b);
					box_colors.emplace_back(clr);
					}else if (render_content[index] == 2)
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
			//glDrawArrays(GL_POINTS, 0, (GLsizei)numBoxes);
		}
		renderer.disable(ctx);
		//Object_Boundary.clear();
	
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
		Object_Boundary.clear();
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


		boxarray.getSubData(0, sizeof(GLuint), &numBoxes);

		//auto start_draw = std::chrono::steady_clock::now();
		//float a[64];
		//std::vector<float> d(64);
		Object_Boundary.clear();
		Object_Boundary.resize(length);
		cubes.getSubData(0, sizeof(float) * length, static_cast<void*>(Object_Boundary.data()));
		render_content = Object_Boundary;
		/*auto stop_draw = std::chrono::steady_clock::now();
		std::chrono::duration<double> diff_draw;
		diff_draw = stop_draw - start_draw;
		std::cout << diff_draw.count() << std::endl;*/
		 
		/*for (int i = 0; i < 64; i++)
			std::cout<<d[i]<<std::endl;
		GLuint* results_ptr = static_cast<GLuint*>(glMapNamedBufferRange(
			V_results_buffer, 0, sizeof(int) * V_length, GL_MAP_READ_BIT));

		std::memcpy(results.data(), results_ptr, results.size() * sizeof(int));
		glUnmapNamedBuffer(V_results_buffer);
					
		std::vector<float> V_float(results.begin(), results.end());*/
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
