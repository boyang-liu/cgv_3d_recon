#include "Voxelization.h"



#include <iostream>
#include <algorithm>
#include <cgv_gl/volume_renderer.h>
#include <cgv/media/volume/volume.h>



#define CUBES_SSB_BP              1
#define OBJBOUNDARY_SSB_BP        9
#define OBJINSIDE_SSB_BP          10
#define DENOISEDOBJ_SSB_BP        11



	
Voxelization::Voxelization()
{
	rcrs.radius = 0.03f;
}

Voxelization::~Voxelization()
{

}



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
			cgv::render::ref_sphere_renderer(ctx, 1);
			cgv::render::ref_rounded_cone_renderer(ctx, 1);
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

void Voxelization::draw_voxels(cgv::render::context& ctx, bool showvolume){
		/*std::vector<vec3> BoxPoses;
		BoxPoses.push_back(vec3(0, 0, 0));
		rgba clr(1.f, 1.f, 1.f, 0.5f);
		
		
		cgv::render::volume_render_style vstyle;
		cgv::render::texture density_tex;
		cgv::media::volume::volume density_volume;
		if (density_tex.is_created())
			density_tex.destruct(ctx);
		std::vector<float>& density_data = density_volume.ref_voxel_grid().data;


		auto& vr = ref_volume_renderer(ctx);				
		vr.set_render_style(vstyle);
		vr.set_volume_texture(&density_tex);
		vr.set_transfer_function_texture(&tf_editor_ptr->ref_tex());
		vr.set_bounding_box();
		vr.transform_to_bounding_box(true);
		vr.render(ctx,0,0);*/

		if (render_content.size() == 0)
			return;
		
		//renderer.set_color_array();
		
		boxes.clear();
		box_colors.clear();  
		rgba clr(1.f, 1.f, 1.f,1.0f);

		//std::vector<vec3> BoxPoses;
		//BoxPoses.push_back(vec3(0,0,0));

		center_gravity = vec3(0, 0, 0);
		vec3 sum = vec3(0, 0, 0);
		float sigma_m = 0;
		for (int i = 1; i <= Voxel_size[0];i++)
			for (int j = 1; j <= Voxel_size[1]; j++)
				for(int k = 1;k <= Voxel_size[2]; k++)
				{
					int index = (k - 1) * Voxel_size[1] * Voxel_size[0] + (j - 1) * Voxel_size[0] + i - 1;
					if (render_content[index] != 0) {
					vec3 BoxMinPos = min_pos + vec3((i-1)*side_length, (j - 1) *side_length, (k - 1) *side_length);
					vec3 BoxMaxPos = min_pos + vec3(i*side_length, j*side_length, k*side_length);
					boxes.emplace_back(box3(BoxMinPos, BoxMaxPos));
					box_colors.emplace_back(clr);
					sigma_m += 1;
					sum =sum+ (BoxMinPos + BoxMaxPos)/2 ;
					//BoxPoses.push_back(BoxMinPos);
					}
					
				}
		center_gravity = sum / sigma_m;
		//trajectory of center of mass 
	    if (center_pts.empty()) {
				center_pts.emplace_back(center_gravity);
				center_clrs.emplace_back(rgb(1.0, 0.0, 0.0));
			}
		else {
				
				center_pts.emplace_back(center_gravity);
				center_clrs.emplace_back(rgb(1.0, 0.0, 0.0));
				auto& rcr = cgv::render::ref_rounded_cone_renderer(ctx);
				rcr.set_render_style(rcrs);
				rcr.set_position_array(ctx, center_pts);
				rcr.set_color_array(ctx, center_clrs);
				rcr.render(ctx,0, center_pts.size());
				center_pts.emplace_back(center_gravity);
				center_clrs.emplace_back(rgb(1.0, 0.0, 0.0));
		}
		//draw center of mass using a sphere
		draw_center_mass(ctx, center_gravity);

		if (showvolume) {
		cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
		cgv::render::box_render_style style;		
		renderer.set_render_style(style);
		renderer.set_box_array(ctx, boxes);
		renderer.set_color_array(ctx, box_colors);
		renderer.render(ctx, 0, boxes.size());
	
		

		/*glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDepthMask(GL_FALSE);
		glDisable(GL_LIGHTING);
		glColor4f(1.0, 1.0, 0.0, 1.0);
		
		
		
		glColor4f(1.0, 0.0, 0.0, 0.5);
		glDisable(GL_BLEND);
		glEnable(GL_LIGHTING);
		glDepthMask(GL_TRUE);*/
	
		
		
		}
	
	}
	
void Voxelization::draw_center_mass(cgv::render::context& ctx,vec3 center_m) {
	
		
		vec3 sphere_center = center_m;
		std::vector<vec4> sphere;
		std::vector<rgb> color;
		float Radius_SelectMode = 0.05;
		sphere.push_back(vec4(sphere_center, Radius_SelectMode));
		color.push_back(rgb(0, 1, 0));
		cgv::render::sphere_renderer& sr = ref_sphere_renderer(ctx);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		//sr.set_render_style(sphere_style);
		sr.set_color_array(ctx, color);
		sr.set_sphere_array(ctx, sphere);
		sr.render(ctx, 0, 1);
		sphere.clear();
		color.clear();
		glDisable(GL_BLEND);
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
		//deleteBuffers();
		
		
		createBuffers();
		bindbuffer();
		
		return true;
	}

	bool Voxelization::generate(cgv::render::context& ctx, std::vector<vec3> cam_pos) 
	{

		//init_voxelization(ctx);
		
		/*createBuffers();	
		bindbuffer();*/
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
		
		//GLuint zero = 0;
		//boxarray.setSubData(0, sizeof(GLuint), &zero);
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

		Object_Boundary.clear();
		Object_Boundary.resize(length);
		cubes.getSubData(0, sizeof(float) * length, static_cast<void*>(Object_Boundary.data()));
		render_content = Object_Boundary;
		deleteBuffers();
		return true;
	}

	void Voxelization::bindbuffer()
	{

		object_boundary.setBindingPoint(OBJBOUNDARY_SSB_BP);
		filled_object.setBindingPoint(OBJINSIDE_SSB_BP);
		denoised_object.setBindingPoint(DENOISEDOBJ_SSB_BP);
		cubes.setBindingPoint(CUBES_SSB_BP);
		//boxarray.setBindingPoint(BOXARRAY_SSB_BP);
		
	}

	void Voxelization::createBuffers() 
	{
		int length = Voxel_size[0] * Voxel_size[1] * Voxel_size[2];
		object_boundary = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, length * sizeof(float));
		object_boundary.setSubData(0, sizeof(float)* Object_Boundary.size(),  Object_Boundary.data());
		filled_object = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, length * sizeof(float));
		denoised_object = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, length * sizeof(float));
		cubes = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, length * sizeof(float));
		//boxarray = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, length *6* sizeof(float)+ sizeof(GLuint));
		//center_mass = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, length * 6 * sizeof(float));
	}
	void Voxelization::deleteBuffers()
	{
		object_boundary.deleteBuffer();
		filled_object.deleteBuffer();
		denoised_object.deleteBuffer();
		cubes.deleteBuffer();
		//boxarray.deleteBuffer();
	}

	void Voxelization::clear(cgv::render::context& ctx)
	{
		cgv::render::ref_rounded_cone_renderer(ctx, -1);
	}
