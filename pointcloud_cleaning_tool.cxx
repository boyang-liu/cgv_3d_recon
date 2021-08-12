#include <cgv/base/base.h>
#include "pointcloud_cleaning_tool.h"
#include <cgv_gl/gl/gl.h>
#include <cgv/gui/file_dialog.h>
#include <cgv/math/ftransform.h>
#include <cgv/math/constants.h>

#include <random>
#include <chrono>
#include <numeric>

#include <concurrency.h>
#include "util.h"

using namespace std;
using namespace cgv::base;
using namespace cgv::signal;
using namespace cgv::type;
using namespace cgv::gui;
using namespace cgv::data;
using namespace cgv::utils;
using namespace cgv::render;
using namespace cgv::pointcloud;

namespace {
	static cgv::pointcloud::utility::WorkerPool pool(std::thread::hardware_concurrency() - 1);
	
	//glCheckError from https://learnopengl.com/In-Practice/Debugging
	GLenum glCheckError_(const char* file, int line)
	{
		GLenum errorCode;
		while ((errorCode = glGetError()) != GL_NO_ERROR)
		{
			std::string error;
			switch (errorCode)
			{
			case GL_INVALID_ENUM:                  error = "INVALID_ENUM"; break;
			case GL_INVALID_VALUE:                 error = "INVALID_VALUE"; break;
			case GL_INVALID_OPERATION:             error = "INVALID_OPERATION"; break;
			case GL_STACK_OVERFLOW:                error = "STACK_OVERFLOW"; break;
			case GL_STACK_UNDERFLOW:               error = "STACK_UNDERFLOW"; break;
			case GL_OUT_OF_MEMORY:                 error = "OUT_OF_MEMORY"; break;
			case GL_INVALID_FRAMEBUFFER_OPERATION: error = "INVALID_FRAMEBUFFER_OPERATION"; break;
			}
			std::cout << error << " | " << file << " (" << line << ")" << std::endl;
		}
		return errorCode;
	}
	#define glCheckError() glCheckError_(__FILE__, __LINE__)

	//prints errors in debug builds if shader code is wrong
	void shaderCheckError(cgv::render::shader_program& prog, const char name[]) {
#ifndef NDEBUG
		if (prog.last_error.size() > 0) {
			std::cerr << "error in " << name << "\n" << prog.last_error << '\n';
			prog.last_error = "";
		}
#endif // #ifdef NDEBUG
	}
}
///
pointcloud_cleaning_tool::pointcloud_cleaning_tool() {
	set_name("pointcloud_cleaning_tool");

	source_prs.measure_point_size_in_pixel = false;
	source_prs.point_size = 1.00f;
	source_prs.blend_width_in_pixel = 1.0f;
	source_prs.blend_points = true;

	source_srs.measure_point_size_in_pixel = false;
	source_srs.point_size = 1.00f;
	source_srs.blend_width_in_pixel = 1.0f;
	source_srs.blend_points = true;
	source_srs.illumination_mode = cgv::render::IM_TWO_SIDED;

	source_pc.ref_render_style().draw_circles = true;

	rot_intensity = 0.2f;
	trans_intensity = 0.1;
	rcrs.radius = 0.001f;
	srs.radius = 0.12f;
	ars.length_scale = 0.05f;
	show_environment = true;
	show_table = true;
	cube_length = 0.05f;
	cube_rhand = box3(vec3(-cube_length, cube_length, cube_length), vec3(cube_length, cube_length, cube_length));
	//plane_length = 0.05f;
	plane_rhand = box3(vec3(-0.001f, -0.1f, -0.1f), vec3(0.001f, 0.1f, 0.1f));

	(InteractionMode)interaction_mode == InteractionMode::TELEPORT;

	build_scene(5, 7, 3, 0.2f, 1.6f, 0.8f, table_height, 0.03f);

	//reflected members
	rebuild_ptrs.insert(&put_on_table);
	rebuild_ptrs.insert(&pointcloud_fit_table);
	radius_for_test_labeling = 1.4;

	color_based_on_lod = false;
	enable_label_history = true;
	all_chunks_are_considered = false;
	
	pointcloud_fit_table = true;
	put_on_table = false;

	// model positioning 
	addi_model_transform.identity();
	concat_mat.identity();
	recompute_centroid = true;
	is_scaling = false;
	is_rotating_moving = false;
	is_tp_floor = false;

	for (int i =0;i<num_culling_protection_zones;++i){
		point_selection_center[i] = vec3(0.f);
		//point_selection_radii[i] = 0.05f;
		point_selection_group_mask[i] = (int)point_label::GROUP_MASK;
	}
	//create label history with a fixed size of (2^24)-1 points
	label_history = history(24);

	point_selection_colors[0] = rgba(0.09, 1.0, 0.2, 1.0);
	point_selection_colors[1] = rgba(0.09, 0.7, 1.0, 1.0);

	initial_offset_rhand = vec3(0, 0, -0.1f);
	curr_offset_rhand = initial_offset_rhand;


	/*procedure color generation*/
	//for (int i = 0; i < point_cloud_kit->pc.num_of_palette_spheres_rendered; i++) {
	//	rgba tmpcol = rgba(
	//		0.9f * distribution(generator) + 0.1f,
	//		0.9f * distribution(generator) + 0.1f,
	//		0.9f * distribution(generator) + 0.1f,
	//		1.0f
	//	);
	//	face_id_to_color.push_back(tmpcol);
	//}
	//// print them out 
	//if (true) {
	//	int i = 0;
	//	for (auto c: face_id_to_color) {
	//		std::cout << "face_id_to_color.push_back(rgba(" 
	//			<< c[0] << "," << c[1] <<  ","  << c[2] << "," << c.alpha() << "));" << std::endl;
	//		i++;
	//		if (i == 20)std::cout << "//" << std::endl;
	//	}
	//}
	
	// hard written to optimize cpu-gpu bandwidth, the same version saved in shader 
	PALETTE_COLOR_MAPPING.push_back(rgba(0.885186, 0.349231, 0.384895, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.187419, 0.234203, 0.141554, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.839713, 0.841112, 0.994662, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.38539, 0.212664, 0.725346, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.54153, 0.9552, 0.787375, 1));

	PALETTE_COLOR_MAPPING.push_back(rgba(0.49487, 0.697245, 0.131001, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.289188, 0.443403, 0.213307, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.81568, 0.146095, 0.788965, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.467858, 0.268185, 0.132797, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.501028, 0.51219, 0.540788, 1));

	PALETTE_COLOR_MAPPING.push_back(rgba(0.814578, 0.681682, 0.538812, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.779218, 0.928787, 0.738428, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.735197, 0.348423, 0.826778, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.689588, 0.102537, 0.711732, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.679565, 0.246351, 0.739634, 1));

	PALETTE_COLOR_MAPPING.push_back(rgba(0.548528, 0.51043, 0.207098, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.616379, 0.96377, 0.796525, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.626741, 0.889082, 0.406347, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.115997, 0.301431, 0.827358, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.329586, 0.839121, 0.77614, 1));

	PALETTE_COLOR_MAPPING.push_back(rgba(0.946067, 0.555361, 0.838757, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.901813, 0.4714, 0.729169, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.622861, 0.963362, 0.480849, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.224762, 0.242252, 0.592494, 1));
	PALETTE_COLOR_MAPPING.push_back(rgba(0.30714, 0.234365, 0.785558, 1));

	// 25 positions for labels 
	for (int iz = 5; iz > 0; iz--) {
		for (int ix = -2; ix < 3; ix++) {
			palette_lefthand_object_positions.push_back(vec3(ix * 0.05, 0.1, -iz * 0.05));
			palette_lefthand_palette_initialpose_positions.push_back(vec3(ix * 0.05, 0.1, -iz * 0.05));
		}
	}

	// 
	for (int i = 0; i < palette_lefthand_object_positions.size(); i++)
		palette_lefthand_object_colors.push_back(PALETTE_COLOR_MAPPING[i]);

	picked_sphere_index = 0;
	picked_label = (uint32_t)point_label::DELETED;
	point_selection_shape = selection_shape::SS_SPHERE;

	sphere_style_rhand.radius = 0.02f;
	sphere_style_lhand.radius = 0.02f;
	radius_adjust_step = 0.001f;
}
///
namespace {
	template<typename Point>
	void generate_lods_poisson(std::vector<Point>& input_buffer_data)
	{
		static constexpr int mean = 8;
		bool run_parralel = (input_buffer_data.size() > 10'000);

		if (run_parralel) {

			struct Task {
				Point* start;
				int num_points;
			};
			struct Tasks {
				std::atomic_int next_task = 0;
				std::vector<Task> task;
			} tasks;

			int64_t points_distributed = 0;
			int64_t points_total = input_buffer_data.size();
			constexpr int64_t batch_size = 500000;

			while (points_distributed < points_total) {
				int64_t batch = std::min(batch_size, points_total - points_distributed);
				tasks.task.push_back({ &input_buffer_data[points_distributed],(int)batch });
				points_distributed += batch;
			}

			pool.run([&input_buffer_data ,&tasks](int thread_id) {
				std::poisson_distribution<int> dist(mean);
				std::random_device rdev;

				while (true) {
					//fetch task
					int tid = tasks.next_task.fetch_add(1, std::memory_order_relaxed);
					if (tid < tasks.task.size()) {
						Task& task = tasks.task[tid];

						Point* end = task.start + task.num_points;
						for (Point* p = task.start; p < end; p++) {
							p->level() = std::min(2 * mean, std::max(0, mean - abs(dist(rdev) - mean)));
						}
					}
					else {
						return;
					}
				}
				});
		}
		else {
			std::poisson_distribution<int> dist(8);
			std::random_device rdev;

			for (auto& v : input_buffer_data) {
				v.level() = std::min(2 * mean, std::max(0, mean - abs(dist(rdev) - mean)));
			}
		}
	}
}
///
bool pointcloud_cleaning_tool::self_reflect(cgv::reflect::reflection_handler & rh)
{
	return
		rh.reflect_member("pointcloud_fit_table", pointcloud_fit_table) &&
		rh.reflect_member("max_points", max_points) &&
		rh.reflect_member("point_color_based_on_lod", color_based_on_lod) &&
		rh.reflect_member("model_scale", source_pc.ref_point_cloud_scale()) &&
		rh.reflect_member("model_position_x", source_pc.ref_point_cloud_position().x()) &&
		rh.reflect_member("model_position_y", source_pc.ref_point_cloud_position().y()) &&
		rh.reflect_member("model_position_z", source_pc.ref_point_cloud_position().z()) &&
		rh.reflect_member("model_put_on_table", put_on_table) &&
		rh.reflect_member("model_rotation_x", source_pc.ref_point_cloud_rotation().x()) &&
		rh.reflect_member("model_rotation_y", source_pc.ref_point_cloud_rotation().y()) &&
		rh.reflect_member("model_rotation_z", source_pc.ref_point_cloud_rotation().z()) &&
		//rh.reflect_member("selected_points_color", selected_points_color) &&
		rh.reflect_member("show_environment", show_environment) &&
		rh.reflect_member("show_table", show_table) &&
		rh.reflect_member("clod_point_style", source_pc.ref_render_style()) &&
		rh.reflect_member("chunk_size", chunk_cube_size) &&
		rh.reflect_member("use_chunks", use_chunks);
}
///
void pointcloud_cleaning_tool::on_set(void * member_ptr)
{
	if (rebuild_ptrs.find(member_ptr) != rebuild_ptrs.end()) {
		renderer_out_of_date = true;
	}
	else if (member_ptr == &show_environment){
		clear_scene();
		build_scene(5, 7, 3, 0.2f, 1.6f, 0.8f, table_height, 0.03f);
	}
	else if (member_ptr == &show_table) {
		clear_scene();
		build_scene(5, 7, 3, 0.2f, 1.6f, 0.8f, table_height, 0.03f);
	}
	else if (member_ptr == &color_based_on_lod) {
		recolor_point_cloud = true;
	}
	update_member(member_ptr);
}
///
void pointcloud_cleaning_tool::on_register()
{
}
///
void pointcloud_cleaning_tool::unregister()
{
}
///
bool pointcloud_cleaning_tool::init(cgv::render::context & ctx)
{
	cgv::gui::connect_vr_server(true);

	ctx.set_bg_clr_idx(3);
	ctx.set_bg_color(0, 0, 0, 0.9);
	cgv::render::view* view_ptr = find_view_as_node();
	
	if (view_ptr) {
		view_ptr->set_view_up_dir(vec3(0, 1, 0));
		view_ptr->set_focus(vec3(0, 0, 0));
		//view_ptr->set_eye_keep_view_angle(dvec3(0, 4, -4));
		// if the view points to a vr_view_interactor
		vr_view_ptr = dynamic_cast<vr_view_interactor*>(view_ptr);
		if (vr_view_ptr) {
			// configure vr event processing
			vr_view_ptr->set_event_type_flags(
				cgv::gui::VREventTypeFlags(
					cgv::gui::VRE_KEY +
					cgv::gui::VRE_ONE_AXIS +
					cgv::gui::VRE_TWO_AXES +
					cgv::gui::VRE_TWO_AXES_GENERATES_DPAD +
					cgv::gui::VRE_POSE
				));
			vr_view_ptr->enable_vr_event_debugging(false);
			// configure vr rendering
			vr_view_ptr->draw_action_zone(false);
			vr_view_ptr->draw_vr_kits(true);
			vr_view_ptr->enable_blit_vr_views(true);
			vr_view_ptr->set_blit_vr_view_width(200);
		}
	}
	
	cgv::render::ref_surfel_renderer(ctx, 1);
	cgv::render::ref_rounded_cone_renderer(ctx, 1);
	cgv::render::ref_box_renderer(ctx,1);
	cgv::render::ref_clod_point_renderer(ctx, 1);
	cgv::render::ref_sphere_renderer(ctx, 1);
	cgv::render::ref_arrow_renderer(ctx, 1);
	cgv::render::ref_box_wire_renderer(ctx, 1);

	ctx.set_bg_color(0.7, 0.7, 0.8, 1.0);

	//build custom shader prog
	if (!labeled_points_ss_draw_prog.is_created()) {
		labeled_points_ss_draw_prog.build_program(ctx, "clod_point_labels_ss.glpr",true);
		labeled_points_sp_draw_prog.build_program(ctx, "clod_point_labels_sp.glpr", true);
		labeled_points_sc_draw_prog.build_program(ctx, "clod_point_labels_sc.glpr", true);
		labeled_points_sn_draw_prog.build_program(ctx, "clod_point_labels_no_selection.glpr", true);

		//labeled_points_ss_draw_prog.build_program(ctx, "clod_point_labels.glpr", true);
		//add lookup table for label colors
		//TODO use a uniform buffer
		labeled_points_ss_draw_prog.set_uniform_array(ctx, "label_color_map", PALETTE_COLOR_MAPPING);
		labeled_points_sp_draw_prog.set_uniform_array(ctx, "label_color_map", PALETTE_COLOR_MAPPING);
		labeled_points_sc_draw_prog.set_uniform_array(ctx, "label_color_map", PALETTE_COLOR_MAPPING);
		labeled_points_sn_draw_prog.set_uniform_array(ctx, "label_color_map", PALETTE_COLOR_MAPPING);
	}

	if (!labeling_tool_prog.is_created()) {
		labeling_tool_prog.create(ctx);
		labeling_tool_prog.attach_file(ctx, "point_labeler_tool.glcs", cgv::render::ST_COMPUTE);
		shaderCheckError(labeling_tool_prog, "point_labeler_tool.glcs");
		labeling_tool_prog.attach_file(ctx, "point_label.glsl", cgv::render::ST_COMPUTE);
		labeling_tool_prog.attach_file(ctx, "point_label_history.glsl", cgv::render::ST_COMPUTE);
		labeling_tool_prog.link(ctx);
	}
	if (!selection_relabel_prog.is_created()) {
		selection_relabel_prog.create(ctx);
		selection_relabel_prog.attach_file(ctx, "selection_labeler.glcs", cgv::render::ST_COMPUTE);
		shaderCheckError(selection_relabel_prog, "selection_labeler.glcs");
		selection_relabel_prog.attach_file(ctx, "point_label.glsl", cgv::render::ST_COMPUTE);
		shaderCheckError(selection_relabel_prog, "point_label.glsl");
		selection_relabel_prog.attach_file(ctx, "point_label_history.glsl", cgv::render::ST_COMPUTE);
		shaderCheckError(selection_relabel_prog, "point_label_history.glsl");
		selection_relabel_prog.link(ctx,true);
	}
	//create shader prog
	if (!label_rollback_prog.is_created()) {
		label_rollback_prog.create(ctx);
		label_rollback_prog.attach_file(ctx, "point_label_rollback.glcs", cgv::render::ST_COMPUTE);
		shaderCheckError(label_rollback_prog, "point_label_rollback.glcs");
		label_rollback_prog.attach_file(ctx, "point_label.glsl", cgv::render::ST_COMPUTE);
		label_rollback_prog.attach_file(ctx, "point_label_history.glsl", cgv::render::ST_COMPUTE);
		label_rollback_prog.link(ctx, true);
		label_history.set_rollback_shader_prog(&label_rollback_prog);
	}

	glGenBuffers(1, &point_label_buffer);
	label_history.init(ctx);
	return true;
}

std::chrono::duration<double> diff3;

///
void pointcloud_cleaning_tool::init_frame(cgv::render::context& ctx)
{
	if (init_label_buffer && point_labels.size() > 0) {
		assert(point_label_buffer != 0);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, point_label_buffer);
		glBufferData(GL_SHADER_STORAGE_BUFFER, point_labels.size() * sizeof(GLint), point_labels.data(), GL_STATIC_READ);
		//glBufferStorage(GL_SHADER_STORAGE_BUFFER, point_labels.size() * sizeof(GLint), 
		//	point_labels.data(), GL_MAP_READ_BIT| GL_MAP_PERSISTENT_BIT);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
		glCheckError();
		init_label_buffer = false;
	}

	bool process_queue = !queued_actions.empty();
	
	/* run queued actions which require an opengl context */
	auto start_draw3 = std::chrono::steady_clock::now();
	if (process_queue) {
		cgv::render::clod_point_renderer& cp_renderer = ref_clod_point_renderer(ctx);
		GLuint reduced_points = cp_renderer.get_reduced_points();
		GLuint point_indices = cp_renderer.get_index_buffer();
		GLuint num_points = cp_renderer.num_reduced_points();
		GLuint num_input_points = chunked_points.num_points();
		GLuint input_buffer = cp_renderer.get_input_buffer();

		/* apply the point labels */
		for (auto& action : queued_actions) {
			switch (action->get_type()) {
				case buffer_action::POINT_LABELING_TOOL:
				{
					point_labeling_tool_commit* pltc = action->get_as<point_labeling_tool_commit>();
					if (picked_sphere_index != 3 && picked_sphere_index != 2) {
						label_points_in_sphere(ctx, pltc->label, pltc->group_mask, pltc->position, pltc->radius, input_buffer);
						//label_points_in_sphere(ctx, pltc->label, pltc->group_mask, pltc->position, pltc->radius, reduced_points,
						//	point_indices, num_points);
					}
					else if (picked_sphere_index == 2)
					{
						label_points_by_clipping(ctx, pltc->label, pltc->group_mask, pltc->position, pltc->plane_ori, input_buffer);
					}
					else if (picked_sphere_index == 3)
					{
						label_points_in_cube(ctx, pltc->label, pltc->group_mask, pltc->position, pltc->max_ext, pltc->min_ext, input_buffer);
					}
					break;
				}
				case buffer_action::POINT_RELABELING:
				{
					point_relabel_commit* prc = action->get_as<point_relabel_commit>();
					label_selected_points(ctx, prc->new_label, prc->expected_label, prc->point_group_mask, input_buffer, num_input_points);
					break;
				}
				case buffer_action::ROLLBACK: {
					rollback_last_operation(ctx);
					break;
				}
			}
		}
		queued_actions.clear();
	}
	auto stop_draw3 = std::chrono::steady_clock::now();
	diff3 = stop_draw3 - start_draw3;
	
	chunk_render_message_ss.str("");
	chunk_render_message_ss.clear();
}
///
void pointcloud_cleaning_tool::draw(cgv::render::context & ctx)
{
	// some variables required later
	vec4 selection_center_in_view_space;
	vec4 selection_center_in_world_space;
	mat4 view_transform = ctx.get_modelview_matrix(); 	// save the view transfrom before its changed by set_modelview_matrix
	{
		selection_center_in_world_space = (point_selection_center[point_selection_hand] + curr_offset_rhand).lift();
		selection_center_in_view_space = view_transform * selection_center_in_world_space;
	}
	
	//the renderer is accessible over a reference to a singelton but new instances are also possible
	cgv::render::clod_point_renderer& cp_renderer = ref_clod_point_renderer(ctx);
	{
		cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
		renderer.set_render_style(style);
		renderer.set_box_array(ctx, boxes);
		renderer.set_color_array(ctx, box_colors);
		renderer.render(ctx, 0, boxes.size());

		renderer.set_box_array(ctx, floor_box);
		renderer.set_color_array(ctx, floor_box_clr);
		renderer.render(ctx, 0, floor_box.size());
	}
	cp_renderer.set_render_style(source_pc.ref_render_style());

	bool is_snd_eye = false;
	if (vr_view_ptr->get_rendered_eye() == 1)
		is_snd_eye = true;

	if (source_pc.get_nr_points() > 0) {

		cp_renderer.set_max_drawn_points(ctx, max_points);

		if (renderer_out_of_date || recolor_point_cloud){
			//cp_renderer.generate_lods((cgv::render::LoDMode)lod_mode);

			if (color_based_on_lod) {
				//std::vector<LODPoint> pnts = points_with_lod;
				std::vector<LODPoint> pnts = chunked_points.points;
				int num_points = pnts.size();
				int max_lod = 0;
				for (int i = 0; i < chunked_points.num_points(); ++i) {
					max_lod = std::max((int)pnts[i].level(),max_lod);
				}

				std::vector<rgb8> col_lut;
				for (int lod = 0; lod <= max_lod; ++lod) {
					cgv::media::color<float, cgv::media::HLS> col;
					col.L() = 0.5f;
					col.S() = 1.f;
					col.H() = min_level_hue + (max_level_hue - min_level_hue) * ((float)lod / (float)max_lod);
					col_lut.push_back(col);
				}
				for (int i = 0; i < num_points; ++i) {
					pnts[i].color() = col_lut[pnts[i].level()];
				}
				cp_renderer.set_points(ctx, pnts.data(), pnts.size());
				//cp_renderer.set_points(ctx,&pnts.data()->position(), &pnts.data()->color(), &pnts.data()->level(), pnts.size(), sizeof(LODPoint));
			}
			else {
				//cp_renderer.set_points(ctx, &points_with_lod.data()->position(), &points_with_lod.data()->color(), &points_with_lod.data()->level(), points_with_lod.size(), sizeof(LODPoint));
				cp_renderer.set_points(ctx, chunked_points.points.data(), chunked_points.num_points());
			}
			renderer_out_of_date = false;
			recolor_point_cloud = false;
		}

		// build model_transform in real time 
		model_transform = cgv::math::translate4(source_pc.ref_point_cloud_position())
			* cgv::math::rotate4<float>(source_pc.ref_point_cloud_rotation())
			* cgv::math::scale4(source_pc.ref_point_cloud_scale(), source_pc.ref_point_cloud_scale(), source_pc.ref_point_cloud_scale());
		// add additional positioning matrix 
		concat_mat = model_transform * addi_model_transform;
		ctx.push_modelview_matrix();
		ctx.set_modelview_matrix(ctx.get_modelview_matrix()* concat_mat);

		//choose a shader program based on selection mode
		shader_program* active_label_prog = nullptr;
		if (use_label_prog) {
			if ((InteractionMode)interaction_mode == InteractionMode::LABELING) {
				switch (point_selection_shape) {
				case SS_SPHERE: {
					vec4 selection_center_and_radius = selection_center_in_view_space;
					selection_center_and_radius.w() = sphere_style_rhand.radius;
					labeled_points_ss_draw_prog.set_uniform(ctx, "selection_enclosing_sphere", selection_center_and_radius, true);
					labeled_points_ss_draw_prog.set_uniform(ctx, "selection_color", point_selection_colors[1], true);
					active_label_prog = &labeled_points_ss_draw_prog;
					break;
				}
				case SS_PLANE:
				{
					vec3 normal(1.f, 0.f, 0.f);
					plane_ori_right.rotate(normal);
					vec4 plane = normal.lift(); plane.w() = 0;
					//transform to view space
					plane = view_transform * plane;
					labeled_points_sp_draw_prog.set_uniform(ctx, "selection_plane_normal", plane, true);
					labeled_points_sp_draw_prog.set_uniform(ctx, "selection_plane_origin", selection_center_in_view_space, true);
					labeled_points_sp_draw_prog.set_uniform(ctx, "selection_color", point_selection_colors[1], true);
					active_label_prog = &labeled_points_sp_draw_prog;
					break;
				}
				case SS_CUBOID: {
					mat3 view_transform33 = mat3(3, 3, view_transform.begin());
					quat view_rotation = quat(view_transform33);
					vec4 view_offset = view_transform.col(4);
					
					labeled_points_sc_draw_prog.set_uniform(ctx, "selection_box_rotation", cube_ori_right);
					labeled_points_sc_draw_prog.set_uniform(ctx, "aabb_min_p", cube_rhand.get_min_pnt());
					labeled_points_sc_draw_prog.set_uniform(ctx, "aabb_max_p", cube_rhand.get_max_pnt());
					labeled_points_sc_draw_prog.set_uniform(ctx, "selection_box_translation", selection_center_in_world_space, true);
					labeled_points_sc_draw_prog.set_uniform(ctx, "inv_view_transform", inv(view_transform), true);
					labeled_points_sc_draw_prog.set_uniform(ctx, "selection_color", point_selection_colors[1], true);
					active_label_prog = &labeled_points_sc_draw_prog;
					//active_label_prog = &labeled_points_sn_draw_prog;
					break;
				}
				default:
					active_label_prog = &labeled_points_sn_draw_prog;
				}
			}
			else { //no labeling
				active_label_prog = &labeled_points_sn_draw_prog;
			}
		}

		if (use_chunks) { //chunked rendering path
			std::chrono::duration<double> diff0;
			auto start_draw0 = std::chrono::steady_clock::now();
			static std::vector<uint32_t> visible_chunks;
			//extract frustum
				dmat4 transform = ctx.get_projection_matrix() * ctx.get_modelview_matrix();
				vec4 planes[6];
				vec4 p4 = transform.row(3);
				for (int i = 0; i < 3; ++i) {
					planes[(i << 1)] = p4 - transform.row(i);
					planes[(i << 1) + 1] = p4 + transform.row(i);
				}
				//find chunks within the frustum
				visible_chunks.clear();
				chunked_points.intersect_chunks(planes, visible_chunks);
				auto stop_draw0 = std::chrono::steady_clock::now();
				diff0 = stop_draw0 - start_draw0;
			//std::cout << "rendered chunks: (" << visible_chunks.size() << "//"<< chunked_points.get_filled_chunks_ids().size() << ")\n";

			//chunk_render_message_ss << "(" << visible_chunks.size() << "/" << chunked_points.num_filled_chunks() << ") ";

			//render chunk
			if (visible_chunks.size() > 0) {
				if (active_label_prog) {
					cp_renderer.set_prog(*active_label_prog);
				}

				if (cp_renderer.enable(ctx)) {
					std::chrono::duration<double> diff1;
			
						auto start_reduce = std::chrono::steady_clock::now();
						cp_renderer.reduce_chunks(ctx, chunked_points.start_indices.data(), chunked_points.chunk_sizes.data(), visible_chunks.data(), visible_chunks.size());
						auto stop_reduce = std::chrono::steady_clock::now();
						diff1 = stop_reduce - start_reduce;
						//int n = cp_renderer.num_reduced_points();
						//std::cout << "num of points: " << n << std::endl;
					
					glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 6, point_label_buffer);
					glCheckError();
					auto start_draw = std::chrono::steady_clock::now();
					cp_renderer.draw_points(ctx);
					glFinish();
					auto stop_draw = std::chrono::steady_clock::now();
					std::chrono::duration<double> diff2 = stop_draw - start_draw;
					/*if (diff3.count() > 0.001)
					std::cout << "Time (" << vr_view_ptr->get_rendered_eye() << ") label: " << diff3.count() << " cull: " << diff0.count() << " reduce: " << diff1.count() << " draw: " << diff2.count() 
						<< " sum: " << diff3.count() + diff0.count() + diff1.count() + 2*diff2.count() <<" s\n";*/
					/*if(diff3.count() > 0.001)
						std::cout << "Time (" << vr_view_ptr->get_rendered_eye() << ") label: " << diff3.count() << " s\n";*/
					/*if(vr_view_ptr->get_rendered_eye() == 0)
					{ 
					if (idiff < 100 )
					{
						diff0_ave += diff0;
						diff1_ave += diff1;
						diff2_ave += diff2;
						diff3_ave += diff3;
						++idiff;
					}
					else{
						diff0_ave = diff0_ave / 100;
						diff1_ave = diff1_ave / 100;
						diff2_ave = diff2_ave / 100 * 2;
						diff3_ave = diff3_ave / 100;
						idiff = 0;
						std::cout << "Time (" << vr_view_ptr->get_rendered_eye() << ") average label: " << diff3_ave.count() << " cull: " << diff0_ave.count() << " reduce: " << diff1_ave.count() << " draw: " << diff2_ave.count()
							<< " sum: " << diff3_ave.count() + diff0_ave.count() + diff1_ave.count() + 2 * diff2_ave.count() << " s\n";
					}
					}*/
					cp_renderer.disable(ctx);
				}

				if (draw_chunk_bounding_boxes) {
					std::vector<box3> chunk_bbs;
					std::vector<rgb> chunk_bbs_colors;
					for (auto& chunk : visible_chunks) {
						chunk_bbs.push_back(chunked_points.bounding_boxes[chunk]);
						chunk_bbs_colors.emplace_back(0.1, 0.1, 0.87);
					}

					box_wire_renderer& br = ref_box_wire_renderer(ctx);
					br.set_render_style(style);
					br.set_box_array(ctx, chunk_bbs);
					br.set_color_array(ctx, chunk_bbs_colors);
					br.render(ctx, 0, chunk_bbs.size());
				}
			}
		}
		else { //normal rendering path
			if (active_label_prog) {
				cp_renderer.set_prog(*active_label_prog);
			}

			if (cp_renderer.enable(ctx)) {
				//auto start_reduce = std::chrono::steady_clock::now();
					cp_renderer.reduce_points(ctx, 0, (size_t)chunked_points.num_points());
				/*GLint* labels = static_cast<GLint*>(glMapNamedBufferRange(
					point_label_buffer, 0, source_pc.get_nr_points()*sizeof(GLint), GL_MAP_READ_BIT));

				glCheckError();
				glUnmapNamedBuffer(point_label_buffer);*/

				//auto stop_reduce = std::chrono::steady_clock::now();

				glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 6, point_label_buffer);
				glCheckError();

				//auto start_draw = std::chrono::steady_clock::now();

				cp_renderer.draw_points(ctx);
				/*if (diff3.count() > 0.001)
					std::cout << "Time (" << vr_view_ptr->get_rendered_eye() << ") label: " << diff3.count() << " s\n";*/
				//auto stop_draw = std::chrono::steady_clock::now();
				//glCheckError();
				cp_renderer.disable(ctx);
				//glCheckError();
			}
		}
		ctx.pop_modelview_matrix();
	}

	// draw intersection points
	if (!intersection_points.empty()) {
		auto& sr = cgv::render::ref_sphere_renderer(ctx);
		sr.set_position_array(ctx, intersection_points);
		sr.set_color_array(ctx, intersection_colors);
		sr.set_render_style(srs);
		sr.render(ctx, 0, intersection_points.size());
	}

	// draw vr stuff
	if ((InteractionMode)interaction_mode == InteractionMode::TELEPORT) {
		if (vr_view_ptr && is_tp_floor) {
			std::array<vec3, 4> P;
			std::array<float, 4> R;
			std::array<rgb, 4> C;
			size_t array_index = 0;
			constexpr float ray_length = 1.2;
			const vr::vr_kit_state* state_ptr = vr_view_ptr->get_current_vr_state();
			if (state_ptr) {
				for (int ci = 0; ci < 1; ++ci) if (state_ptr->controller[ci].status == vr::VRS_TRACKED) {
					vec3 ray_origin, ray_direction;
					state_ptr->controller[ci].put_ray(&ray_origin(0), &ray_direction(0));
					P[array_index] = ray_origin;
					R[array_index] = 0.002f;
					P[array_index + 1] = ray_origin + ray_length * ray_direction;
					R[array_index + 1] = 0.003f;
					rgb c(float(1 - ci), 0.5f, float(ci));
					C[array_index] = c;
					C[array_index + 1] = c;
					array_index += 2;
				}
			}
			if (P.size() > 0) {
				auto& cr = cgv::render::ref_rounded_cone_renderer(ctx);
				cr.set_render_style(cone_style);
				cr.set_position_array(ctx, P.data(), array_index);
				cr.set_color_array(ctx, C.data(), array_index);
				cr.set_radius_array(ctx, R.data(), array_index);
				cr.render(ctx, 0, P.size());
			}
		}
	}
	// draw vr stuff
	if ((InteractionMode)interaction_mode == InteractionMode::TRANSFORMING) {
		if (vr_view_ptr && is_tp_floor) {
			std::array<vec3, 4> P;
			std::array<float, 4> R;
			std::array<rgb, 4> C;
			size_t array_index = 0;
			constexpr float ray_length = 0.2;
			const vr::vr_kit_state* state_ptr = vr_view_ptr->get_current_vr_state();
			if (state_ptr) {
				for (int ci = 0; ci < 2; ++ci) if (state_ptr->controller[ci].status == vr::VRS_TRACKED) {
					vec3 ray_origin, ray_direction;
					state_ptr->controller[ci].put_ray(&ray_origin(0), &ray_direction(0));
					P[array_index] = ray_origin;
					R[array_index] = 0.002f;
					P[array_index + 1] = ray_origin + ray_length * ray_direction;
					R[array_index + 1] = 0.003f;
					rgb c(float(1 - ci), 0.5f, float(ci));
					C[array_index] = c;
					C[array_index + 1] = c;
					array_index += 2;
				}
			}
			if (P.size() > 0) {
				auto& cr = cgv::render::ref_rounded_cone_renderer(ctx);
				cr.set_render_style(cone_style);
				cr.set_position_array(ctx, P.data(), array_index);
				cr.set_color_array(ctx, C.data(), array_index);
				cr.set_radius_array(ctx, R.data(), array_index);
				cr.render(ctx, 0, P.size());
			}
		}
	}
	render_a_handhold_arrow(ctx, rgb(0.4), 0.1f);
	if ((InteractionMode)interaction_mode == InteractionMode::LABELING) {
		render_palette_for_labeling_on_lhand(ctx);
		switch (point_selection_shape) {
		case SS_SPHERE:
			render_palette_sphere_on_rhand(ctx);
			break;
		case SS_PLANE:
			render_palette_plane_on_rhand(ctx);
			break;
		case SS_CUBOID:
			render_palette_cube_on_rhand(ctx);
			break;
		}
	}
}
///
void pointcloud_cleaning_tool::finish_draw(cgv::render::context& ctx) {
	chunk_render_message = chunk_render_message_ss.str();
	update_member(&chunk_render_message);
}
///
void pointcloud_cleaning_tool::render_a_handhold_arrow(cgv::render::context& ctx, rgb c, float r) {
	if (curr_offset_rhand.length() < 1e-6)
		return;

	//auto& prog = ctx.ref_surface_shader_program();
	//prog.set_uniform(ctx, "map_color_to_material", 3);
	//prog.enable(ctx);
	//ctx.set_color(c);
	//ctx.tesselate_arrow(point_selection_center[1], point_selection_center[1] + curr_offset_rhand, r, 2.0, 0.5f);
	//prog.disable(ctx);

	float previous_scale = ars.length_scale;
	ars.length_scale = 1.0f;
	cgv::render::arrow_renderer& a_renderer = ref_arrow_renderer(ctx);
	a_renderer.set_render_style(ars);
	a_renderer.set_position_array(ctx, &point_selection_center[1], 1);
	a_renderer.set_color_array(ctx, &rgb(0.4), 1);
	a_renderer.set_direction_array(ctx, &curr_offset_rhand, 1);
	a_renderer.render(ctx, 0, 1);
	ars.length_scale = previous_scale;
}
/// 
void pointcloud_cleaning_tool::render_palette_sphere_on_rhand(cgv::render::context& ctx) {
	glDisable(GL_CULL_FACE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	sphere_style_rhand.material.set_transparency(0.4);
	auto& sr = cgv::render::ref_sphere_renderer(ctx);
	sr.set_render_style(sphere_style_rhand);
	sr.set_position_array(ctx, &(point_selection_center[1] + curr_offset_rhand), 1);// picked_sphere_index changes in POSE event 
	if (picked_sphere_index >= 0 && picked_sphere_index < palette_lefthand_object_colors.size()) {
		//rgba tmpcol = palette_lefthand_object_colors[picked_sphere_index];
		//tmpcol[3] = 0.5;
		//sr.set_color_array(ctx, &rgba(0.4, 0.9, 0.4, 0.4), 1);
		sr.set_color_array(ctx, &palette_lefthand_object_colors[picked_sphere_index], 1);
	}
	else
		sr.set_color_array(ctx, &rgba(0.4), 1); // gray means out of range 
	sr.render(ctx, 0, 1);
	glDisable(GL_BLEND);
}
///
void pointcloud_cleaning_tool::render_palette_for_labeling_on_lhand(cgv::render::context& ctx) {
	auto& sr = cgv::render::ref_sphere_renderer(ctx);
	sr.set_render_style(sphere_style_lhand);
	sr.set_position_array(ctx, palette_lefthand_object_positions);
	sr.set_color_array(ctx, palette_lefthand_object_colors);
	sr.render(ctx, 0, palette_lefthand_object_positions.size());
}
///
void pointcloud_cleaning_tool::render_palette_plane_on_rhand(cgv::render::context& ctx) {
	glDisable(GL_CULL_FACE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	plane_style_rhand.material.set_transparency(0.4);
	auto& cuber = cgv::render::ref_box_renderer(ctx);
	//TODO adjust size of cube_rhand
	cuber.set_render_style(plane_style_rhand);
	cuber.set_box_array(ctx, &plane_right,1);
	cuber.set_rotation_array(ctx, &plane_ori_right,1);
	//cuber.set_position_array(ctx, &(point_selection_center[1] + curr_offset_rhand), 1);// picked_sphere_index changes in POSE event 
	if (picked_sphere_index >= 0 && picked_sphere_index < palette_lefthand_object_colors.size()) {
		//rgba tmpcol = palette_lefthand_object_colors[picked_sphere_index];
		//tmpcol[3] = 0.5;
		//sr.set_color_array(ctx, &rgba(0.4, 0.9, 0.4, 0.4), 1);
		cuber.set_color_array(ctx, &palette_lefthand_object_colors[picked_sphere_index], 1);
	}
	else
		cuber.set_color_array(ctx, &rgba(0.4), 1); // gray means out of range 
	cuber.render(ctx, 0, 1);
	glDisable(GL_BLEND);
}
///
void pointcloud_cleaning_tool::render_palette_cube_on_rhand(cgv::render::context& ctx) {
	glDisable(GL_CULL_FACE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	cube_style_rhand.material.set_transparency(0.4);
	auto& cuber = cgv::render::ref_box_renderer(ctx);
	//TODO adjust size of cube_rhand
	cuber.set_render_style(cube_style_rhand);
	cuber.set_box_array(ctx, &cube_rhand,1);
	vec3 cube_position = point_selection_center[1] + curr_offset_rhand;
	cuber.set_translation_array(ctx, &cube_position,1);
	cuber.set_rotation_array(ctx, &cube_ori_right,1);
	//cuber.set_position_array(ctx, &(point_selection_center[1] + curr_offset_rhand), 1);// picked_sphere_index changes in POSE event 
	if (picked_sphere_index >= 0 && picked_sphere_index < palette_lefthand_object_colors.size()) {
		//rgba tmpcol = palette_lefthand_object_colors[picked_sphere_index];
		//tmpcol[3] = 0.5;
		//sr.set_color_array(ctx, &rgba(0.4, 0.9, 0.4, 0.4), 1);
		cuber.set_color_array(ctx, &palette_lefthand_object_colors[picked_sphere_index], 1);
	}
	else
		cuber.set_color_array(ctx, &rgba(0.4), 1); // gray means out of range 
	cuber.render(ctx, 0, 1);
	glDisable(GL_BLEND);
}
///
void pointcloud_cleaning_tool::clear(cgv::render::context & ctx)
{
	labeled_points_ss_draw_prog.destruct(ctx);
	labeled_points_sn_draw_prog.destruct(ctx);
	labeled_points_sp_draw_prog.destruct(ctx);
	labeled_points_sc_draw_prog.destruct(ctx);
	selection_relabel_prog.destruct(ctx);
	labeling_tool_prog.destruct(ctx);
	glDeleteBuffers(1, &point_label_buffer);
	label_history.clear(ctx);

	cgv::render::ref_surfel_renderer(ctx, -1);
	cgv::render::ref_rounded_cone_renderer(ctx, -1);
	cgv::render::ref_box_renderer(ctx, -1);
	cgv::render::ref_clod_point_renderer(ctx, -1);
	cgv::render::ref_sphere_renderer(ctx, -1);
	cgv::render::ref_arrow_renderer(ctx, -1);
	cgv::render::ref_box_wire_renderer(ctx, -1);
}
///
bool pointcloud_cleaning_tool::handle(cgv::gui::event & e)
{
	if ((e.get_flags() & cgv::gui::EF_VR) == 0)
		return false;

	switch (e.get_kind()) {
		case cgv::gui::EID_POSE:
		{
			cgv::gui::vr_pose_event& vrpe = static_cast<cgv::gui::vr_pose_event&>(e);


			// check for controller pose events
			int ci = vrpe.get_trackable_index();

			if (ci < 0 || ci >= 2) {
				return true;
			}
			// store pose for later use
			controller_poses[ci] = vrpe.get_pose_matrix();

			if (ci != -1) {
				points.clear();
				colors.clear();
				normals.clear();
				pos = vrpe.get_position();
				ori = vrpe.get_orientation();
				points.push_back(pos);
				colors.push_back(rgb(0.0f, 1.0f, 0.0f));
				normals.push_back(ori * vec3(0.0f, 0.0f, -1.0f));
			}
			// keep track of the controllers positions for point selection
			point_selection_center[ci] = vrpe.get_position();
			point_selection_center_ori[ci] = vrpe.get_orientation();

			// update offset on right hand
			if (ci == 1) {
				vec3 off = initial_offset_rhand;
				vrpe.get_quaternion().rotate(off);
				curr_offset_rhand = off;
			}

			// update offset on left hand
			if (ci == 0) {
				if (palette_lefthand_object_positions.size() == palette_lefthand_palette_initialpose_positions.size()) 
					for (int i = 0; i < palette_lefthand_object_positions.size();i++) {
						vec3 off = palette_lefthand_palette_initialpose_positions[i];
						vrpe.get_quaternion().rotate(off);
						palette_lefthand_object_positions[i] = point_selection_center[0] + off;
					}
				if ((InteractionMode)interaction_mode == InteractionMode::TELEPORT)
				{
					// compute intersections
					intersection_points.clear();
					intersection_colors.clear();
					vec3 origin, direction;
					vrpe.get_state().controller[ci].put_ray(&origin(0), &direction(0));
					compute_intersections(origin, direction, ci, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
				}
			}

			if (is_scaling) {
				static constexpr float movement_to_scale_factor = 0.20;

				// find some controller parameters
				int other_controller = 1 ^ ci; //(ci == 0) ? 1 : 0;
				float dist_controllers = (controller_poses[0].col(3) - controller_poses[1].col(3)).length();
				float old_dist_controllers = (vrpe.get_last_pose_matrix().col(3) - controller_poses[other_controller].col(3)).length();

				//scale point cloud
				float old_scale = source_pc.ref_point_cloud_scale();
				float old_clod_scale = source_pc.ref_render_style().scale;
				source_pc.ref_point_cloud_scale() = std::max<float>(0.f, old_scale + (dist_controllers - old_dist_controllers) * movement_to_scale_factor);
				source_pc.ref_render_style().scale = std::max<float>(0.f, old_clod_scale + (dist_controllers - old_dist_controllers) * movement_to_scale_factor);
				source_pc.ref_render_style().CLOD = source_pc.ref_render_style().scale;
				update_member(&source_pc.ref_render_style().CLOD);
				update_member(&source_pc.ref_render_style().scale);
			}

			if (is_rotating_moving && ci == 1) {
				vec3 movement = controller_poses[1].col(3) - vrpe.get_last_pose_matrix().col(3);
				//calculate rotation delta
				quat to = quat(mat3(3, 3, controller_poses[1].begin()));
				quat from = quat(mat3(3, 3, vrpe.get_last_pose_matrix().begin()));
				quat delta_rotation = to*from.conj(); //delta_rotation = to*inv(from), conjugate of rotation quaternion == inverse of rotation quaternion
				
				mat4 concat_mat = model_transform * addi_model_transform;
				
				mat4 linear_transform = cgv::math::translate4(controller_poses[1].col(3)+ movement) * delta_rotation.get_homogeneous_matrix() * cgv::math::translate4(-controller_poses[1].col(3)) * concat_mat;
				
				mat3 rotation;
				for (int j = 0; j < 3; ++j) {
					for (int i = 0; i < 3; ++i) {
						rotation(i, j) = linear_transform(i, j);
					}
				}
				quat q_rotation = quat(rotation);
				vec4 point_cloud_translation = linear_transform.col(3);
				source_pc.ref_point_cloud_rotation() = cgv::math::rad2deg(to_euler_angels(q_rotation));
				source_pc.ref_point_cloud_position() = vec3(point_cloud_translation.x(), point_cloud_translation.y(), point_cloud_translation.z());
			}

			// pick a label 
			int nearest_palette_idx = -1;
			vec3 picking_position_rhand = point_selection_center[1] + curr_offset_rhand;
			float dist = std::numeric_limits<float>::max();
			for (int i = 0; i < palette_lefthand_object_positions.size(); i++) {
				float cur_dist = (palette_lefthand_object_positions[i] - picking_position_rhand).length();
				if (cur_dist < dist) {
					dist = cur_dist;
					nearest_palette_idx = i;
					if (dist < sphere_style_lhand.radius)
						break;
				}
			}
			if (dist < sphere_style_lhand.radius) {
				picked_sphere_index = nearest_palette_idx;

				//assigns a selection shape and a label based on picked_sphere_index (may need to change this in a future version)
				switch (picked_sphere_index) {
				case 0:
					picked_label = make_label(0, point_label::DELETED);
					break;
				default:
					point_selection_shape = selection_shape::SS_SPHERE;
					picked_label = make_label(picked_sphere_index, point_label::VISIBLE);
				}
				//std::cout << "currently picked index = " << picked_sphere_index << std::endl;
			}
			//pick shapes for rendering if required
			switch (point_selection_shape) {
			case selection_shape::SS_PLANE:
				// render a plane
				plane_rhand = box3(vec3(-0.001f, -1.f, -1.f), vec3(0.001f, 1.f, 1.f));
				plane_rhand.translate(point_selection_center[1] + curr_offset_rhand);
				plane_ori_right = point_selection_center_ori[1];
				plane_right = plane_rhand;
				break;
			case selection_shape::SS_CUBOID:
				// render a cube
				cube_rhand = box3(vec3(-cube_length, -cube_length, -cube_length), vec3(cube_length, cube_length, cube_length));
				//cube_rhand.translate(point_selection_center[1] + curr_offset_rhand);
				
				cube_ori_right = point_selection_center_ori[1];
				break;
			}
		}
		case cgv::gui::EID_KEY:
		{
			//read out some buttons that need to be holded down for somthing to happen.
			//The state may have changed without noticing the application, e.g. a controller got disconnected before releasing a button and the button relase event is never triggered.
			{
				const vr::vr_kit_state* state = vr_view_ptr->get_current_vr_state();
				if (state) {
					if (interaction_mode == (int)InteractionMode::TRANSFORMING) {
						//first controller allows scaling
						is_scaling = state->controller[0].button_flags & vr::VRF_GRIP;
						//second does movement and rotating
						is_rotating_moving = state->controller[1].button_flags & vr::VRF_GRIP;
							//may switch this mappings based on if the person is right or left handed
					}
					else {
						is_scaling = false;
						is_rotating_moving = false;
					}
				}
			}

			static const float angle = std::asin(1.f)/3.f;
			cgv::gui::vr_key_event& vrke = static_cast<cgv::gui::vr_key_event&>(e);

			if (vrke.get_action() == cgv::gui::KA_RELEASE) {
				return true;
			}
			switch (vrke.get_key()) {
			case vr::VR_MENU:
				if (vrke.get_controller_index() == 0)
				{
					schedule_a_rollback();
				}
				if (vrke.get_controller_index() == 1)
				{
					//all points with the SELECTED_BIT label get assigned the DELETED label
					schedule_a_labeling_of_selected_points((int)point_label::DELETED, (int)point_label::SELECTED_BIT, (int)point_label::GROUP_MASK);
				}
				break;
			case vr::VR_DPAD_LEFT:
				//all points with the SELECTED_BIT label get assigned the DELETED label
				//schedule_a_labeling_of_selected_points((int)point_label::DELETED, (int)point_label::SELECTED_BIT, (int)point_label::GROUP_MASK);
				if (vrke.get_controller_index() == 1) {
					if ((InteractionMode)interaction_mode == InteractionMode::CONFIG) {
						if (source_pc.get_nr_points() > 0)
						{
							source_pc.ref_render_style().spacing -= 0.01;
							update_member(&source_pc.ref_render_style().spacing);
						}
					}

					if ((InteractionMode)interaction_mode == InteractionMode::LABELING) {
						point_selection_shape = (selection_shape) ((point_selection_shape - 1) % selection_shape::NUM_OF_SHAPES);
					}
				}
				break;
			case vr::VR_DPAD_RIGHT:
				//schedule_a_rollback();
				if (vrke.get_controller_index() == 1) {
					if ((InteractionMode)interaction_mode == InteractionMode::CONFIG) {
						if (source_pc.get_nr_points() > 0)
						{
							source_pc.ref_render_style().spacing += 0.01;
							update_member(&source_pc.ref_render_style().spacing);
						}
					}
					if ((InteractionMode)interaction_mode == InteractionMode::LABELING) {
						point_selection_shape = (selection_shape)((point_selection_shape + 1) % selection_shape::NUM_OF_SHAPES);
					}
				}
				break;
			case vr::VR_DPAD_UP:
				//switch interaction mode
				if (vrke.get_controller_index() == 0) {
					interaction_mode = (interaction_mode + 1) % (int)(NUM_OF_INTERACTIONS);
					std::cout << "interaction mode: " << interaction_mode;
				}

				if (vrke.get_controller_index() == 1) {
					
					c_pos = vr_view_ptr->get_tracking_origin();
					//std::cout << "pos: " << c_pos << std::endl;

					/*forward_points.push_back(c_pos);
					forward_colors.push_back(rgb(0.0, 0.0, 1.0));
					forward_normals.push_back(vec3(0.0f, 0.0f, pos.z()) - c_pos);*/
					p.position = ori * vec3(0.0f, 0.0f, -1.0f) * 0.1f + c_pos;
					p.color = rgb(1.0, 0.0, 0.0);
					p.radius = 0.15f;
					vr_view_ptr->set_tracking_origin(p.position);
					coordinate_c.push_back(p);
					if ((InteractionMode)interaction_mode == InteractionMode::LABELING) {
						sphere_style_rhand.radius += radius_adjust_step;
					}
					if ((InteractionMode)interaction_mode == InteractionMode::TRANSFORMING) {
						// add some action for dpad up here
					}
					if ((InteractionMode)interaction_mode == InteractionMode::CONFIG) {
						/*if (source_pc.get_nr_points() > 0)
						{
							source_pc.ref_render_style().pointSize += 0.1;
							update_member(&source_pc.ref_render_style().pointSize);
						}	*/
					}

				}
				break;
			case vr::VR_DPAD_DOWN:
				//switch interaction mode
				if (vrke.get_controller_index() == 0) {
					interaction_mode = (interaction_mode-1) % (int)(NUM_OF_INTERACTIONS);
					std::cout << "interaction mode:" << interaction_mode;
				}

				if (vrke.get_controller_index() == 1) {
					c_pos = vr_view_ptr->get_tracking_origin();
					std::cout << "pos: " << c_pos << std::endl;

					/*forward_points.push_back(c_pos);
					forward_colors.push_back(rgb(0.0, 0.0, 1.0));
					forward_normals.push_back(vec3(0.0f, 0.0f, pos.z()) - c_pos);*/
					p.position = ori * vec3(0.0f, 0.0f, 1.0f) * 0.1f + c_pos;
					p.color = rgb(1.0, 0.0, 0.0);
					p.radius = 0.15f;
					vr_view_ptr->set_tracking_origin(p.position);
					coordinate_c.push_back(p);					
					if ((InteractionMode)interaction_mode == InteractionMode::LABELING) {
						if (point_selection_shape != SS_PLANE)
						{
							if (sphere_style_rhand.radius > radius_adjust_step)
								sphere_style_rhand.radius -= radius_adjust_step;
						}
					}
					else if ((InteractionMode)interaction_mode == InteractionMode::TRANSFORMING) {
						// add some action for dpad down here
					}
					else if ((InteractionMode)interaction_mode == InteractionMode::CONFIG) {
						/*std::cout << "config mode" << std::endl;
						if (source_pc.get_nr_points() > 0)
						{
							source_pc.ref_render_style().pointSize -= 0.1;
							update_member(&source_pc.ref_render_style().pointSize);
						}*/
					}
				}

				break;
			case vr::VR_GRIP: 
				{
					if ((InteractionMode)interaction_mode == InteractionMode::LABELING) {
						switch (point_selection_shape) {
						case SS_SPHERE:
							schedule_a_labeling_of_points((int)point_label::SELECTED_BIT, point_selection_center[1] + curr_offset_rhand,
								sphere_style_rhand.radius, point_selection_group_mask[1]);
							break;
						case SS_CUBOID:
							schedule_a_labeling_of_points_in_cube((int)point_label::SELECTED_BIT, point_selection_center[1] + curr_offset_rhand,
								cube_rhand.get_max_pnt(), cube_rhand.get_min_pnt(), point_selection_group_mask[1]);
							break;
						case SS_PLANE:
							schedule_a_labeling_of_points_by_clipping_plane((int)point_label::SELECTED_BIT, point_selection_center[1] + curr_offset_rhand,
								point_selection_center_ori[1], point_selection_group_mask[1]);
							break;
						}
					}
					if (vrke.get_controller_index() == 0) {
						if ((InteractionMode)interaction_mode == InteractionMode::TELEPORT) {
							is_tp_floor = true;
							if (intersection_points.empty())
								break;
							//std::cout << "p.position: " << intersection_points.at(0) << std::endl;
							p.position = intersection_points.at(0);
							p.color = rgb(1.0, 0.0, 0.0);
							p.radius = 0.15f;
							p.position = p.position + vec3(-0.4f, 0.f, -1.1f);
							//c_pos = vr_view_ptr->get_tracking_origin();
							//std::cout << "c_pos: " << c_pos << std::endl;
							forward_points.push_back(c_pos);
							forward_colors.push_back(rgb(0.0, 0.0, 1.0));
							forward_normals.push_back(vec3(0.0f, 0.0f, pos.z()) - c_pos);
							vr_view_ptr->set_tracking_origin(p.position);
							coordinate_c.push_back(p);
						}
					}
					break;
				}
			}
			break;
		}
	}

	if (e.get_kind() == cgv::gui::EID_STICK) {
		cgv::gui::vr_stick_event& vrse = static_cast<cgv::gui::vr_stick_event&>(e);
		if (vrse.get_controller_index() == 0)
		{
			if (vrse.get_action() == cgv::gui::SA_MOVE) {
				//clamp to -1.0, 1.0
				static constexpr float threshold = 0.25f;
				//zero if sub threashold otherwise sign(vrse.get_y())*1.f
				float factor = abs(vrse.get_y()) > threshold ? (cgv::math::sign(vrse.get_y()) >= 0 ? 1.f : -1.f) : 0;

				if ((InteractionMode)interaction_mode == InteractionMode::LABELING) {
					if (factor != 0) {
						switch (point_selection_shape) {
						case SS_SPHERE:
							sphere_style_rhand.radius += factor * radius_adjust_step;
							sphere_style_rhand.radius = std::max(0.f, sphere_style_rhand.radius);
							break;
						case SS_CUBOID:
							cube_length += factor*radius_adjust_step;
							cube_length = std::max(0.f, cube_length);
							cube_rhand = box3(vec3(-cube_length, -cube_length, -cube_length), vec3(cube_length, cube_length, cube_length));
							break;
						case SS_PLANE:
							//nothing to do here
							break;
						}
					}
				}
				
				if ((InteractionMode)interaction_mode == InteractionMode::CONFIG) {
					if (source_pc.get_nr_points() > 0)
					{
						if (vrse.get_x() > 0 && vrse.get_y() < 0.5 && vrse.get_y() > -0.5)
						{
							source_pc.ref_render_style().CLOD += 0.01;
							update_member(&source_pc.ref_render_style().CLOD);
						}
						else if (vrse.get_x() < 0 && vrse.get_y() < 0.5 && vrse.get_y() > -0.5){
							source_pc.ref_render_style().CLOD -= 0.01;
							update_member(&source_pc.ref_render_style().CLOD);
						}
					}
				}
			}
		}
		if (vrse.get_controller_index() == 1)
		{
			if (vrse.get_action() == cgv::gui::SA_MOVE) {
				if ((InteractionMode)interaction_mode == InteractionMode::CONFIG) {
					if (source_pc.get_nr_points() > 0)
					{
						if (vrse.get_x() > 0 && vrse.get_y() < 0.5 && vrse.get_y() > -0.5)
						{
							source_pc.ref_render_style().pointSize += 0.01;
							update_member(&source_pc.ref_render_style().pointSize);
						}
						else if(vrse.get_x() < 0 && vrse.get_y() < 0.5 && vrse.get_y() > -0.5){
							source_pc.ref_render_style().pointSize -= 0.01;
							update_member(&source_pc.ref_render_style().pointSize);
						}
					}
				}
			}
		}
	}

	if (e.get_kind() == cgv::gui::EID_THROTTLE) {
		auto& te = static_cast<cgv::gui::vr_throttle_event&>(e);
		float v = te.get_value();
		bool d = (v == 1); // event 
		if (1 == te.get_controller_index()) { // right hand 
			if (v > 0) {
				if ((InteractionMode)interaction_mode == InteractionMode::LABELING) {
					schedule_a_labeling_of_points(picked_label, point_selection_center[1] + curr_offset_rhand,
						sphere_style_rhand.radius, point_selection_group_mask[1]);
				}
			}
		}
	}
	return false;
}
/// schedule a labeling of points by adding a point_labeling action to the queue
void pointcloud_cleaning_tool::schedule_a_labeling_of_points(int32_t label, vec3 position, float radius, int32_t group_mask) {
	auto action = std::make_unique<point_labeling_tool_commit>();
	action->label = label;
	action->position = position;
	action->radius = radius;
	action->group_mask = group_mask;
	queued_actions.push_back(std::move(action));
}
/// schedule a labeling of points by adding a point_labeling action to the queue
void pointcloud_cleaning_tool::schedule_a_labeling_of_points_in_cube(int32_t label, vec3 position, vec3 max_ext, vec3 min_ext, int32_t group_mask) {
	auto action = std::make_unique<point_labeling_tool_commit>();
	action->label = label;
	action->position = position;
	action->max_ext = max_ext;
	action->min_ext = min_ext;
	action->group_mask = group_mask;
	queued_actions.push_back(std::move(action));
}
/// schedule a labeling of points by adding a point_labeling action to the queue
void pointcloud_cleaning_tool::schedule_a_labeling_of_points_by_clipping_plane(int32_t label, vec3 position, mat3 plane_ori, int32_t group_mask) {
	auto action = std::make_unique<point_labeling_tool_commit>();
	action->label = label;
	action->position = position;
	action->plane_ori = plane_ori;
	action->group_mask = group_mask;
	queued_actions.push_back(std::move(action));
}
///
void  pointcloud_cleaning_tool::schedule_a_labeling_of_selected_points(int32_t new_label, int32_t expected_label, int32_t group_mask) {
	auto action = std::make_unique<point_relabel_commit>();
	action->new_label = new_label;
	action->expected_label = expected_label;
	action->point_group_mask = group_mask;
	queued_actions.push_back(std::move(action));
}
/// a quick test that enables to label the point cloud without a VR device 
void pointcloud_cleaning_tool::test_labeling_of_points() {
	auto action = std::make_unique<point_labeling_tool_commit>();
	action->label = (int)point_label::SELECTED_BIT;
	action->position = vec3(0);
	action->radius = radius_for_test_labeling;
	action->group_mask = (int)point_label::GROUP_MASK;
	queued_actions.push_back(std::move(action));
}
///
void pointcloud_cleaning_tool::schedule_a_rollback() {
	auto action = std::make_unique<rollback_commit>();
	queued_actions.push_back(std::move(action));
}
///
void pointcloud_cleaning_tool::stream_help(std::ostream & os)
{
}
///
void pointcloud_cleaning_tool::colorize_with_height() {
	vec3 pmin(std::numeric_limits<float>::infinity()), pmax(-std::numeric_limits<float>::infinity());
	for (int i = 0; i < chunked_points.num_points(); i++) {
		vec3 pnt = chunked_points.points.at(i).position();

		pmin.x() = std::min(pnt.x(), pmin.x());
		pmin.y() = std::min(pnt.y(), pmin.y());
		pmin.z() = std::min(pnt.z(), pmin.z());

		pmax.x() = std::max(pnt.x(), pmax.x());
		pmax.y() = std::max(pnt.y(), pmax.y());
		pmax.z() = std::max(pnt.z(), pmax.z());
	}

	vec3 ext = (pmax - pmin);
	int minimal_ext_idx = 0;
	if (ext.x() < ext.y() && ext.x() < ext.z())
		minimal_ext_idx = 0;
	if (ext.y() < ext.x() && ext.y() < ext.z())
		minimal_ext_idx = 1;
	if (ext.z() < ext.y() && ext.z() < ext.x())
		minimal_ext_idx = 2;

	for (int i = 0; i < chunked_points.num_points(); i++) {
		vec3 pnt = chunked_points.points.at(i).position();
		rgb8* ref_clr = &chunked_points.points.at(i).color();
		if (minimal_ext_idx == 0) { // colorize according to x 
			float factor = (pnt.x() - pmin.x()) / ext.x();
			*ref_clr = rgb8((int)255 * factor, 0, (int)255 * (1 - factor));
		}
		if (minimal_ext_idx == 1) { // colorize according to y 
			float factor = (pnt.y() - pmin.y()) / ext.y();
			*ref_clr = rgb8((int)255 * factor, 0, (int)255 * (1 - factor));
		}
		if (minimal_ext_idx == 2) { // colorize according to z 
			float factor = (pnt.z() - pmin.z()) / ext.z();
			*ref_clr = rgb8((int)255 * factor, 0, (int)255 * (1 - factor));
		}
	}

	renderer_out_of_date = true;
	post_redraw();
}
///
void pointcloud_cleaning_tool::print_point_cloud_info() {
	std::cout << "point loaded: " << source_pc.get_nr_points() << std::endl;
	std::cout << "chunks used: " << chunked_points.chunk_sizes.size() << std::endl;
}
/// fills points_with_lod with data generated from the source point cloud (source_pc)
void pointcloud_cleaning_tool::prepare_point_cloud() noexcept
{
	std::vector<indexed_point> source_points(source_pc.get_nr_points());
	//std::vector<LODPoint> source_points(source_pc.get_nr_points());
	rgb default_color(1.0, 0.0, 0.0);
	size_t num_source_points = source_pc.get_nr_points();

	//the octree_lod_generator expects the input points to be an array of structs, so we need to reshape the data
	/*std::chrono::duration<double> diff7;
	auto start_draw7 = std::chrono::steady_clock::now();*/
	for (int i = 0; i < num_source_points; ++i) {
		source_points[i].position() = source_pc.pnt(i);
		source_points[i].index = i;
		if (source_pc.has_colors()) {
			source_points[i].color() = source_pc.clr(i);
		}
		else {
			source_points[i].color() = rgb8(default_color);
		}
		if (source_pc.has_lods()) {
			
			source_points[i].level() = source_pc.lod(i);
			
		}
	}
	
	
	chunks<indexed_point> chunked_indexed_points;
	{
		std::vector<indexed_point> indexed_points_with_lod;
		
		
		if (!source_pc.has_lods()) {
			// have to (re-)compute level of details for each point 
			std::chrono::duration<double> diff5;
			auto start_draw5 = std::chrono::steady_clock::now();
			if ((LoDMode)lod_mode == LoDMode::OCTREE) {
				indexed_points_with_lod = std::move(lod_generator.generate_lods(source_points));
				//free memory of source_points
				source_points.swap(std::vector<indexed_point>());
			}
			else {
				generate_lods_poisson(source_points);
				indexed_points_with_lod.swap(source_points);
			}

			// write back the lods, order is changed 
			source_pc.resize_lods();
			for (int i = 0; i < indexed_points_with_lod.size(); ++i) {
				source_pc.lod(indexed_points_with_lod[i].index) = indexed_points_with_lod[i].level();
			}
			auto stop_draw5 = std::chrono::steady_clock::now();
			diff5 = stop_draw5 - start_draw5;
			std::cout << "diff5: lod: " << diff5.count() << std::endl;
		}
		else {
			
			// skip the lod computation, reuse the attributes from file 
			indexed_points_with_lod.swap(source_points);
			
		}

		// 
		std::chrono::duration<double> diff6;
		auto start_draw6 = std::chrono::steady_clock::now();
		if (auto_chunk_cube_size) {
			vec3 pmin(std::numeric_limits<float>::infinity()), pmax(-std::numeric_limits<float>::infinity());
			for (int i = 0; i < source_pc.get_nr_points(); i++) {
				pmin.x() = std::min(source_pc.pnt(i).x(), pmin.x());
				pmin.y() = std::min(source_pc.pnt(i).y(), pmin.y());
				pmin.z() = std::min(source_pc.pnt(i).z(), pmin.z());

				pmax.x() = std::max(source_pc.pnt(i).x(), pmax.x());
				pmax.y() = std::max(source_pc.pnt(i).y(), pmax.y());
				pmax.z() = std::max(source_pc.pnt(i).z(), pmax.z());
			}
			vec3 ext = (pmax - pmin);
			float max_element = static_cast<float>(*std::max_element(ext.begin(), ext.end()));
			chunk_cube_size = max_element / max_num_chunks;		
		}

		//split point data into chunks
		chunked_indexed_points = chunks<indexed_point>(indexed_points_with_lod, chunk_cube_size);
		auto stop_draw6 = std::chrono::steady_clock::now();
		diff6 = stop_draw6 - start_draw6;
		std::cout << "diff6: chunks: " << diff6.count() << std::endl;
	}
	//read other point attribute arrays (labels)
	size_t num_points = chunked_indexed_points.num_points();
	point_labels.resize(num_points);

	//
	for (size_t i = 0; i < chunked_indexed_points.num_points(); ++i) {
		int index = chunked_indexed_points.points[i].index;
		if (source_pc.has_labels())
			point_labels[i] = source_pc.label(index);
		else
			point_labels[i] = (GLint)point_label::VISIBLE; // initialize point labels to VISIBLE if no labels are present
	}
	
	//convert to clod renderer point format
	chunked_points.move_from_chunks(std::move(chunked_indexed_points));
}
///
void pointcloud_cleaning_tool::on_point_cloud_fit_table()
{
	rgb color(1.0, 0.0, 0.0);
	//find weighted center
	vec3 centroid(0.f), pmin(std::numeric_limits<float>::infinity()), pmax(-std::numeric_limits<float>::infinity());
	float scale = 1.f;
	for (int i = 0; i < source_pc.get_nr_points(); ++i) {
		centroid += source_pc.pnt(i);
		pmin.x() = std::min(source_pc.pnt(i).x(), pmin.x()); pmin.y() = std::min(source_pc.pnt(i).y(), pmin.y());
		pmin.z() = std::min(source_pc.pnt(i).z(), pmin.z());
		pmax.x() = std::max(source_pc.pnt(i).x(), pmax.x()); pmax.y() = std::max(source_pc.pnt(i).y(), pmax.y());
		pmax.z() = std::max(source_pc.pnt(i).z(), pmax.z());
	}
	centroid /= source_pc.get_nr_points();
	vec3 ext = (pmax - pmin);

	if (pointcloud_fit_table) {
		scale = (1.0 / static_cast<double>(*std::max_element(ext.begin(), ext.end())));
	}

	{
		if (put_on_table) {
			vec3 position(0);
			position.y() = table_height - (pmin.y() - centroid.y()) * scale;
			source_pc.ref_point_cloud_position() = -centroid + position;
		}
		if (pointcloud_fit_table) {
			source_pc.ref_point_cloud_scale() = scale;
		}
	}
}
/// the transform matrix in source_pc will be changed 
void pointcloud_cleaning_tool::automatic_scale_to_fit_table() {
	//if (abs(source_pc.ref_point_cloud_scale() - 1) > 1e-6) return;

	mat4 table_mat = cgv::math::translate4(vec3(0, 1, 0));
	vec3 centroid(0.f), pmin(std::numeric_limits<float>::infinity()), pmax(-std::numeric_limits<float>::infinity());
	for (int i = 0; i < source_pc.get_nr_points(); i++) {
		centroid += source_pc.pnt(i);

		pmin.x() = std::min(source_pc.pnt(i).x(), pmin.x());
		pmin.y() = std::min(source_pc.pnt(i).y(), pmin.y());
		pmin.z() = std::min(source_pc.pnt(i).z(), pmin.z());

		pmax.x() = std::max(source_pc.pnt(i).x(), pmax.x());
		pmax.y() = std::max(source_pc.pnt(i).y(), pmax.y());
		pmax.z() = std::max(source_pc.pnt(i).z(), pmax.z());
	}
	centroid /= source_pc.get_nr_points();
	vec3 ext = (pmax - pmin);
	auto_positioning_scale = (1.0 / static_cast<double>(*std::max_element(ext.begin(), ext.end())));

	//std::cout << "ext: " << ext << std::endl;
	//std::cout << "scale: " << scale << std::endl;
	//std::cout << "centroid: " << centroid << std::endl;

	// do not change the point cloud directly 
	//source_pc.ref_point_cloud_position() = -centroid;
	//source_pc.ref_point_cloud_scale() = scale;
	//source_pc.ref_render_style().pointSize = scale * 0.1f;

	// 
	mat4 RST = cgv::math::scale4(auto_positioning_scale, auto_positioning_scale, auto_positioning_scale) 
		* cgv::math::translate4(-centroid);
	// adjust point size in rendering style  
	source_pc.ref_render_style().pointSize = auto_positioning_scale * 0.1f;

	// update matrix for rendering 
	if (recompute_centroid) {
		// re-compute centroid ... TODO: use .lift()
		vec3 newcentroid = vec3(0);
		for (int i = 0; i < source_pc.get_nr_points(); i++) {
			vec4 tmpposi = vec4(source_pc.pnt(i), 1);
			vec4 transformed_posi_4 = RST * tmpposi;
			vec3 transformed_posi_3;
			transformed_posi_3.x() = transformed_posi_4.x();
			transformed_posi_3.y() = transformed_posi_4.y();
			transformed_posi_3.z() = transformed_posi_4.z();
			newcentroid += transformed_posi_3;
		}
		newcentroid /= source_pc.get_nr_points();

		// TODO(ZY): additional model matrix to support positioning 
		addi_model_transform = table_mat * cgv::math::translate4(-newcentroid) * RST;
	}
	else {
		addi_model_transform = RST;
	}
}
///
void pointcloud_cleaning_tool::compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color)
{
	for (size_t i = 0; i < floor_box.size(); ++i) {
		float t_result;
		vec3  p_result;
		vec3  n_result;
		if (cgv::media::ray_axis_aligned_box_intersection(
			origin, direction,
			floor_box[i],
			t_result, p_result, n_result, 0.000001f)) {
			// store intersection information
			
			intersection_points.push_back(p_result);
			intersection_colors.push_back(color);
			intersection_box_indices.push_back((int)i);
			intersection_controller_indices.push_back(ci);
		}
	}
}

void pointcloud_cleaning_tool::on_load_point_cloud_cb()
{
	std::string fn = cgv::gui::file_open_dialog("source point cloud(*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt)", "Point cloud files:*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt;");
	if (fn.empty())
		return;

	//clear parameters 
	on_clear_point_cloud_cb();
	source_pc.read(fn);
	std::cout << "source_pc.read done." << std::endl;

	//create default labels until we can store them
	//std::vector<int> source_labels = std::vector<int>(source_pc.get_nr_points(), (GLint)point_label::VISIBLE);

	//the octree_lod_generator expects the input points to be an array of structs, so we need to reshape the data
	//prepare_point_cloud() does this, the labels from source_pc_labels will end up in point_labels and the points in chunked_points.points
	prepare_point_cloud();
	std::cout << "prepare_point_cloud done." << std::endl;

	if (pointcloud_fit_table) {
		automatic_scale_to_fit_table();
		std::cout << "automatic_scale_to_fit_table done." << std::endl;	
	}
	init_label_buffer = true;
	renderer_out_of_date = true;
	post_redraw();

	std::cout << "loaded pointcloud " << fn << " with " << source_pc.get_nr_points() << " points!\n";
}
///
void pointcloud_cleaning_tool::parallel_saving() {
	std::string fn = cgv::gui::file_save_dialog("point cloud(*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt)", "Point cloud files:*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt;");
	if (fn.empty())
		return;
	//TODO write back edited pointcloud to file
	source_pc.write(fn);
	std::cout << "saved!" << std::endl;
}
///
void pointcloud_cleaning_tool::on_save_point_cloud_cb()
{
	//copy label buffer from gpu memory to point_labels
	sync_data(PA_ALL);
	//copy data from chunks to source_pc for the case anything was changed, also applies point deletions
	copy_chunks_to_point_cloud(source_pc);
	writing_thread = new std::thread(&pointcloud_cleaning_tool::parallel_saving, this);
}
///
void pointcloud_cleaning_tool::on_clear_point_cloud_cb()
{
	source_pc.clear();
	label_history.remove_all();
	chunked_points = chunks<LODPoint>();
	renderer_out_of_date = true;
	post_redraw();
}
///
void pointcloud_cleaning_tool::on_randomize_position_cb()
{
	uniform_real_distribution<float> angle_distribution(0.f, 3.142f);
	uniform_real_distribution<float> direction_distribution(0.f, 0.05f);
	random_device rng;
	float angle = rot_intensity*angle_distribution(rng);
	source_pc.rotate(cgv::math::quaternion<float>(normalize(vec3(direction_distribution(rng), direction_distribution(rng), direction_distribution(rng))), angle));
	source_pc.translate(trans_intensity*vec3(direction_distribution(rng), direction_distribution(rng), direction_distribution(rng)));
	renderer_out_of_date = true;
	post_redraw();
}
///
void pointcloud_cleaning_tool::on_point_cloud_style_cb()
{
	post_redraw();
}
///
void pointcloud_cleaning_tool::on_lod_mode_change()
{
	renderer_out_of_date = true;
}
///
void pointcloud_cleaning_tool::on_move_to_center() {
	dvec4 centroid = dvec4(0, 0, 0, 1);
	for (int i = 0; i < source_pc.get_nr_points(); ++i) {
		const vec3& pnt = source_pc.pnt(i);
		centroid += dvec4(pnt.x(), pnt.y(), pnt.z(), 0.0);
	}
	centroid = centroid / source_pc.get_nr_points();

	source_pc.ref_point_cloud_position() = vec3(-centroid.x(),-centroid.y(),-centroid.z());
}
/// construct boxes that represent a table of dimensions tw,td,th and leg width tW
void pointcloud_cleaning_tool::construct_table(float tw, float td, float th, float tW) {
	// construct table
	rgb table_clr(0.3f, 0.2f, 0.0f);
	boxes.push_back(box3(
		vec3(-0.5f * tw - 2 * tW, th - tW, -0.5f * td - 2 * tW),
		vec3(0.5f * tw + 2 * tW, th, 0.5f * td + 2 * tW)));
	box_colors.push_back(table_clr);

	boxes.push_back(box3(vec3(-0.5f * tw, 0, -0.5f * td), vec3(-0.5f * tw - tW, th - tW, -0.5f * td - tW)));
	boxes.push_back(box3(vec3(-0.5f * tw, 0, 0.5f * td), vec3(-0.5f * tw - tW, th - tW, 0.5f * td + tW)));
	boxes.push_back(box3(vec3(0.5f * tw, 0, -0.5f * td), vec3(0.5f * tw + tW, th - tW, -0.5f * td - tW)));
	boxes.push_back(box3(vec3(0.5f * tw, 0, 0.5f * td), vec3(0.5f * tw + tW, th - tW, 0.5f * td + tW)));
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
}
/// construct boxes that represent a room of dimensions w,d,h and wall width W
void pointcloud_cleaning_tool::construct_room(float w, float d, float h, float W, bool walls, bool ceiling) {	
	// construct floor
	/*boxes.push_back(box3(vec3(-0.5f * w, -W, -0.5f * d), vec3(0.5f * w, 0, 0.5f * d)));
	box_colors.push_back(rgb(0.2f, 0.2f, 0.2f));*/

	floor_box.push_back(box3(vec3(-0.5f * w, -W, -0.5f * d), vec3(0.5f * w, 0, 0.5f * d)));
	floor_box_clr.push_back((rgb(0.2f, 0.2f, 0.2f)));
	std::cout << "f: "<<floor_box.size() << std::endl;

	if (walls) {
		// construct walls
		boxes.push_back(box3(vec3(-0.5f * w, -W, -0.5f * d - W), vec3(0.5f * w, h, -0.5f * d)));
		box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));
		boxes.push_back(box3(vec3(-0.5f * w, -W, 0.5f * d), vec3(0.5f * w, h, 0.5f * d + W)));
		box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));

		boxes.push_back(box3(vec3(0.5f * w, -W, -0.5f * d - W), vec3(0.5f * w + W, h, 0.5f * d + W)));
		box_colors.push_back(rgb(0.5f, 0.8f, 0.5f));
	}
	if (ceiling) {
		// construct ceiling
		boxes.push_back(box3(vec3(-0.5f * w - W, h, -0.5f * d - W), vec3(0.5f * w + W, h + W, 0.5f * d + W)));
		box_colors.push_back(rgb(0.5f, 0.5f, 0.8f));
	}
}
///
void pointcloud_cleaning_tool::construct_environment(float s, float ew, float ed, float w, float d, float h) {
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	unsigned n = unsigned(ew / s);
	unsigned m = unsigned(ed / s);
	float ox = 0.5f * float(n) * s;
	float oz = 0.5f * float(m) * s;
	for (unsigned i = 0; i < n; ++i) {
		float x = i * s - ox;
		for (unsigned j = 0; j < m; ++j) {
			float z = j * s - oz;
			if (fabsf(x) < 0.5f * w && fabsf(x + s) < 0.5f * w && fabsf(z) < 0.5f * d && fabsf(z + s) < 0.5f * d)
				continue;
			float h = 0.2f * (std::max(abs(x) - 0.5f * w, 0.0f) + std::max(abs(z) - 0.5f * d, 0.0f)) * distribution(generator) + 0.1f;
			boxes.push_back(box3(vec3(x, 0.0f, z), vec3(x + s, h, z + s)));
			constexpr float hue_radius = 0.3;
			constexpr float hue_center = 0.4;
			rgb color = cgv::media::color<float, cgv::media::HLS>(fmod(hue_center + hue_radius * distribution(generator), 1.f), 0.1f * distribution(generator) + 0.15f, 0.6f);
			box_colors.push_back(color);
		}
	}
}
/// construct a scene with a table
void pointcloud_cleaning_tool::build_scene(float w, float d, float h, float W, float tw, float td, float th, float tW)
{
	construct_room(w, d, h, W, false, false);
	if(show_table)
		construct_table(tw, td, th, tW);
	if (show_environment)
		construct_environment(0.30f, 3 * w, 3 * d, w, d, h);
}
///
void pointcloud_cleaning_tool::build_test_object_32()
{
	int grid_size = 64;
	source_pc = build_test_point_cloud(grid_size, grid_size, grid_size, grid_size, 1.0f);
	renderer_out_of_date = true;
}
///
point_cloud pointcloud_cleaning_tool::build_test_point_cloud(int x, int y, int z, int grid_size, float cube_size)
{
	double dgrid_size = grid_size;
	point_cloud pc;
	pc.create_colors();
	for (int x = 0; x < grid_size; ++x) {
		for (int y = 0; y < grid_size; ++y) {
			for (int z = 0; z < grid_size; ++z) {
				double dx = x, dy = y, dz = z;
				int i = std::max(std::max(x, y), z) + 1;
				vec3 v(dx, dy, dz);
				v *= cube_size / grid_size;
				while (i > 0) {
					pc.add_point(v);
					pc.clr(pc.get_nr_points() - 1) = rgb8(i);
					--i;
				}
			}
		}
	}
	return std::move(pc);
}
///
void pointcloud_cleaning_tool::label_points_in_sphere(cgv::render::context& ctx, const GLint label, const int32_t point_group_mask, vec3 position, const float radius, const GLuint points) {
	
	//
	float scaled_radius = radius / (source_pc.ref_point_cloud_scale() * auto_positioning_scale);
	dmat4 concat_trans = model_transform * addi_model_transform;

	// find affected chunks with transformed position and scale, model not changed  
	std::vector<uint32_t> chunks;
	vec4 position_in_model_space = inv(concat_trans) * dvec4(position.lift());
	vec3 position_in_model_space_vec3 = vec3(position_in_model_space.x(), position_in_model_space.y(), position_in_model_space.z());
	std::cout << "scaled_radius: " << scaled_radius  << std::endl;
	chunked_points.intersect_sphere(position_in_model_space_vec3, scaled_radius, chunks); // points, bboxes are not transformed 
	std::cout << "number of chunks intersected: " << chunks.size() << std::endl;

	// pass to labeling shader, model will be transformed, radius wont changed 
	constexpr int points_pos = 1, index_pos = 2, labels_pos = 6;
	vec4 sphere =  dvec4(position.lift()); sphere.w() = radius;
	std::cout << "sphere " << sphere << std::endl;
	mat4 float_trans_matrix = concat_trans;
	labeling_tool_prog.set_uniform(ctx, "picked_index", picked_sphere_index);
	labeling_tool_prog.set_uniform(ctx, "sphere", sphere, true);
	//labeling_tool_prog.set_uniform(ctx, "sphere_position", position, true);
	//labeling_tool_prog.set_uniform(ctx, "sphere_radius", radius, true);
	labeling_tool_prog.set_uniform(ctx, "point_label", label, true);
	labeling_tool_prog.set_uniform(ctx, "model_transform", float_trans_matrix, true);
	labeling_tool_prog.set_uniform(ctx, "point_groups", point_group_mask & (int32_t)point_label::GROUP_MASK);

	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, points_pos, points);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, labels_pos, point_label_buffer);

	if(enable_label_history)
		label_history.bind(ctx);

	labeling_tool_prog.enable(ctx);

	if (use_chunks) {
		for (uint32_t id : chunks) {
			labeling_tool_prog.set_uniform(ctx, "batch_size", (GLint)chunked_points.num_points_in_chunk(id),true);
			labeling_tool_prog.set_uniform(ctx, "batch_offset", (GLint)chunked_points.start_index(id), true);

			// run computation
			glDispatchCompute((chunked_points.num_points_in_chunk(id) / 128) + 1, 1, 1);
		}	
	}
	else {
		labeling_tool_prog.set_uniform(ctx, "batch_size", (GLint)chunked_points.num_points(), true);
		labeling_tool_prog.set_uniform(ctx, "batch_offset", (GLint)0, true);

		// run computation
		glDispatchCompute(((GLint)chunked_points.num_points() / 128) + 1, 1, 1);
	}
	
	// synchronize
	glMemoryBarrier(GL_ALL_BARRIER_BITS);

	labeling_tool_prog.disable(ctx);
	if (enable_label_history)
		label_history.add_rollback_operation();
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}
///
void pointcloud_cleaning_tool::label_points_in_cube(cgv::render::context& ctx, const GLint label, const int32_t point_group_mask, vec3 position, const vec3 max_extent, const vec3 min_extent, const GLuint points) {

	//
	//float scaled_radius = radius / (source_pc.ref_point_cloud_scale() * auto_positioning_scale);
	dmat4 concat_trans = model_transform * addi_model_transform;

	// find affected chunks with transformed position and scale, model not changed  
	std::vector<uint32_t> chunks;
	vec4 position_in_model_space = inv(concat_trans) * dvec4(position.lift());
	vec3 position_in_model_space_vec3 = vec3(position_in_model_space.x(), position_in_model_space.y(), position_in_model_space.z());
	//std::cout << "scaled_radius: " << scaled_radius << std::endl;
	chunked_points.intersect_cube(position_in_model_space_vec3, max_extent, min_extent, chunks); // points, bboxes are not transformed 
	std::cout << "number of chunks intersected: " << chunks.size() << std::endl;

	// pass to labeling shader, model will be transformed, radius wont changed 
	constexpr int points_pos = 1, index_pos = 2, labels_pos = 6;
	//vec4 sphere = dvec4(position.lift()); sphere.w() = radius;
	//std::cout << "sphere " << sphere << std::endl;
	mat4 float_trans_matrix = concat_trans;
	//labeling_tool_prog.set_uniform(ctx, "sphere", sphere, true);
	//labeling_tool_prog.set_uniform(ctx, "sphere_position", position, true);
	//labeling_tool_prog.set_uniform(ctx, "sphere_radius", radius, true);
	labeling_tool_prog.set_uniform(ctx, "picked_index", picked_sphere_index);
	labeling_tool_prog.set_uniform(ctx, "box_position", position);
	labeling_tool_prog.set_uniform(ctx, "box_max_extent", max_extent);
	labeling_tool_prog.set_uniform(ctx, "box_min_extent", min_extent);
	labeling_tool_prog.set_uniform(ctx, "point_label", label, true);
	labeling_tool_prog.set_uniform(ctx, "model_transform", float_trans_matrix, true);
	labeling_tool_prog.set_uniform(ctx, "point_groups", point_group_mask & (int32_t)point_label::GROUP_MASK);

	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, points_pos, points);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, labels_pos, point_label_buffer);

	if (enable_label_history)
		label_history.bind(ctx);

	labeling_tool_prog.enable(ctx);
	GLuint64 startTime, stopTime;
	unsigned int queryID[2];
	glGenQueries(2, queryID);
	glQueryCounter(queryID[0], GL_TIMESTAMP);

	if (use_chunks) {
		for (uint32_t id : chunks) {
			labeling_tool_prog.set_uniform(ctx, "batch_size", (GLint)chunked_points.num_points_in_chunk(id), true);
			labeling_tool_prog.set_uniform(ctx, "batch_offset", (GLint)chunked_points.start_index(id), true);

			// run computation
			glDispatchCompute((chunked_points.num_points_in_chunk(id) / 128) + 1, 1, 1);
		}
	}
	else {
		labeling_tool_prog.set_uniform(ctx, "batch_size", (GLint)chunked_points.num_points(), true);
		labeling_tool_prog.set_uniform(ctx, "batch_offset", (GLint)0, true);

		// run computation
		glDispatchCompute(((GLint)chunked_points.num_points() / 128) + 1, 1, 1);
	}

	// synchronize
	glMemoryBarrier(GL_ALL_BARRIER_BITS);

	glQueryCounter(queryID[1], GL_TIMESTAMP);

	GLint stopTimerAvailable = 0;
	while (!stopTimerAvailable) {
		glGetQueryObjectiv(queryID[1],
			GL_QUERY_RESULT_AVAILABLE,
			&stopTimerAvailable);
	}

	// get query results
	glGetQueryObjectui64v(queryID[0], GL_QUERY_RESULT, &startTime);
	glGetQueryObjectui64v(queryID[1], GL_QUERY_RESULT, &stopTime);

	//std::cout << "Time spent on the GPU: " << (stopTime - startTime) / 1000000.0 << "ms\n" << std::endl;

	labeling_tool_prog.disable(ctx);
	if (enable_label_history)
		label_history.add_rollback_operation();
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}
///
void pointcloud_cleaning_tool::label_points_by_clipping(cgv::render::context& ctx, const GLint label, const int32_t point_group_mask, vec3 position, mat3 plane_ori, const GLuint points) {

	//
	//float scaled_radius = radius / (source_pc.ref_point_cloud_scale() * auto_positioning_scale);
	dmat4 concat_trans = model_transform * addi_model_transform;

	// find affected chunks with transformed position and scale, model not changed  
	std::vector<uint32_t> chunks;
	vec4 position_in_model_space = inv(concat_trans) * dvec4(position.lift());
	vec3 position_in_model_space_vec3 = vec3(position_in_model_space.x(), position_in_model_space.y(), position_in_model_space.z());
	//std::cout << "scaled_radius: " << scaled_radius << std::endl;
	vec3 pl = plane_ori * vec3(1.0f, 0.f, 0.f);
	chunked_points.intersect_plane(pl, position_in_model_space_vec3.length(), chunks); // points, bboxes are not transformed 
	std::cout << "number of chunks intersected: " << chunks.size() << std::endl;
	
	// pass to labeling shader, model will be transformed, radius wont changed 
	constexpr int points_pos = 1, index_pos = 2, labels_pos = 6;
	//vec4 sphere = dvec4(position.lift()); sphere.w() = radius;
	//std::cout << "sphere " << sphere << std::endl;
	mat4 float_trans_matrix = concat_trans;
	labeling_tool_prog.set_uniform(ctx, "picked_index", picked_sphere_index);
	//labeling_tool_prog.set_uniform(ctx, "sphere", sphere, true);
	//labeling_tool_prog.set_uniform(ctx, "sphere_position", position, true);
	//labeling_tool_prog.set_uniform(ctx, "sphere_radius", radius, true);
	labeling_tool_prog.set_uniform(ctx, "box_position", position, true);
	labeling_tool_prog.set_uniform(ctx, "plane_nml", pl, true);
	labeling_tool_prog.set_uniform(ctx, "point_label", label, true);
	labeling_tool_prog.set_uniform(ctx, "model_transform", float_trans_matrix, true);
	labeling_tool_prog.set_uniform(ctx, "point_groups", point_group_mask & (int32_t)point_label::GROUP_MASK);

	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, points_pos, points);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, labels_pos, point_label_buffer);

	if (enable_label_history)
		label_history.bind(ctx);

	labeling_tool_prog.enable(ctx);
	GLuint64 startTime, stopTime;
	unsigned int queryID[2];
	glGenQueries(2, queryID);
	glQueryCounter(queryID[0], GL_TIMESTAMP);

	if (use_chunks) {
		for (uint32_t id : chunks) {
			labeling_tool_prog.set_uniform(ctx, "batch_size", (GLint)chunked_points.num_points_in_chunk(id), true);
			labeling_tool_prog.set_uniform(ctx, "batch_offset", (GLint)chunked_points.start_index(id), true);

			// run computation
			glDispatchCompute((chunked_points.num_points_in_chunk(id) / 128) + 1, 1, 1);
		}
	}
	else {
		labeling_tool_prog.set_uniform(ctx, "batch_size", (GLint)chunked_points.num_points(), true);
		labeling_tool_prog.set_uniform(ctx, "batch_offset", (GLint)0, true);

		// run computation
		glDispatchCompute(((GLint)chunked_points.num_points() / 128) + 1, 1, 1);
	}

	// synchronize
	glMemoryBarrier(GL_ALL_BARRIER_BITS);

	glQueryCounter(queryID[1], GL_TIMESTAMP);

	GLint stopTimerAvailable = 0;
	while (!stopTimerAvailable) {
		glGetQueryObjectiv(queryID[1],
			GL_QUERY_RESULT_AVAILABLE,
			&stopTimerAvailable);
	}

	// get query results
	glGetQueryObjectui64v(queryID[0], GL_QUERY_RESULT, &startTime);
	glGetQueryObjectui64v(queryID[1], GL_QUERY_RESULT, &stopTime);

	//std::cout << "Time spent on the GPU: " << (stopTime - startTime) / 1000000.0 << "ms\n" << std::endl;

	labeling_tool_prog.disable(ctx);
	if (enable_label_history)
		label_history.add_rollback_operation();
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}
///
void pointcloud_cleaning_tool::label_selected_points(cgv::render::context& ctx, const GLint new_label, const GLint expected_label, const int32_t point_group_mask, const GLuint points, const unsigned num_points)
{
	static constexpr int
		new_point_label_loc = 10,
		point_group_mask_loc = 11,
		batch_offset_loc = 21,
		batch_size_loc = 22,
		selected_point_label_loc = 12;

	constexpr int labels_pos = 6;
	constexpr int rollback_pos = 7;
	
	if (num_points == 0) {
		return;
	}
	
	GLint batch_offset = 0;
	GLint batch_size = num_points;
	
	selection_relabel_prog.set_uniform(ctx, new_point_label_loc, new_label);
	selection_relabel_prog.set_uniform(ctx, point_group_mask_loc, point_group_mask);
	selection_relabel_prog.set_uniform(ctx, selected_point_label_loc, expected_label);
	selection_relabel_prog.set_uniform(ctx, batch_size_loc, batch_size);
	selection_relabel_prog.set_uniform(ctx, batch_offset_loc, batch_offset);
	
	/*
	selection_relabel_prog.set_uniform(ctx, "new_point_label", new_label,true);
	selection_relabel_prog.set_uniform(ctx, "point_group_mask", point_group_mask,true);
	selection_relabel_prog.set_uniform(ctx, "selected_point_label", expected_label,true);
	selection_relabel_prog.set_uniform(ctx, "batch_size", batch_size,true);
	selection_relabel_prog.set_uniform(ctx, "batch_offset", batch_offset,true);
	*/
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, labels_pos, point_label_buffer);
	if (enable_label_history)
		label_history.bind(ctx);

	if (selection_relabel_prog.enable(ctx)) {
		// run computation
		glDispatchCompute((num_points / 128) + 1, 1, 1);
		// synchronize
		glMemoryBarrier(GL_ALL_BARRIER_BITS);
		selection_relabel_prog.disable(ctx);
	}
	//std::vector<label_operation> ops;
	//label_history.get_stored_labels(ops);
	
	//write operation to history 
	if (enable_label_history)
		label_history.add_rollback_operation();

	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}
///
void pointcloud_cleaning_tool::rollback_last_operation(cgv::render::context& ctx) {
	if (enable_label_history)
		label_history.rollback_last_operation(ctx);
}
///
void pointcloud_cleaning_tool::sync_data(int flags)
{
	if (flags & point_attributes::PA_LABELS) {
		assert(point_label_buffer != 0);
		GLint* labels = static_cast<GLint*>(glMapNamedBufferRange(
			point_label_buffer, 0, chunked_points.num_points() * sizeof(GLint), GL_MAP_READ_BIT));
		glCheckError();

		memcpy(point_labels.data(), labels, point_labels.size() * sizeof(int));

		glUnmapNamedBuffer(point_label_buffer);
	}
	if (flags & point_attributes::PA_COLORS || flags & point_attributes::PA_POSITIONS || flags & point_attributes::PA_LODS) {
		//placeholder - not implemented
	}
}

void pointcloud_cleaning_tool::copy_chunks_to_point_cloud(point_cloud& dest) {
	int chunks_i = 0, pc_i = 0;
	size_t num_points = chunked_points.num_points();
	dest.resize(num_points);
	source_pc.resize_labels();
	if (dest.has_colors() && dest.has_labels() && dest.has_lods()) {
		while (chunks_i < num_points) { // itereate all chunks 
			auto& pnt = chunked_points.points[chunks_i];
			if (point_labels[chunks_i] != (int)point_label::DELETED) {
				dest.pnt(pc_i) = pnt.position();
				dest.clr(pc_i) = pnt.color();
				dest.lod(pc_i) = pnt.level();
				dest.label(pc_i) = point_labels[chunks_i];
				++pc_i;
			}
			++chunks_i;
		}
		dest.resize(pc_i);
	}
}

///
void pointcloud_cleaning_tool::clear_scene()
{
	boxes.clear();
	box_colors.clear();
	floor_box.clear();
	floor_box_clr.clear();
}
///
void pointcloud_cleaning_tool::create_gui()
{
	bool gui_scaling = false;
	bool gui_marking = false;
	bool gui_rendering = false;
	bool gui_chunking = false;
	bool gui_manual_positioning = false;
	bool gui_clod_rendering = false;

	add_decorator("Point cloud", "heading", "level=1");
	connect_copy(add_button("load point cloud")->click, rebind(this, &pointcloud_cleaning_tool::on_load_point_cloud_cb));
	connect_copy(add_button("save point cloud")->click, rebind(this, &pointcloud_cleaning_tool::on_save_point_cloud_cb));
	connect_copy(add_button("clear point cloud")->click, rebind(this, &pointcloud_cleaning_tool::on_clear_point_cloud_cb));
	connect_copy(add_button("build test point cloud")->click, rebind(this, &pointcloud_cleaning_tool::build_test_object_32));
	connect_copy(add_button("colorize with height")->click, rebind(this, &pointcloud_cleaning_tool::colorize_with_height));
	connect_copy(add_button("print point cloud info")->click, rebind(this, &pointcloud_cleaning_tool::print_point_cloud_info));
	add_member_control(this, "show environment", show_environment, "toggle");
	add_member_control(this, "show table", show_table, "toggle");

	if (begin_tree_node("CLOD Rendering", gui_clod_rendering, gui_clod_rendering)) {
		align("\a");
		add_member_control(this, "show LODs", color_based_on_lod, "toggle");
		add_member_control(this, "point limit", max_points, "value_slider", "min=10000;max=100000000;log=false;ticks=true");
		std::string mode_defs = "enums='random=2;octree=1'";
		connect_copy(add_control("lod generator", (DummyEnum&)lod_mode, "dropdown", mode_defs)->value_change, 
			rebind(this, &pointcloud_cleaning_tool::on_lod_mode_change));
		if (begin_tree_node("CLOD render style", source_pc.ref_render_style(), false)) {
			align("\a");
			add_gui("clod style", source_pc.ref_render_style());
			align("\b");
			end_tree_node(source_pc.ref_render_style());
		}
		align("\b");
	}

	if (begin_tree_node("Auto Point Cloud Positioning", gui_scaling, gui_scaling)) {
		add_member_control(this, "auto-scale pointcloud", pointcloud_fit_table, "toggle");
		add_member_control(this, "recompute_centroid", recompute_centroid, "check");
		//connect_copy(add_button("move point cloud to center")->click, rebind(this, &pointcloud_cleaning_tool::on_move_to_center));
	}

	if (begin_tree_node("Point Cloud Chunking", gui_chunking, gui_chunking)) {
		align("\a");
		add_member_control(this, "auto_chunk_cube_size", auto_chunk_cube_size, "check");
		add_member_control(this, "render with chunks enabled", use_chunks, "check");
		add_member_control(this, "show chunk bounding boxes", draw_chunk_bounding_boxes, "check");
		add_member_control(this, "chunk cube size", chunk_cube_size, "value_slider", "min=0.1;max=5.0;log=false;ticks=true");
		add_member_control(this, "max num chunks", max_num_chunks, "value_slider", "min=1;max=1000;log=false;ticks=true");
		add_view("chunks", chunk_render_message);
		if (begin_tree_node("cone style", cone_style, false)) {
			align("\a");
			add_gui("cone style", cone_style);
			align("\b");
			end_tree_node(cone_style);
		}
		align("\b");
	}

	if (begin_tree_node("Point Cloud Marking", gui_marking, gui_marking)) {
		align("\a");
		add_member_control(this, "use label shader", use_label_prog, "toggle");
		connect_copy(add_button("test labeling of points")->click, rebind(this, &pointcloud_cleaning_tool::test_labeling_of_points));
		add_member_control(this, "radius", radius_for_test_labeling, "value_slider", "min=0.1;max=100;log=false;ticks=true");
		connect_copy(add_button("rollback one step")->click, rebind(this, &pointcloud_cleaning_tool::schedule_a_rollback));
		add_member_control(this, "enable label history", enable_label_history, "check");
		if (begin_tree_node("point cloud cleaning[grip]", gui_culling_protection_zone, false)) {
			//add_member_control(this, "radius controller 1", point_selection_radii[0], "value_slider", "min=0.0;max=1.0;log=false;ticks=true");
			//add_member_control(this, "radius controller 2", point_selection_radii[1], "value_slider", "min=0.0;max=1.0;log=false;ticks=true");
			add_gui("sphere_style_rhand", sphere_style_rhand);
			add_gui("sphere_style_lhand", sphere_style_lhand);
		}
		align("\b");
	}

	if (begin_tree_node("Manual Positioning", gui_manual_positioning, gui_manual_positioning)) {
		add_member_control(this, "model scale", source_pc.ref_point_cloud_scale(), "value_slider", "min=0.1;max=5.0;log=false;ticks=true");
		add_member_control(this, "model position x", source_pc.ref_point_cloud_position().x(), "value_slider", "min=-10.0;max=10.0;log=false;ticks=true");
		add_member_control(this, "model position y", source_pc.ref_point_cloud_position().y(), "value_slider", "min=-10.0;max=10.0;log=false;ticks=true");
		add_member_control(this, "model position z", source_pc.ref_point_cloud_position().z(), "value_slider", "min=-10.0;max=10.0;log=false;ticks=true");
		add_member_control(this, "model rotation x", source_pc.ref_point_cloud_rotation().x(), "value_slider", "min=0.0;max=360;log=false;ticks=true");
		add_member_control(this, "model rotation y", source_pc.ref_point_cloud_rotation().y(), "value_slider", "min=0.0;max=360;log=false;ticks=true");
		add_member_control(this, "model rotation z", source_pc.ref_point_cloud_rotation().z(), "value_slider", "min=0.0;max=360;log=false;ticks=true");
	}
}
#include "lib_begin.h"
#include <cgv/base/register.h>

cgv::base::object_registration<pointcloud_cleaning_tool> pointcloud_cleaning_tool_reg("");