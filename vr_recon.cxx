#include "vr_recon.h"

///@ingroup VR
///@{

/**@file
   example plugin for vr usage
*/


#include <cgv/base/base.h> // this should be first header to avoid warning
#include <omp.h>
#include <cgv_gl/gl/gl.h>
#include <cgv/gui/trigger.h>
#include <cgv/gui/dialog.h>
#include <cgv/gui/key_event.h>
#include <cgv/gui/file_dialog.h>
#include <cgv/utils/convert.h>
#include <cgv/utils/file.h>
#include <cgv/utils/dir.h>
#include <cgv/utils/statistics.h>
#include <cgv/type/standard_types.h>
#include <cgv/math/ftransform.h>
#include <cgv/math/svd.h>
#include "rgbd_pointcloud.h"
#include "ICP.h"
#include "PCBoundingbox.h"
#include "GoICP.h"
#include "SICP.h"

#include <numeric>
#include <thread>


using namespace std;
using namespace cgv::base;
using namespace cgv::signal;
using namespace cgv::type;
using namespace cgv::gui;
using namespace cgv::data;
using namespace cgv::utils;
using namespace cgv::render;
using namespace rgbd;




/// the plugin class vr_rgbd inherits like other plugins from node, drawable and provider

std::string get_stream_format_enum(const std::vector<rgbd::stream_format>& sfs)
{
	std::string enum_def = "enums='default=-1";
	for (const auto& sf : sfs) {
		enum_def += ",";
		enum_def += to_string(sf);
	}
	return enum_def + "'";
}







void vr_rgbd::compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color)
{
	for (size_t i = 0; i < movable_boxes.size(); ++i) {
		vec3 origin_box_i = origin - movable_box_translations[i];
		movable_box_rotations[i].inverse_rotate(origin_box_i);
		vec3 direction_box_i = direction;
		movable_box_rotations[i].inverse_rotate(direction_box_i);
		float t_result;
		vec3  p_result;
		vec3  n_result;
		if (cgv::media::ray_axis_aligned_box_intersection(
			origin_box_i, direction_box_i,
			movable_boxes[i],
			t_result, p_result, n_result, 0.000001f)) {

			// transform result back to world coordinates
			movable_box_rotations[i].rotate(p_result);
			p_result += movable_box_translations[i];
			movable_box_rotations[i].rotate(n_result);

			// store intersection information
			intersection_points.push_back(p_result);
			intersection_colors.push_back(color);
			intersection_box_indices.push_back((int)i);
			intersection_controller_indices.push_back(ci);
		}
	}
}

	/// register on device change events
void vr_rgbd::on_device_change(void* kit_handle, bool attach)
{
		post_recreate_gui();
}
void vr_rgbd::build_scene(float w, float d, float h, float W,
		float tw, float td, float th, float tW)
{
	boxes.push_back(box3(vec3(-0.9f * w, -W, -1.1f * d)+vec3(1, -1, 2), vec3(0.9f * w, 0, 1.1f * d) + vec3(1, -1, 2)));
	box_colors.push_back(rgb(0.2f, 0.2f, 0.2f));
		//construct_room(w, d, h, W, false, false);
		//construct_table(tw, td, th, tW);
		//construct_environment(0.2f, 3 * w, 3 * d, h, w, d, h);
		//construct_movable_boxes(tw, td, th, tW, 20);
}
	/// generate a random point cloud
void vr_rgbd::generate_point_cloud(std::vector<vertex>& pc)
{
		std::default_random_engine r;
		std::uniform_real_distribution<float> d(0.0f,1.0f);
		vec3 S(0.0f, 2.0f, 0.0f);
		vec3 V(1.0f, 0, 0);
		vec3 U(0.0f, 1.0f, 0);
		vec3 X = cross(V, U);
		float aspect = 1.333f;
		float tan_2 = 0.3f;
		for (int i = 0; i < 10000; ++i) {
			float x = 2 * d(r) - 1;
			float y = 2 * d(r) - 1;
			float z = d(r) + 1;
			vec3  p = x * aspect * tan_2 * z * X + y * tan_2 * z * U + z*V;
			rgba8 c((cgv::type::uint8_type)(255*d(r)), 0, 0);
			vertex v;
			v.point = S + p;
			v.color = c;
			pc.push_back(v);
		}
}
	/// start the rgbd device


void vr_rgbd::start_rgbd()
{
	
	//std::cout << "device idx:" << device_idx << std::endl;
		if (!rgbd_inp.is_attached()) 
		{
			if (rgbd::rgbd_input::get_nr_devices() == 0)
			{				
				return;			
			}
			
			if (!rgbd_inp.attach(rgbd::rgbd_input::get_serial(device_idx)))//device_idx
			{
				return;		
			}	
		}
		//std::cout << "nr_device:" << device_idx << std::endl;
		rgbd_inp.set_near_mode(true);
		std::vector<rgbd::stream_format> stream_formats;
		rgbd_started = rgbd_inp.start(rgbd::IS_COLOR_AND_DEPTH, stream_formats);

		

		update_member(&rgbd_started);	

}
	/// stop rgbd device

void vr_rgbd::attach_all_devices() 
{
	if (rgbd_inp.is_attached()) {
		if(rgbd_inp.is_started()){
			rgbd_inp.stop();
			rgbd_started = !rgbd_inp.stop();
			update_member(&rgbd_started);
		}	
		rgbd_inp.detach();	
	}

	if (!rgbd_inp.is_multi_attached())
	{

		if (rgbd::rgbd_input::get_nr_devices() == 0)
		{

			return;
		}

		vector<std::string> ser;

		for (int nr_de = 0; nr_de < rgbd::rgbd_input::get_nr_devices(); nr_de++)
		{
			ser.push_back(rgbd::rgbd_input::get_serial(nr_de));
		}


		if (!rgbd_inp.multi_attach(ser))
		{
			return;
		}
		std::cout << "size of multi_rgbd:" << rgbd_inp.nr_multi_de() << std::endl;

	}
	all_devices_attached = rgbd_inp.is_multi_attached();
	if (rgbd_inp.is_multi_attached())
		std::cout << "these are attached" << std::endl;
	else
		std::cout << "these are not attached" << std::endl;
	
	update_stream_formats();

	
	current_corrected_cam = -1;
	//save point cloud
	intermediate_rgbdpc.resize(rgbd_inp.get_nr_devices());
	intermediate_rgbdpc_bbox.resize(rgbd_inp.get_nr_devices());
	intermediate_rgbdpc_outside_bbox.resize(rgbd_inp.get_nr_devices());
	rgbdpc.resize(rgbd_inp.get_nr_devices());
	rgbdpc_in_box.resize(rgbd_inp.get_nr_devices());
	rgbdpc_outside_box.resize(rgbd_inp.get_nr_devices());
	cam_fine_r.resize(rgbd_inp.get_nr_devices());
	cam_fine_t.resize(rgbd_inp.get_nr_devices());

	cam_coarse_t.resize(rgbd_inp.get_nr_devices());
	cam_coarse_r.resize(rgbd_inp.get_nr_devices());
	manualcorrect_translation.resize(rgbd_inp.get_nr_devices());
	manualcorrect_rotation.resize(rgbd_inp.get_nr_devices());
	trees.resize(rgbd_inp.get_nr_devices());

	color_frame.resize(rgbd_inp.get_nr_devices());
	depth_frame.resize(rgbd_inp.get_nr_devices());
	warped_color_frame.resize(rgbd_inp.get_nr_devices());
	
	color_frame_2.resize(rgbd_inp.get_nr_devices());
	depth_frame_2.resize(rgbd_inp.get_nr_devices());
	warped_color_frame_2.resize(rgbd_inp.get_nr_devices());
	ir_frame_2.resize(rgbd_inp.get_nr_devices());

	PCfuture_handle.resize(rgbd_inp.get_nr_devices());


	for (int i = 0; i < cam_fine_r.size(); i++) {
		cam_coarse_r[i].identity();
		cam_coarse_t[i] = vec3(0, 0, 0);

		cam_fine_r[i].identity();
		cam_fine_t[i] = vec3(0, 0, 0);

		manualcorrect_rotation[i].identity();
		manualcorrect_translation[i] = vec3(0, 0, 0);
		rgbdpc[i].cam_pos = vec3(0, 0, 0);
		
	}
	
	update_member(&all_devices_attached);
}
void vr_rgbd::detach_all_devices() {
	if (!rgbd_inp.is_multi_attached())
	{
		return;
	}

	if (rgbd_inp.is_multi_started())
		stop_all_rgbd();
	else {
		rgbd_inp.detach();
		rgbdpc.clear();
		intermediate_rgbdpc.clear();
		intermediate_rgbdpc_bbox.clear();
		intermediate_rgbdpc_outside_bbox.clear();
		rgbdpc_in_box.clear();
		rgbdpc_outside_box.clear();
		cam_coarse_t.clear();
		cam_coarse_r.clear();
		cam_fine_t.clear();
		cam_fine_r.clear();
		manualcorrect_translation.clear();
		manualcorrect_rotation.clear();
		current_corrected_cam = -1;
		trees.clear();
		
		std::cout << "nr of attached devices" << rgbd_inp.nr_multi_de() << std::endl;
		current_pc.clear();

		color_frame.clear();
		depth_frame.clear();
		warped_color_frame.clear();

		color_frame_2.clear();
		depth_frame_2.clear();
		warped_color_frame_2.clear();
		ir_frame_2.clear();
	
		PCfuture_handle.clear();
	}
		
	all_devices_attached = rgbd_inp.is_multi_attached();
	
	update_member(&all_devices_attached);
}








void vr_rgbd::start_multi_rgbd()
{
	
	if (!rgbd_inp.is_multi_attached())
	{
		std::cout << "no attached devices!" << std::endl;
		rgbd_multi_started = false;
		update_member(&rgbd_multi_started);
		return;
	}
	setboundingboxmode=false;
	selectPointsmode = false;
	boundingboxisfixed = false;
	manualcorrectmode = false;
	mode=0;

	bool use_default = false;
	std::vector<stream_format> sfs;
	if (color_stream_format_idx == -1)
		use_default = true;
	else
		sfs.push_back(color_stream_formats[color_stream_format_idx]);
	if (depth_stream_format_idx == -1)
		use_default = true;
	else
		sfs.push_back(depth_stream_formats[depth_stream_format_idx]);


	
	std::vector<std::vector<rgbd::stream_format>> multi_stream_formats(rgbd_inp.nr_multi_de());    //??

	if (use_default)
		rgbd_multi_started = rgbd_inp.multi_start(rgbd::IS_COLOR_AND_DEPTH, multi_stream_formats);
	else {
	for (int i = 0; i < multi_stream_formats.size(); i++)
		multi_stream_formats[i] = sfs;

	//rgbd_multi_started = rgbd_inp.multi_start(rgbd::IS_COLOR_AND_DEPTH, multi_stream_formats);
	
	rgbd_multi_started = rgbd_inp.multi_start(multi_stream_formats);
	
	}


	if (rgbd_multi_started)
	{
		generate_pc_from_rgbd = true;
		std::cout << "they are started" << std::endl;
	}
	else
		std::cout << "they are not started" << std::endl;
	
	
	
	update_member(&rgbd_multi_started);
	
}
void vr_rgbd::stop_all_rgbd() 
{
	//std::cout << "nr of attached devices" << rgbd_inp.nr_multi_de() << std::endl;
	if (!rgbd_inp.is_multi_started())
		return;
	//rgbd_inp.stop();//
	
	rgbd_multi_started = !rgbd_inp.stop();
	generate_pc_from_rgbd = false;
	
	update_member(&rgbd_multi_started);
}





void vr_rgbd::stop_rgbd()
{
	if (!rgbd_inp.is_started())
		return;
	
	rgbd_inp.stop();
	rgbd_inp.detach();
	generate_pc_from_rgbd = false;
	rgbd_started = !rgbd_inp.stop();
	update_member(&rgbd_started);
}





void vr_rgbd::set_rgbd_pos()
{
	
	
	
	return;
}

vr_rgbd::vr_rgbd()
{
	
	set_name("vr_rgbd");
	controller_orientation.identity();
	controller_position = vec3(0, 1.5f, 0);

	rgbd_2_controller_orientation.identity();
	rgbd_2_controller_orientation.set_col(0, vec3(-1, 0, 0));
	rgbd_2_controller_orientation.set_col(1, vec3(0, -0.7071f, 0.7071f));
	rgbd_2_controller_orientation.set_col(2, vec3(0, 0.7071f, 0.7071f));
	rgbd_2_controller_position.zeros();

	rgbd_2_controller_orientation_start_calib.identity();
	rgbd_2_controller_position_start_calib.zeros();

	build_scene(5, 7, 3, 0.2f, 1.6f, 0.8f, 0.9f, 0.03f);
	//icp_pc->read("C:/Users/ltf/Desktop/test/monkey.obj");
	//pc2vertex(*icp_pc, current_pc);
	//generate_point_cloud(current_pc);
	vr_view_ptr = 0;
	ray_length = 2;
	connect(cgv::gui::ref_vr_server().on_device_change, this, &vr_rgbd::on_device_change);
	sphere_distance=0.2;
	srs.radius = 0.005f;
	state[0] = state[1] = state[2] = state[3] = IS_NONE;
	rgbd_started = false;
	rgbd_multi_started = false;
	all_devices_attached = false;
	get_tracker_positions = false;
	record_key_frames = false;
	show_points = true;
	point_style.point_size = 2;
	point_style.blend_points = false;
	point_style.blend_width_in_pixel = 0;
	max_nr_shown_recorded_pcs = 20;
	counter_pc = 0;
	record_pc_started=false;
	


	device_mode = No_Device;
	
	device_idx = -2;
	num_devices = 0;

	pc_file_path = QUOTE_SYMBOL_VALUE(INPUT_DIR) " / .. / data";

	connect(cgv::gui::get_animation_trigger().shoot, this, &vr_rgbd::timer_event);

	
	color_stream_format_idx = -1;
	depth_stream_format_idx = -1;
	ir_stream_format_idx = -1;
	

	no_controller=false;



	rotation_scale = 0.1;
	position_scale = 0.1;
	generate_pc_from_rgbd = false;
	selectPointsmode = false;
	//showPCinsidebbox = false;
	setboundingboxmode = false;
	setboundingboxstarted = false;
	showvoxelizationmode = false;
	boundingboxisfixed = false;

	showallpcmode = true;
	showPCinBboxmode = false;
	showvolumemode = false;
	showmeshmode = false;
	showPCoutsideBboxmode = false;
	showcentermassmode = false;

	manualcorrectmode = false;
	manualcorrectstarted = false;
	generate_pc_from_files = false;

	drawvoexls = true;

	int mode = 0;//?
	currentpointcloud = 0;
	Radius_SelectMode = 0.1;
	currentcamera = -1;
	resolution_bbox = 100;;
	range_1 = 3;
	range_2 = 5;
	threshold_1 = 5;
	threshold_2 = 13;

	BoundingBoxlength = 2.0;
	BoundingBoxheight = 2.0;
	BoundingBoxstep = 0.05;
	
	num_recorded_pc = 0;
	

	pcbb.pos1 = vec3(0.03623, -0.85, 2.45);
	pcbb.pos2 = vec3(2.03623, 1.15, 4.45);

}

vr_rgbd::~vr_rgbd()
{

}

size_t vr_rgbd::generate_mesh() {
	cgv::render::context& ctx = *get_context();
	if (rgbdpc_in_box.size() < 3)
		return 0;

	std::vector<vec3> currentcampos;
	currentcampos.push_back(rgbdpc_in_box[0].cam_pos);
	currentcampos.push_back(rgbdpc_in_box[1].cam_pos);
	currentcampos.push_back(rgbdpc_in_box[2].cam_pos);

	Vox->kernel_range_1 = range_1;
	Vox->kernel_range_2 = range_2;
	Vox->filter_threshold_1 = threshold_1;
	Vox->filter_threshold_2 = threshold_2;
	Vox->init(rgbdpc_in_box,ivec3(resolution_bbox, resolution_bbox, resolution_bbox) ,pcbb.pos1, pcbb.pos2,true);
	Vox->generate(ctx, currentcampos,true);



	return Vox->get_numBoxes();
}
	
	
size_t vr_rgbd::voxelize_PC() {
	cgv::render::context& ctx = *get_context();
	if (rgbdpc_in_box.size() <3 )
		return 0;
	 
	std::vector<vec3> currentcampos;
	currentcampos.push_back(rgbdpc_in_box[0].cam_pos);
	currentcampos.push_back(rgbdpc_in_box[1].cam_pos);
	currentcampos.push_back(rgbdpc_in_box[2].cam_pos);

	Vox->kernel_range_1 = range_1;
	Vox->kernel_range_2 = range_2;
	Vox->filter_threshold_1 = threshold_1;
	Vox->filter_threshold_2 = threshold_2;
	Vox->init(rgbdpc_in_box, ivec3(resolution_bbox, resolution_bbox, resolution_bbox), pcbb.pos1, pcbb.pos2,false);
	Vox->generate(ctx, currentcampos,false);

	

	return Vox->get_numBoxes();
}

	size_t vr_rgbd::construct_point_cloud()
	{
		intermediate_pc.clear();
		const unsigned short* depths = reinterpret_cast<const unsigned short*>(&depth_frame_2[0].frame_data.front());
		const unsigned char* colors = reinterpret_cast<const unsigned char*>(&color_frame_2[0].frame_data.front());
	
		rgbd_inp.map_color_to_depth(depth_frame_2[0], color_frame_2[0], warped_color_frame_2[0]);
		colors = reinterpret_cast<const unsigned char*>(&warped_color_frame_2[0].frame_data.front());
		
		int i = 0;
		for (int y = 0; y < depth_frame_2[0].height; ++y)
			for (int x = 0; x < depth_frame_2[0].width; ++x) {
				vec3 p;
				if (rgbd_inp.map_depth_to_point(x, y, depths[i], &p[0])) {
					// flipping y to make it the same direction as in pixel y coordinate

					p = -p;
					p = rgbd_2_controller_orientation * p + rgbd_2_controller_position;
					p = controller_orientation_pc * p + controller_position_pc;
					rgba8 c(colors[4 * i + 2], colors[4 * i + 1], colors[4 * i], 255);
					vertex v;
					//filter points without color for 32 bit formats
					static const rgba8 filter_color = rgba8(0, 0, 0, 255);
					if (!(c == filter_color)) {
						v.color = c;
						v.point = p;
					}
					intermediate_pc.push_back(v);
				}
				++i;
			}
		return intermediate_pc.size();
	}
	
	


	size_t vr_rgbd:: construct_multi_point_cloud(int index)//
	{
		//for (int index_device = 0; index_device < rgbd_inp.nr_multi_de(); index_device++) {//
			
		//if (index == 0) {
		//intermediate_pc.clear();
		
		//}
			
		const unsigned short* depths = reinterpret_cast<const unsigned short*>(&depth_frame_2[index].frame_data.front());
		const unsigned char* colors = reinterpret_cast<const unsigned char*>(&color_frame_2[index].frame_data.front());

		rgbd_inp.map_color_to_depth(depth_frame_2[index], color_frame_2[index], warped_color_frame_2[index], index);
		colors = reinterpret_cast<const unsigned char*>(&warped_color_frame_2[index].frame_data.front());

		
		int i = 0;
		
		mat3 pc_cam_r = manualcorrect_rotation[index] * cam_fine_r[index] * cam_coarse_r[index];
		vec3 pc_cam_t = manualcorrect_rotation[index] * cam_fine_r[index] * cam_coarse_t[index] + manualcorrect_rotation[index] * cam_fine_t[index] + manualcorrect_translation[index];

		float t;
		vec3 origin = vec3(0, 0, 0);
		intermediate_rgbdpc[index].clear();
		intermediate_rgbdpc_bbox[index].clear();
		intermediate_rgbdpc_outside_bbox[index].clear();
		for (int y =0; y < depth_frame_2[index].height; ++y)
			for (int x = 0; x < depth_frame_2[index].width; ++x) {//
				vec3 p;
				if (rgbd_inp.map_depth_to_point(x, y, depths[i], &p[0], index)) {
					// flipping y to make it the same direction as in pixel y coordinate
							
					rgba8 c(colors[4 * i + 2], colors[4 * i + 1], colors[4 * i], 255);
					//rgba8 c(colors[4 * i + 2], colors[4 * i + 1], colors[4 * i + 0], colors[4 * i + 3]);
					
					vertex v;
					//filter points without color for 32 bit formats
					static const rgba8 filter_color = rgba8(0, 0, 0, 255);
					//static const rgba8 filter_color = rgba8(0, 0, 0, 0);
					
					if (!(c == filter_color)) {
																			
						v.color = c;						
						
						t = p[1];
						p[1] = p[2];
						p[2] = t;

						p = pc_cam_r * p + pc_cam_t;
						
						v.point = p;
						
						if (p[0]<pcbb.pos2[0] && p[1] < pcbb.pos2[1] && p[2] < pcbb.pos2[2] &&
							p[0] > pcbb.pos1[0] && p[1] > pcbb.pos1[1] && p[2] > pcbb.pos1[2]) {
						
							if ((manualcorrectmode&& currentpointcloud == index) || (manualcorrectmode&&currentpointcloud == rgbdpc.size()))
								intermediate_rgbdpc_bbox[index].add_point(v.point, rgba8(255,0,0,255));
							else	
								intermediate_rgbdpc_bbox[index].add_point(v.point, v.color);

						}
						else {
							intermediate_rgbdpc_outside_bbox[index].add_point(v.point, v.color);
						
						}
						if ((manualcorrectmode && currentpointcloud == index) || (manualcorrectmode&&currentpointcloud == rgbdpc.size()))
							intermediate_rgbdpc[index].add_point(v.point, rgba8(255, 0, 0, 255));
						else
							intermediate_rgbdpc[index].add_point(v.point, v.color);
											
					}
						
				}
				++i;
			}

		
		
		intermediate_rgbdpc[index].cam_pos = pc_cam_r * origin + pc_cam_t;
		intermediate_rgbdpc[index].cam_rotation = pc_cam_r;
		intermediate_rgbdpc[index].cam_translation = pc_cam_t;;
		intermediate_rgbdpc_bbox[index].cam_pos = pc_cam_r * origin + pc_cam_t;
		intermediate_rgbdpc_bbox[index].cam_rotation = pc_cam_r;
		intermediate_rgbdpc_bbox[index].cam_translation = pc_cam_t;;

		intermediate_rgbdpc_outside_bbox[index].cam_pos = pc_cam_r * origin + pc_cam_t;
		intermediate_rgbdpc_outside_bbox[index].cam_rotation = pc_cam_r;
		intermediate_rgbdpc_outside_bbox[index].cam_translation = pc_cam_t;;

		return intermediate_rgbdpc[index].get_nr_Points();
		
	}

	void vr_rgbd::save_all_pc()
	{
		for (int i = 1; i < rgbdpc.size(); i++)
		{
			rgbdpc[i].cam_rotation = manualcorrect_rotation[i] * cam_fine_r[i] * cam_coarse_r[i];
			rgbdpc[i].cam_translation = manualcorrect_rotation[i] * cam_fine_r[i] * cam_coarse_t[i] + manualcorrect_rotation[i] * cam_fine_t[i] + manualcorrect_translation[i];

		}
		for (int i = 1; i < rgbdpc_in_box.size(); i++)
		{
			rgbdpc_in_box[i].cam_rotation = manualcorrect_rotation[i] * cam_fine_r[i] * cam_coarse_r[i];
			rgbdpc_in_box[i].cam_translation = manualcorrect_rotation[i] * cam_fine_r[i] * cam_coarse_t[i] + manualcorrect_rotation[i] * cam_fine_t[i] + manualcorrect_translation[i];

		}


		if (rgbdpc.size()==0){
			std::cout<<"no pointcloud in the scene"<<std::endl;
			return;
		}	
		rgbd_pointcloud my_pc;
		if (!setboundingboxmode)
			my_pc=rgbdpc[0];	
		else
			my_pc = rgbdpc_in_box[0];
		if (rgbdpc.size() >= 1) {
			if (!setboundingboxmode) {
			for (int i = 1; i < rgbdpc.size(); i++) {
				for (int j = 0; j < rgbdpc[i].get_nr_Points(); j++) {
				
					my_pc.add_point(rgbdpc[i].pnt(j), rgbdpc[i].clr(j));
				
				}
			}}else
				for (int i = 1; i < rgbdpc_in_box.size(); i++) {
					for (int j = 0; j < rgbdpc_in_box[i].get_nr_Points(); j++) {

						my_pc.add_point(rgbdpc_in_box[i].pnt(j), rgbdpc_in_box[i].clr(j));

					}
				}
					
		}
		
		
		std::string fn = cgv::gui::file_save_dialog("point cloud", "Point Cloud Files (lbypc,ply,bpc,apc,obj):*.txt;*.lbypc");
		//for (int i = 0; i < 10; i++) {
					
		if (fn.empty())
			return;
		FILE* fp = fopen(fn.c_str(), "wb");
		if (!fp)
			return;
		
		my_pc.write_pc(fn);
		
		fclose(fp);
		
		//}
		return;
	}
	void vr_rgbd::Record_PC_FromOneCam(int cam)
	{		
		
		
		std::string fn = data_dir + "\\" + "record" + "\\" +"camera" + to_string(cam+1) + "\\" + to_string(num_recorded_pc) + ".lbypc";

		if (fn.empty())
			return;
		FILE* fp = fopen(fn.c_str(), "wb");
		if (!fp)
			return;
		rgbdpc[cam].write_pc(fn);
		fclose(fp);

	}

	void vr_rgbd::Record_PC_FromAllcams() {
	
		if (rgbdpc.size() == 0) {
			std::cout << "no pointcloud in the scene" << std::endl;
			return;
		}



		rgbd_pointcloud my_pc = rgbdpc[0];

		if (rgbdpc.size() >= 1) {
			for (int i = 1; i < rgbdpc.size(); i++) {
				for (int j = 0; j < rgbdpc[i].get_nr_Points(); j++) {
					my_pc.add_point(rgbdpc[i].pnt(j), rgbdpc[i].clr(j));
				}
			}
		}



		/*std::string fn = cgv::gui::file_save_dialog("point cloud", "Point Cloud Files (lbypc,ply,bpc,apc,obj):*.txt;*.lbypc");*/
		

			std::string fn = data_dir + "\\" + "record" + "\\" + "combined_pc"  + "\\" + to_string(num_recorded_pc) + ".lbypc";

			//std::cout<<"fn:" << fn << std::endl;
			//e.g. fn = D:\by\123.lbypc fn = D:\by\123.txt

			if (fn.empty())
				return;
			FILE* fp = fopen(fn.c_str(), "wb");
			if (!fp)
				return;

			my_pc.write_pc(fn);

			fclose(fp);

		



		return;
	
	}



	void vr_rgbd::load_pc() 
	{
		

		

		std::string fn = cgv::gui::file_open_dialog("source point cloud(*.lbypc;*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt)", "Point cloud files:*.lbypc;*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt;");
		
		if (fn.empty())
			return;
		//clear_current_point_cloud();

		rgbd_pointcloud source_pc;
		source_pc.read_pc(fn);
		
		rgbdpc.push_back(source_pc);
		rgbdpc_in_box.resize(rgbdpc.size());
		rgbdpc_outside_box.resize(rgbdpc.size());
		rgbdpc[rgbdpc.size() - 1].cam_pos = vec3(0, 0, 0);
		rgbdpc_in_box[rgbdpc_in_box.size() - 1].cam_pos = vec3(0, 0, 0);
		rgbdpc_outside_box[rgbdpc_in_box.size() - 1].cam_pos = vec3(0, 0, 0);
		cam_coarse_t.resize(rgbdpc.size());
		cam_coarse_r.resize(rgbdpc.size());

		cam_fine_r.resize(rgbdpc.size());
		cam_fine_t.resize(rgbdpc.size());

		manualcorrect_rotation.resize(rgbdpc.size());
		manualcorrect_translation.resize(rgbdpc.size());

		

		int i = rgbdpc.size() - 1;
		cam_coarse_r[i] = rgbdpc[i].cam_rotation;
		cam_coarse_t[i] = rgbdpc[i].cam_translation;

		cam_fine_r[i].identity();
		cam_fine_t[i] = vec3(0, 0, 0);

		manualcorrect_rotation[i].identity();
		manualcorrect_translation[i] = vec3(0, 0, 0);
		for (int j = 0; j < rgbdpc[i].get_nr_Points(); j++)
		{
			vec3 p = rgbdpc[i].pnt(j);
			if (p[0]<pcbb.pos2[0] && p[1] < pcbb.pos2[1] && p[2] < pcbb.pos2[2] &&
				p[0] > pcbb.pos1[0] && p[1] > pcbb.pos1[1] && p[2] > pcbb.pos1[2])
				rgbdpc_in_box[i].add_point(p, rgbdpc[i].clr(j));
			else
				rgbdpc_outside_box[i].add_point(p, rgbdpc[i].clr(j));
		}
		
		rgbdpc_in_box[i].cam_pos = rgbdpc[i].cam_rotation *vec3(0,0,0)+ rgbdpc[i].cam_translation;
		rgbdpc_in_box[i].cam_rotation = rgbdpc[i].cam_rotation;
		rgbdpc_in_box[i].cam_translation = rgbdpc[i].cam_translation;

		rgbdpc_outside_box[i].cam_pos = rgbdpc[i].cam_rotation *vec3(0, 0, 0) + rgbdpc[i].cam_translation;
		rgbdpc_outside_box[i].cam_rotation = rgbdpc[i].cam_rotation;
		rgbdpc_outside_box[i].cam_translation = rgbdpc[i].cam_translation;

		trees.resize(rgbdpc.size());
		post_redraw();
		
	

		std::cout<< "rgbdpc.size() :"<<rgbdpc .size()<<std::endl;

		//pc_load_dir = fn.substr(0, fn.length() - 8);
		
		
	}
	void vr_rgbd::start_load_recorded_pc() {
		if (rgbd_inp.is_multi_started() || rgbd_inp.is_started()) {
			std::cout << "camera(s) is running!" << std::endl;
			return;
		}
		if (pc_load_dir.size() == 0)
			clear_current_point_cloud();


		rgbdpc.clear();
		rgbdpc_in_box.clear();
		rgbdpc_outside_box.clear();
		rgbdpc.clear();
		cam_coarse_t.clear();
		cam_coarse_r.clear();
		cam_fine_r.clear();
		cam_fine_t.clear();
		manualcorrect_rotation.clear();
		manualcorrect_translation.clear();

		rgbdpc.resize(3);
		rgbdpc_in_box.resize(3);
		rgbdpc_outside_box.resize(3);
		cam_coarse_t.resize(3);
		cam_coarse_r.resize(3);
		cam_fine_r.resize(3);
		cam_fine_t.resize(3);
		manualcorrect_rotation.resize(3);
		manualcorrect_translation.resize(3);
		trees.resize(3);

		for (int i = 0; i < 3; i++) {
		
			std::string fn = data_dir + "\\" + "record" + "\\" + "camera" + to_string(i + 1) + "\\" + to_string(num_recorded_pc) + ".lbypc";
		
			rgbd_pointcloud source_pc;
			source_pc.read_pc(fn);

			rgbdpc[i]=source_pc;	

			for (int j = 0; j < rgbdpc[i].get_nr_Points(); j++)
			{
				vec3 p = rgbdpc[i].pnt(j);
				if (p[0]<pcbb.pos2[0] && p[1] < pcbb.pos2[1] && p[2] < pcbb.pos2[2] &&
					p[0] > pcbb.pos1[0] && p[1] > pcbb.pos1[1] && p[2] > pcbb.pos1[2])
					rgbdpc_in_box[i].add_point(p, rgbdpc[i].clr(j));
				else
					rgbdpc_outside_box[i].add_point(p, rgbdpc[i].clr(j));
			}
			rgbdpc_in_box[i].cam_pos = rgbdpc[i].cam_rotation *vec3(0, 0, 0) + rgbdpc[i].cam_translation;
			rgbdpc_in_box[i].cam_rotation = rgbdpc[i].cam_rotation;
			rgbdpc_in_box[i].cam_translation = rgbdpc[i].cam_translation;
			

			rgbdpc_outside_box[i].cam_pos = rgbdpc[i].cam_rotation *vec3(0, 0, 0) + rgbdpc[i].cam_translation;
			rgbdpc_outside_box[i].cam_rotation = rgbdpc[i].cam_rotation;
			rgbdpc_outside_box[i].cam_translation = rgbdpc[i].cam_translation;
			
			
			cam_coarse_r[i]= rgbdpc[i].cam_rotation;
			cam_coarse_t[i] = rgbdpc[i].cam_translation;
			cam_fine_r[i].identity();
			cam_fine_t[i] = vec3(0, 0, 0);
			manualcorrect_rotation[i].identity();
			manualcorrect_translation[i] = vec3(0, 0, 0);
				
			

			
		
		
		}
		
		std::cout << "rgbdpc.size() :" << rgbdpc.size() << std::endl;
		post_redraw();
	}
	void vr_rgbd::load_recorded_pc() {
		if (rgbd_inp.is_multi_started()) {
			std::cout<<"camera(s) is running!" << std::endl;
			return;
		}
		rgbdpc.clear();
		rgbdpc_in_box.clear();
		rgbdpc_outside_box.clear();
		rgbdpc.clear();
		cam_coarse_t.clear();
		cam_coarse_r.clear();
		cam_fine_r.clear();
		cam_fine_t.clear();
		manualcorrect_rotation.clear();
		manualcorrect_translation.clear();
		if (rgbdpc.size() != 3) {
		rgbdpc.resize(3);
		rgbdpc_in_box.resize(3);
		rgbdpc_outside_box.resize(3);
		cam_coarse_t.resize(3);
		cam_coarse_r.resize(3);
		cam_fine_r.resize(3);
		cam_fine_t.resize(3);
		manualcorrect_rotation.resize(3);
		manualcorrect_translation.resize(3);
		trees.resize(3);}
		for (int i = 0; i < 3; i++) {

			std::string fn = data_dir + "\\" + "record" + "\\" + "camera" + to_string(i + 1) + "\\" + to_string(num_recorded_pc) + ".lbypc";

			rgbd_pointcloud source_pc;
			source_pc.read_pc(fn);

			rgbdpc[i] = source_pc;

			for (int j = 0; j < rgbdpc[i].get_nr_Points(); j++)
			{
				vec3 p = rgbdpc[i].pnt(j);
				if (p[0]<pcbb.pos2[0] && p[1] < pcbb.pos2[1] && p[2] < pcbb.pos2[2] &&
					p[0] > pcbb.pos1[0] && p[1] > pcbb.pos1[1] && p[2] > pcbb.pos1[2])
					rgbdpc_in_box[i].add_point(p, rgbdpc[i].clr(j));
				else
					rgbdpc_outside_box[i].add_point(p, rgbdpc[i].clr(j));
			}
			rgbdpc_in_box[i].cam_pos = rgbdpc[i].cam_rotation *vec3(0, 0, 0) + rgbdpc[i].cam_translation;
			rgbdpc_in_box[i].cam_rotation = rgbdpc[i].cam_rotation;
			rgbdpc_in_box[i].cam_translation = rgbdpc[i].cam_translation;

			rgbdpc_outside_box[i].cam_pos = rgbdpc[i].cam_rotation *vec3(0, 0, 0) + rgbdpc[i].cam_translation;
			rgbdpc_outside_box[i].cam_rotation = rgbdpc[i].cam_rotation;
			rgbdpc_outside_box[i].cam_translation = rgbdpc[i].cam_translation;


			cam_coarse_r[i] = rgbdpc[i].cam_rotation;
			cam_coarse_t[i] = rgbdpc[i].cam_translation;
			cam_fine_r[i].identity();
			cam_fine_t[i] = vec3(0, 0, 0);
			manualcorrect_rotation[i].identity();
			manualcorrect_translation[i] = vec3(0, 0, 0);

		}
		
		post_redraw();
		
		
	
	}

	void vr_rgbd::clear_trajectory()
	{

		Vox->clear_center_pts();
	}

	void vr_rgbd::reset_num_pc()
	{
		num_recorded_pc = 0;
		update_member(&num_recorded_pc);
	}
	void vr_rgbd::load_recorded_PCs()
	{
		clear_current_point_cloud();
		start_load_recorded_pc();
	}
	void vr_rgbd::clear_current_point_cloud() 
	{
		
		generate_pc_from_rgbd = false;
		stop_all_rgbd();
		detach_all_devices();
		current_pc.clear();
		rgbdpc.clear();
		rgbdpc_in_box.clear();
		rgbdpc_outside_box.clear();
		
		post_redraw();
	}
	void vr_rgbd::test1(int a) {
	
	
	}
	
	

	void vr_rgbd::registerPointCloud(rgbd_pointcloud target, rgbd_pointcloud& source, cgv::math::fmat<float, 3, 3>& r, cgv::math::fvec<float, 3>& t,int source_index) {

		
		
		SICP* sicp = new SICP();
		sicp->set_source_cloud(source);
		sicp->set_target_cloud(target);		
		sicp->register_point_to_point(r, t);
		
		vec3 mean = accumulate(&rgbdpc[0].pnt(0), &rgbdpc[0].pnt(0) + rgbdpc[0].get_nr_Points(), vec3(0, 0, 0)) / ((float)rgbdpc[0].get_nr_Points());
		
		//need to de-mean for rotation
		source.do_transformation(-mean);
		//do rotation		
		source.do_transformation(r);
		//do translation and reapply mean
		source.do_transformation(t + mean);
		source.cam_pos -=mean;
		source.cam_pos = r*source.cam_pos   + t + mean;
		cam_fine_t[source_index] = r * cam_fine_t[source_index] +(- r * mean + t+mean);
		cam_fine_r[source_index] = r* cam_fine_r[source_index];
		post_redraw();


	}
	
	void vr_rgbd::build_tree_feature_points(rgbd_pointcloud& pc1,int i) {
		trees[i].reset();
		trees[i] = std::make_shared<ann_tree>();
		trees[i]->build(pc1);
	
	}
	void vr_rgbd::select_feature_points(rgbd_pointcloud& pc1,vec3 p,float radius) {
		
		
		std::vector<int> temp_knn;	
		int NrPointinRadius = trees[currentpointcloud]->find_fixed_radius_points(p, radius, temp_knn);
		//std::cout << "NrPointinRadius :" << NrPointinRadius << std::endl;
		trees[currentpointcloud]->find_closest_points(p, NrPointinRadius, knn);
		//std::cout << "knn size!!!!!!!!!!!!!" << knn.size() << std::endl;

		pc1.merge_labels(knn);
		pc1.set_render_color();

		//std::cout << "labels:" << pc1.labels.size() << std::endl;
				
	}
	void vr_rgbd::cancell_selected_feature_points(rgbd_pointcloud& pc1, vec3 p, float radius)
	{
		std::vector<int> temp_knn;
		int NrPointinRadius = trees[currentpointcloud]->find_fixed_radius_points(p, radius, temp_knn);
		//std::cout << "NrPointinRadius :" << NrPointinRadius << std::endl;
		trees[currentpointcloud]->find_closest_points(p, NrPointinRadius, knn);
		//std::cout << "knn size!!!!!!!!!!!!!" << knn.size() << std::endl;
		pc1.delete_labels(knn);
		pc1.set_render_color();
		
		//std::cout << "labels:" << pc1.labels.size() << std::endl;
	}
	void vr_rgbd::delete_selected_points(rgbd_pointcloud& pc1) {
		pc1.delete_labeled_points();
			
	}

	void vr_rgbd::timer_event(double t, double dt)
	{
		// in case a point cloud is being constructed
		
		if (future_handle.valid()) {
			// check for termination of thread
			if (future_handle.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
				size_t N = future_handle.get();
				// copy computed point cloud								
				current_pc = intermediate_pc;
				post_redraw();
				
			}
		}

		if (rgbd_inp.is_multi_started()) {
		for(int i=0;i<rgbdpc.size();i++)
			if (PCfuture_handle[i].valid() ) {
			// check for termination of thread
					if (PCfuture_handle[i].wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
						size_t N = PCfuture_handle[i].get();
				// copy computed point cloud
						rgbdpc[i] = intermediate_rgbdpc[i];				
						rgbdpc_in_box[i] = intermediate_rgbdpc_bbox[i];
						rgbdpc_outside_box[i] = intermediate_rgbdpc_outside_bbox[i];
						post_redraw();					
				}
		}
		}				

		if (rgbdpc.size() > 2 && (showvolumemode|| showcentermassmode))//				
			voxelize_PC();
		else if (rgbdpc.size() > 2 && showmeshmode)
			generate_mesh();

		if (rgbd_inp.is_multi_started()) {
			for (int i = 0; i < rgbd_inp.nr_multi_de();++i )			
			{
				if (!PCfuture_handle[i].valid())
				{
				//obtain 
					bool new_color_frame_changed = rgbd_inp.get_frame(rgbd::IS_COLOR, color_frame[i], 0, i);		
					bool new_depth_frame_changed = rgbd_inp.get_frame(rgbd::IS_DEPTH, depth_frame[i], 0, i);		
					if (color_frame[i].is_allocated() && depth_frame[i].is_allocated()&&(new_color_frame_changed || new_depth_frame_changed))  //
					{
						color_frame_2[i] = color_frame[i];
						depth_frame_2[i] = depth_frame[i];					
						PCfuture_handle[i] = std::async(&vr_rgbd::construct_multi_point_cloud, this, i);
					
					}	
				}
			}			
								
		}
		else {
							
				if (generate_pc_from_files)//&& mytime==0
				{										
					load_recorded_pc();
					if (num_recorded_pc == 999)
					{
						generate_pc_from_files = false;
						update_member(&generate_pc_from_files);
					}
					else {
					num_recorded_pc++;					
					update_member(&num_recorded_pc);
					
					}
					
				}
			
			else {
				generate_pc_from_files = false;
				update_member(&generate_pc_from_files);
			}
		}

		//record pc from cameras
		if (record_pc_started)
		{
			if (rgbd_inp.is_multi_started()) {
				if (num_recorded_pc == 999)
				{
					for (int i = 0; i < rgbdpc.size(); i++)
					{
						Record_PC_FromOneCam(i);
					}
					record_pc_started = false;
					update_member(&record_pc_started);
				}
				else {

					for (int i = 0; i < rgbdpc.size(); i++)
					{
						Record_PC_FromOneCam(i);
					}
					num_recorded_pc++;

					update_member(&num_recorded_pc);


				}

			}
			else {
				std::cout << "no cameras to record" << std::endl;
				record_pc_started = false;
				update_member(&record_pc_started);
			}
		}
		

}

std::string vr_rgbd::get_type_name() const
{
		return "vr_rgbd";
}
void vr_rgbd::create_gui()
{
	//std::cout<<"================================1231"<<std::endl;

		add_decorator("vr_rgbd", "heading", "level=2");

		unsigned n = rgbd_input::get_nr_devices();
		string device_def = "enums='check advice ID=-2;empty=-1";
		for (unsigned i = 0; i < n; ++i) {
			device_def += ",";
			device_def += rgbd_input::get_serial(i);
		}
		device_def += "'";
		num_devices = n;
		
		connect_copy(add_control("device", (DummyEnum&)device_idx, "dropdown", device_def)->value_change, rebind(this, &vr_rgbd::device_select));		
		add_member_control(this, "currentcamera", currentcamera, "value_slider", "min=-1;max=2;ticks=true");
		add_member_control(this, "color_stream_format", (DummyEnum&)color_stream_format_idx, "dropdown", get_stream_format_enum(color_stream_formats));
		add_member_control(this, "depth_stream_format", (DummyEnum&)depth_stream_format_idx, "dropdown", get_stream_format_enum(depth_stream_formats));				
		add_member_control(this, "attach all devices", all_devices_attached, "check");
		add_member_control(this, "rgbd_multi_started", rgbd_multi_started, "check");
		add_member_control(this, "get_tracker_positions", get_tracker_positions, "check");		
		connect_copy(add_button("save current pointcloud")->click, rebind(this, &vr_rgbd::save_all_pc));
		connect_copy(add_button("load one pointcloud")->click, rebind(this, &vr_rgbd::load_pc));		
		add_member_control(this, "showallpcmode", showallpcmode, "check");
		add_member_control(this, "showPCinBboxmode", showPCinBboxmode, "check");
		add_member_control(this, "showPCoutsideBboxmode", showPCoutsideBboxmode, "check");
		
		add_member_control(this, "showvolumemode", showvolumemode, "check");
		add_member_control(this, "resolution", resolution_bbox, "value_slider", "min=50;max=150;ticks=true");
		add_member_control(this, "range_value_1", range_1, "value_slider", "min=3;max=7;ticks=true");
		add_member_control(this, "threshold_value_1", threshold_1, "value_slider", "min=0;max=100;ticks=true");
		add_member_control(this, "range_value_2", range_2 , "value_slider", "min=3;max=7;ticks=true");
		add_member_control(this, "threshold_value_2", threshold_2, "value_slider", "min=0;max=200;ticks=true");
		add_member_control(this, "showcentermassmode", showcentermassmode, "check");
		connect_copy(add_button("clear trajectory")->click, rebind(this, &vr_rgbd::clear_trajectory));
		add_member_control(this, "showmeshmode", showmeshmode, "check");
		
		connect_copy(add_button("reset num of recorded pc")->click, rebind(this, &vr_rgbd::reset_num_pc));
		connect_copy(add_button("go to current recorded pc")->click, rebind(this, &vr_rgbd::load_recorded_PCs));
		add_member_control(this, "start to record pc", record_pc_started, "check");
		connect_copy(add_control("num_recorded_pc", num_recorded_pc, "value_slider", "min=0;max=999;ticks=true")->value_change, rebind(static_cast<drawable*>(this), &drawable::post_redraw));
		add_member_control(this, "play recorded pc", generate_pc_from_files, "check");
		connect_copy(add_button("clear pc")->click, rebind(this, &vr_rgbd::clear_current_point_cloud));	
		
		add_member_control(this, "ray_length", ray_length, "value_slider", "min=0.1;max=10;log=true;ticks=true");
		bool show = begin_tree_node("points", show_points, true, "w=100;align=' '");
		add_member_control(this, "show", show_points, "toggle", "w=50");
		if (show) {
			align("\a");
			add_member_control(this, "max_nr_shown_recorded_pcs", max_nr_shown_recorded_pcs, "value_slider", "min=0;max=100;log=true;ticks=true");
			//add_member_control(this, "sort_points", sort_points, "check");
			if (begin_tree_node("point style", point_style)) {
				align("\a");
				add_gui("point_style", point_style);
				align("\b");
				end_tree_node(point_style);
			}
			align("\b");
			end_tree_node(show_points);
		}
		if (begin_tree_node("box style", style)) {
			align("\a");
			add_gui("box style", style);
			align("\b");
			end_tree_node(style);
		}
		if (begin_tree_node("movable box style", movable_style)) {
			align("\a");
			add_gui("movable box style", movable_style);
			align("\b");
			end_tree_node(movable_style);
		}
		if (begin_tree_node("intersections", srs)) {
			align("\a");
			add_gui("sphere style", srs);
			align("\b");
			end_tree_node(srs);
		}
		
}
bool vr_rgbd::self_reflect(cgv::reflect::reflection_handler& rh)
{
	return

		rh.reflect_member("ray_length", ray_length) &&

		rh.reflect_member("rgbd_started", rgbd_started) &&
		rh.reflect_member("generate_pc_from_files", generate_pc_from_files) &&       //????????????

		rh.reflect_member("show all PointClouds", showallpcmode) &&
		rh.reflect_member("show PC inside BBox", showPCinBboxmode) &&
		rh.reflect_member("show PC outside Bbox", showPCoutsideBboxmode) &&
		rh.reflect_member("showvolumemode", showvolumemode) &&
		rh.reflect_member("showcentermassmode", showcentermassmode) &&
		rh.reflect_member("showmeshmode", showmeshmode) &&

		rh.reflect_member("rgbd_multi_started", rgbd_multi_started) &&
		rh.reflect_member("all_devices_attached", all_devices_attached) &&
		rh.reflect_member("record_pc_started", record_pc_started) &&


		rh.reflect_member("get_tracker_positions", get_tracker_positions);
}
void vr_rgbd::on_set(void* member_ptr)
{
	
		if (member_ptr == &rgbd_started && rgbd_started != rgbd_inp.is_started()) {
			if (rgbd_started)
			{ 				
				start_rgbd();				
			}				
			else
			{ 				
				stop_rgbd();			
			}				
		}
		
		if (member_ptr == &rgbd_multi_started && rgbd_multi_started != rgbd_inp.is_multi_started())
		{
			
			if (rgbd_multi_started)
			{
				start_multi_rgbd();
			}
			else
			{
				//std::cout<<"run there?"<<std::endl;
				stop_all_rgbd();
				
			}
		}
		if (member_ptr == &all_devices_attached && all_devices_attached != rgbd_inp.is_multi_attached())
		{

			if (all_devices_attached)
			{
				attach_all_devices();
			}
			else
			{				
				detach_all_devices();
			}
		}

		if (member_ptr == &get_tracker_positions)
		{
			set_rgbd_pos();
		}
		if (member_ptr == &generate_pc_from_files)
		{
			if (rgbd_inp.is_multi_started())
				generate_pc_from_files = false;
			update_member(&generate_pc_from_files);
		}


		if (member_ptr == & showallpcmode)
		{
			showPCinBboxmode = false;
			showPCoutsideBboxmode = false;
			update_member(&showPCinBboxmode);
			update_member(&showPCoutsideBboxmode);
		}


		if (member_ptr == &showPCoutsideBboxmode)
		{
			showallpcmode = false;
			update_member(&showallpcmode);
			//std::cout << showallpcmode << std::endl;
			//std::cout << showPCoutsideBboxmode << std::endl;
		}
		if (member_ptr == &showPCinBboxmode)
		{
			showallpcmode = false;
			update_member(&showallpcmode);
		}
		if (member_ptr == &showvolumemode)
		{
			showmeshmode = false;
			showcentermassmode = false;
			update_member(&showmeshmode);
			update_member(&showcentermassmode);
		}

		if (member_ptr == &showmeshmode)
		{
			showvolumemode = false;
			showcentermassmode = false;
			update_member(&showvolumemode);
			update_member(&showcentermassmode);
		}
		if (member_ptr == &showcentermassmode)
		{
			showmeshmode = false;
			showvolumemode = false;
			update_member(&showmeshmode);
			update_member(&showvolumemode);
		}

		update_member(member_ptr);
		post_redraw();
}
void vr_rgbd::stream_help(std::ostream& os)
{
		os << "vr_rgbd: no shortcuts defined" << std::endl;
}


void vr_rgbd::update_stream_formats()
{
	color_stream_formats.clear();
	rgbd_inp.query_stream_formats(IS_COLOR, color_stream_formats);
	if (find_control(color_stream_format_idx))
		find_control(color_stream_format_idx)->multi_set(get_stream_format_enum(color_stream_formats));
	depth_stream_formats.clear();
	rgbd_inp.query_stream_formats(IS_DEPTH, depth_stream_formats);
	if (find_control(depth_stream_format_idx))
		find_control(depth_stream_format_idx)->multi_set(get_stream_format_enum(depth_stream_formats));
	
}






bool vr_rgbd::handle(cgv::gui::event& e)
{
		// check if vr event flag is not set and don't process events in this case
		if ((e.get_flags() & cgv::gui::EF_VR) == 0)
			return false;
		// check event id
		switch (e.get_kind()) {
		case cgv::gui::EID_KEY:
		{
			cgv::gui::vr_key_event& vrke = static_cast<cgv::gui::vr_key_event&>(e);
			int ci = vrke.get_controller_index();
			if (ci == 1 && vrke.get_key() == vr::VR_DPAD_UP) {
				switch (vrke.get_action()) {
				case cgv::gui::KA_PRESS:
					
					if (manualcorrectmode) {
						manualcorrectstarted = true;
					}
					if (setboundingboxmode) {
						setboundingboxstarted = true;
						showvolumemode = false;
						showcentermassmode = false;
						showmeshmode = false;
						update_member(&showvolumemode);
						update_member(&showcentermassmode);
						update_member(&showmeshmode);
					
					
					}
					break;
				case cgv::gui::KA_RELEASE:
					
						manualcorrectstarted = false;
						setboundingboxstarted = false;
					break;
				}
			}
			if (ci == 1 && vrke.get_key() == vr::VR_DPAD_DOWN) {
				switch (vrke.get_action()) {
				case cgv::gui::KA_PRESS :
								
					
					
					break;
				case cgv::gui::KA_RELEASE:
					
					break;
				}
			}
			
			if (ci == 1 && vrke.get_key() == vr::VR_DPAD_LEFT)
			{
				switch (vrke.get_action()) {
				case cgv::gui::KA_PRESS :
					break;
				case cgv::gui::KA_RELEASE:
					break;
				}
			}
			if (ci == 1 && vrke.get_key() == vr::VR_DPAD_RIGHT)
			{
				switch (vrke.get_action()) {
				case cgv::gui::KA_PRESS:

					break;
				case cgv::gui::KA_RELEASE:
					break;
				}
			}
			if (ci == 0 && vrke.get_key() == vr::VR_DPAD_UP)
			{
				switch (vrke.get_action()) {
				case cgv::gui::KA_PRESS:
					currentpointcloud = (currentpointcloud + 1) % (rgbdpc.size() + 1);
					
										
					break;
				case cgv::gui::KA_RELEASE:
					
					break;
				}
			}
			if (ci == 0 && vrke.get_key() == vr::VR_DPAD_DOWN)
			{
				switch (vrke.get_action()) {
				case cgv::gui::KA_PRESS:
					currentpointcloud = (currentpointcloud + rgbdpc.size()) % (rgbdpc.size() + 1);
					
					
					break;
				case cgv::gui::KA_RELEASE:
					
					break;
				}
			}

			if (ci == 0 && vrke.get_key() == vr::VR_DPAD_RIGHT)
			{
				switch (vrke.get_action()) {
				case cgv::gui::KA_PRESS:
					
					
					break;
				case cgv::gui::KA_RELEASE:

					break;
				}
			}
			if (ci == 0 && vrke.get_key() == vr::VR_DPAD_LEFT)
			{
				switch (vrke.get_action()) {
				case cgv::gui::KA_PRESS:
				{
					
				}
					break;
				case cgv::gui::KA_RELEASE:

					break;
				}

			}

			if (ci == 0 && vrke.get_key() == vr::VR_MENU)
			{
				switch (vrke.get_action()) {
				case cgv::gui::KA_PRESS:
				{
					manualcorrectmode = false;
					if (setboundingboxmode == false) {
						setboundingboxmode = true;	
						showvolumemode = false;
						showcentermassmode = false;
						showmeshmode = false;
						update_member(&showvolumemode);
						update_member(&showcentermassmode);
						update_member(&showmeshmode);
					}
				}
				break;
			
				case cgv::gui::KA_RELEASE:

					break;
				}
			}
			if (ci == 1 && vrke.get_key() == vr::VR_MENU)
			{
				switch (vrke.get_action()) {
				case cgv::gui::KA_PRESS:
				{
					setboundingboxmode = false;
					if (manualcorrectmode == false) 
						manualcorrectmode = true;
					else
						manualcorrectmode = false;
														
				}
					break;
				case cgv::gui::KA_RELEASE:
					
					break;
				}
			}
			return true;
		}
		case cgv::gui::EID_THROTTLE:
		{
			cgv::gui::vr_throttle_event& vrte = static_cast<cgv::gui::vr_throttle_event&>(e);
			if ((vrte.get_last_value() > 0.5f) != (vrte.get_value() > 0.5f)) {
							
			}
			break;
		}
		case cgv::gui::EID_POSE:
			cgv::gui::vr_pose_event& vrpe = static_cast<cgv::gui::vr_pose_event&>(e);
			
			// check for controller pose events
			int ci = vrpe.get_trackable_index();
			
	
			//std::cout<<"ci:"<<ci<<std::endl;


			if ( ci == 2)
			{

				if (rgbdpc.size() > 1 && currentcamera >= 0)
				{

				cam_coarse_t [currentcamera]= vrpe.get_position();
				cam_coarse_r[currentcamera] = vrpe.get_orientation();

				}				
				if (rgbdpc.size() == 1 && currentcamera >= 0)
				{
					cam_coarse_t[currentcamera] = vrpe.get_position();
					cam_coarse_r[currentcamera] = vrpe.get_orientation();
				}
				
			}
			if (ci == 1 && manualcorrectstarted) {
				vec3 last_pos = vrpe.get_last_position();
				vec3 pos = vrpe.get_position();
				mat3 rotation = vrpe.get_rotation_matrix();
				//vec3 new_intersection = origin + intersection_offsets[i] * direction;
				vec3 translation = pos - last_pos;

				mat3 controllerRotation = vrpe.get_rotation_matrix();
				if (rgbdpc.size() != 0) {
					if (currentpointcloud == rgbdpc.size())
					{
						for (int i = 0; i < rgbdpc.size(); i++)
						{
							manualcorrect_translation[i] = controllerRotation * manualcorrect_translation[i] - controllerRotation * last_pos + pos;
							manualcorrect_rotation[i] = controllerRotation * manualcorrect_rotation[i];

							if (rgbd_inp.is_multi_attached() && !rgbd_inp.is_multi_started())
							{
								for (int k = 0; k < rgbdpc[i].get_nr_Points(); k++) {
									rgbdpc[i].pnt(k) = controllerRotation * (rgbdpc[i].pnt(k) - last_pos) + pos;
								
								}
							}
						}

					}
					else {
					manualcorrect_translation[currentpointcloud]= controllerRotation * manualcorrect_translation[currentpointcloud] -controllerRotation * last_pos +pos;
					manualcorrect_rotation[currentpointcloud]= controllerRotation* manualcorrect_rotation[currentpointcloud];

					if (rgbd_inp.is_multi_attached() && !rgbd_inp.is_multi_started())
					{
						for (int k = 0; k < rgbdpc[currentpointcloud].get_nr_Points(); k++) {
							rgbdpc[currentpointcloud].pnt(k) = controllerRotation * (rgbdpc[currentpointcloud].pnt(k) - last_pos) + pos;

						}
					}


					}
				}

				post_redraw();
			}
			else if (ci == 1 && setboundingboxstarted) {
			
				vec3 last_pos = vrpe.get_last_position();
				vec3 pos = vrpe.get_position();
				vec3 translation = pos - last_pos;
				vec3 box_center = (pcbb.pos1 + pcbb.pos2) / 2;
				box_center = box_center + translation;
				vec3 pos1, pos2;
				pos1 = box_center - vec3(BoundingBoxlength / 2, BoundingBoxlength / 2, BoundingBoxheight / 2);
				pos2 = box_center + vec3(BoundingBoxlength / 2, BoundingBoxlength / 2, BoundingBoxheight / 2);				
				pcbb.pos1 = pos1;
				pcbb.pos2 = pos2;
				//pcbb.step = BoundingBoxstep;
			
			
			
			}

			if (ci != -1) {
				vec3 origin, direction;
				vrpe.get_state().controller[ci].put_ray(&origin(0), &direction(0));
				if (state[ci] == IS_GRAB) {
					
					for (size_t i = 0; i < intersection_points.size(); ++i) {
						if (intersection_controller_indices[i] != ci)
							continue;
						

					}
				}
				else {// not grab
					// clear intersections of current controller 
					size_t i = 0;
					while (i < intersection_points.size()) {
						if (intersection_controller_indices[i] == ci) {
							intersection_points.erase(intersection_points.begin() + i);
							intersection_colors.erase(intersection_colors.begin() + i);
							intersection_box_indices.erase(intersection_box_indices.begin() + i);
							intersection_controller_indices.erase(intersection_controller_indices.begin() + i);
						}
						else
							++i;
					}

					// compute intersections
					vec3 origin, direction;
					vrpe.get_state().controller[ci].put_ray(&origin(0), &direction(0));
					compute_intersections(origin, direction, ci, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));

					// update state based on whether we have found at least 
					// one intersection with controller ray
					if (intersection_points.size() == i)
						state[ci] = IS_NONE;
					else
						if (state[ci] == IS_NONE)
							state[ci] = IS_OVER;
				}
				post_redraw();
			}
			return true;
		}
		return false;
}


bool vr_rgbd::init(cgv::render::context& ctx)
{
		//ctx.set_bg_color(0.7, 0.7, 0.8, 1.0);
	
		cgv::render::ref_point_renderer(ctx, 1);
		cgv::render::ref_box_renderer(ctx, 1);
		cgv::render::ref_sphere_renderer(ctx, 1);

		if (!cgv::utils::has_option("NO_OPENVR"))
			ctx.set_gamma(1.0f);
		cgv::gui::connect_vr_server(true);

		auto view_ptr = find_view_as_node();
		if (view_ptr) {
			view_ptr->set_eye_keep_view_angle(dvec3(0, 4, -4));
			// if the view points to a vr_view_interactor
			vr_view_ptr = dynamic_cast<vr_view_interactor*>(view_ptr);
			if (vr_view_ptr) {
				// configure vr event processing
				vr_view_ptr->set_event_type_flags(
					cgv::gui::VREventTypeFlags(
						cgv::gui::VRE_KEY +
						cgv::gui::VRE_ONE_AXIS +
						cgv::gui::VRE_ONE_AXIS_GENERATES_KEY +
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

		if (!sky_prog.is_created()) {
			sky_prog.build_program(ctx, "glsl/sky.glpr");
			img_tex.create_from_images(ctx, data_dir + "/skybox/cm_{xp,xn,yp,yn,zp,zn}.jpg");//
			
		}
		
		Vox->init_voxelization(ctx);
		
		return true;		
}
void vr_rgbd::clear(cgv::render::context& ctx)
{
		cgv::render::ref_point_renderer(ctx, -1);
		cgv::render::ref_box_renderer(ctx, -1);
		cgv::render::ref_sphere_renderer(ctx, -1);
		sky_prog.destruct(ctx);
}
rgbd_pointcloud vr_rgbd::setboundingbox(rgbd_pointcloud pc1, vec3 pos1, vec3 pos2) //pos1 <pos2
{
	rgbd_pointcloud pc2;
	//pc2.clear();
	for (int i = 0; i < pc1.get_nr_Points(); i++)
	{
		if (pc1.pnt(i)[0] > pos1[0] && pc1.pnt(i)[0] < pos2[0])
			if (pc1.pnt(i)[1] > pos1[1] && pc1.pnt(i)[1] < pos2[1])
				if (pc1.pnt(i)[2] > pos1[2] && pc1.pnt(i)[2] < pos2[2])
					pc2.add_point(pc1.pnt(i), pc1.clr(i));
	}

	return  pc2;
}
void vr_rgbd::draw_grid(cgv::render::context& ctx, vec3 min, vec3 max, float voxelsize) {
	std::vector<vec3> P;
	std::vector<rgb> C;
	rgb c(0.5, 0.5, 0.5);
	uvec3 num_id = ceil((max - min) / float(voxelsize));
	for (int i = 0; i <= num_id[0]; i++)
	{
		for (int j = 0; j <= num_id[1]; j++)
		{
			P.push_back(min + vec3(i*voxelsize, j*voxelsize, 0));
			P.push_back(min + vec3(i*voxelsize, j*voxelsize, num_id[2] * voxelsize));
			C.push_back(c);
			C.push_back(c);
		}
	}
	for (int i = 0; i <= num_id[0]; i++)
	{
		for (int j = 0; j <= num_id[2]; j++)
		{
			P.push_back(min + vec3(i*voxelsize,  0,j*voxelsize));
			P.push_back(min + vec3(i*voxelsize, num_id[1] * voxelsize, j*voxelsize));
			C.push_back(c);
			C.push_back(c);
		}
	}
	for (int i = 0; i <= num_id[1]; i++)
	{
		for (int j = 0; j <= num_id[2]; j++)
		{
			P.push_back(min + vec3(0,i*voxelsize, j*voxelsize ));
			P.push_back(min + vec3(num_id[0] * voxelsize,i*voxelsize, j*voxelsize ));
			C.push_back(c);
			C.push_back(c);
		}
	}

	
	cgv::render::shader_program& prog = ctx.ref_default_shader_program();
	int pi = prog.get_position_index();
	int ci = prog.get_color_index();
	cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
	cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
	cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ci, C);
	cgv::render::attribute_array_binding::enable_global_array(ctx, ci);
	glLineWidth(0.1);
	prog.enable(ctx);
	glDrawArrays(GL_LINES, 0, (GLsizei)P.size());
	prog.disable(ctx);
	cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
	cgv::render::attribute_array_binding::disable_global_array(ctx, ci);
	//glLineWidth(1);

}


void vr_rgbd::draw_boudingbox(cgv::render::context& ctx, vec3& pos1, vec3& pos2)
{
	std::vector<vec3> P;
	std::vector<rgb> C;
	P.push_back(pos1);
	P.push_back(pos1 + vec3(pos2[0]- pos1[0], 0, 0));
	P.push_back(pos1);
	P.push_back(pos1 + vec3(0, pos2[1] - pos1[1], 0));
	P.push_back(pos1);
	P.push_back(pos1 + vec3(0, 0, pos2[2] - pos1[2]));

	P.push_back(P[1]);
	P.push_back(P[1] + vec3(0, pos2[1] - pos1[1], 0));
	P.push_back(P[1]);
	P.push_back(P[1] + vec3(0, 0, pos2[2]-pos1[2]));

	P.push_back(P[3]);
	P.push_back(P[3] + vec3(pos2[0] - pos1[0], 0, 0));
	P.push_back(P[3]);
	P.push_back(P[3] + vec3(0, 0, pos2[2] - pos1[2]));

	P.push_back(P[5]);
	P.push_back(P[5] + vec3(pos2[0] - pos1[0], 0, 0));
	P.push_back(P[5]);
	P.push_back(P[5] + vec3(0, pos2[1] - pos1[1], 0));

	P.push_back(P[7]);
	P.push_back(pos2);
	P.push_back(P[9]);
	P.push_back(pos2);
	P.push_back(P[13]);
	P.push_back(pos2);
	
	rgb c;//(0.5, 0.5, 0.5)
	if (!setboundingboxmode)
		c = rgb(0.5, 0.5, 0.5);
	else
		c = rgb(1.0, 0, 0);
	for (int i = 0; i <= 24; i++)
		C.push_back(c);
	cgv::render::shader_program& prog = ctx.ref_default_shader_program();
	int pi = prog.get_position_index();
	int ci = prog.get_color_index();
	cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
	cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
	cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ci, C);
	cgv::render::attribute_array_binding::enable_global_array(ctx, ci);
	glLineWidth(3);
	prog.enable(ctx);
	glDrawArrays(GL_LINES, 0, (GLsizei)P.size());
	prog.disable(ctx);
	cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
	cgv::render::attribute_array_binding::disable_global_array(ctx, ci);
	glLineWidth(1);


	//// draw static boxes
	//	cgv::render::surface_renderer& renderer = cgv::render::ref_box_renderer(ctx);
	//	renderer.set_render_style(style);
	//	renderer.set_box_array(ctx, boxes);
	//	renderer.set_color_array(ctx, box_colors);
	//	if (renderer.validate_and_enable(ctx)) {
	//		glDrawArrays(GL_POINTS, 0, (GLsizei)boxes.size());
	//	}
	//	renderer.disable(ctx);


}


void vr_rgbd::draw_pc(cgv::render::context& ctx, const std::vector<vertex>& pc)
{
		
		if (pc.empty())
			return;
		auto& pr = cgv::render::ref_point_renderer(ctx);
		pr.set_render_style(point_style);
		pr.set_position_array(ctx, &pc.front().point, pc.size(), sizeof(vertex));
		pr.set_color_array(ctx, &pc.front().color, pc.size(), sizeof(vertex));
		if (pr.validate_and_enable(ctx)) {
			glDrawArrays(GL_POINTS, 0, (GLsizei)pc.size());
			pr.disable(ctx);
		}

}

void vr_rgbd::draw_rgbdpc(cgv::render::context& ctx, const rgbd_pointcloud& pc){
	if (pc.get_nr_Points()==0)
		return;
	auto& pr = cgv::render::ref_point_renderer(ctx);
	pr.set_render_style(point_style);
	pr.set_position_array(ctx, pc.getPoints());
	pr.set_color_array(ctx, pc.getColors());
	if (pr.validate_and_enable(ctx)) {
		glDrawArrays(GL_POINTS, 0, (GLsizei)pc.get_nr_Points());
		pr.disable(ctx);
	}

}
void vr_rgbd::draw_selected_rgbdpc(cgv::render::context& ctx, const rgbd_pointcloud& pc) {
	if (pc.get_nr_Points() == 0)
		return;
	auto& pr = cgv::render::ref_point_renderer(ctx);
	pr.set_render_style(point_style);
	pr.set_position_array(ctx, pc.getPoints());

	std::vector<Rgba> myrenderColors;
	myrenderColors.resize(pc.get_nr_Points(), Rgba(255, 0, 0, 255));
	pr.set_color_array(ctx, myrenderColors);
	if (pr.validate_and_enable(ctx)) {
		glDrawArrays(GL_POINTS, 0, (GLsizei)pc.get_nr_Points());
		pr.disable(ctx);
	}

}

void vr_rgbd::draw_viewingcone(cgv::render::context& ctx, int index, std::vector<vec3>& P, std::vector<rgb>& C,mat3 r,vec3 t) {
	vec3 a = vec3(0.275, 0.25, 0.2375);
	vec3 b = vec3(0.275, 0.25, -0.1525);
	vec3 c = vec3(-0.2875, 0.25, -0.1525);
	vec3 d = vec3(-0.2875, 0.25, 0.2375);
	vec3 e = vec3(0, 0, 0);
	
	a = r * a + t;
	b = r * b + t;
	c = r * c + t;
	d = r * d + t;
	e = r * e + t;
	P.push_back(a);
	P.push_back(b);
	P.push_back(b);
	P.push_back(c);
	P.push_back(c);
	P.push_back(d);
	P.push_back(d);
	P.push_back(a);

	P.push_back(a);
	P.push_back(e);
	P.push_back(b);
	P.push_back(e);
	P.push_back(c);
	P.push_back(e);
	P.push_back(d);
	P.push_back(e);

	C.push_back(rgb(0, 1, 1));
	C.push_back(rgb(0, 1, 1));
	C.push_back(rgb(0, 1, 1));
	C.push_back(rgb(0, 1, 1));
	C.push_back(rgb(0, 1, 1));
	C.push_back(rgb(0, 1, 1));
	C.push_back(rgb(0, 1, 1));
	C.push_back(rgb(0, 1, 1));
	C.push_back(rgb(0, 1, 1));
	C.push_back(rgb(0, 1, 1));
	C.push_back(rgb(0, 1, 1));
	C.push_back(rgb(0, 1, 1));
	C.push_back(rgb(0, 1, 1));
	C.push_back(rgb(0, 1, 1));
	C.push_back(rgb(0, 1, 1));
	C.push_back(rgb(0, 1, 1));

}




void vr_rgbd::draw(cgv::render::context& ctx)
{
	
	draw_boudingbox(ctx, pcbb.pos1,pcbb.pos2);
			
		if (vr_view_ptr) {
			std::vector<vec3> P;
			std::vector<rgb> C;
			const vr::vr_kit_state* state_ptr = vr_view_ptr->get_current_vr_state();

			if (rgbdpc.size() != 0) {
				if (currentcamera == -1) {
					for (int i = 0; i < rgbdpc.size(); i++) {
						mat3 r_viewing = manualcorrect_rotation[i] * cam_fine_r[i] * cam_coarse_r[i];
						vec3 t_viewing = manualcorrect_rotation[i] * cam_fine_r[i] * cam_coarse_t[i] + manualcorrect_rotation[i] * cam_fine_t[i] + manualcorrect_translation[i];
						//draw_viewingcone(ctx, i, P, C, r_viewing, t_viewing);
					}
				}
				else {
					
					mat3 r_viewing = manualcorrect_rotation[currentcamera] * cam_fine_r[currentcamera] * cam_coarse_r[currentcamera];
					vec3 t_viewing = manualcorrect_rotation[currentcamera] * cam_fine_r[currentcamera] * cam_coarse_t[currentcamera] + manualcorrect_rotation[currentcamera] * cam_fine_t[currentcamera] + manualcorrect_translation[currentcamera];
					//draw_viewingcone(ctx, currentcamera, P, C, r_viewing, t_viewing);
				}
			}
			if (state_ptr) {
				 
				if (setboundingboxmode) {
				
				}
				else {
			
					for (int ci = 0; ci < 2; ++ci) if (state_ptr->controller[ci].status == vr::VRS_TRACKED) {
						vec3 ray_origin, ray_direction;
						state_ptr->controller[ci].put_ray(&ray_origin(0), &ray_direction(0));
						P.push_back(ray_origin);
						P.push_back(ray_origin + ray_length * ray_direction);
						rgb c(float(1 - ci), 0.5f * (int)state[ci], float(ci));
						C.push_back(c);
						C.push_back(c);
					}
				}

				//drawing the tracker pose
				for (int ci = 2; ci < 5; ++ci) if (state_ptr->controller[ci].status == vr::VRS_TRACKED) {
					vec3 ray_origin, ray_direction;
					state_ptr->controller[ci].put_ray(&ray_origin(0), &ray_direction(0));
					P.push_back(ray_origin);
					vec3 new_ray;
					new_ray[0] = state_ptr->controller[ci].pose[3];
					new_ray[1] = state_ptr->controller[ci].pose[4];
					new_ray[2] = state_ptr->controller[ci].pose[5];
					P.push_back(ray_origin + ray_length * new_ray);
					rgb c(float(1 - ci), 0.5f * (int)state[ci], float(ci));
					C.push_back(c);
					C.push_back(c);
				}

			}
			if (P.size() > 0) {
				cgv::render::shader_program& prog = ctx.ref_default_shader_program();
				int pi = prog.get_position_index();
				int ci = prog.get_color_index();
				cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
				cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
				cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ci, C);
				cgv::render::attribute_array_binding::enable_global_array(ctx, ci);
				glLineWidth(3);
				prog.enable(ctx);
				glDrawArrays(GL_LINES, 0, (GLsizei)P.size());
				prog.disable(ctx);
				cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
				cgv::render::attribute_array_binding::disable_global_array(ctx, ci);
				glLineWidth(1);
			}
		}

		if (showPCinBboxmode|| showPCoutsideBboxmode) {
			if (showPCinBboxmode) {
			if (rgbdpc_in_box.size() != 0) {
				for (int i = 0; i < rgbdpc_in_box.size(); i++) {
					
					
					if (rgbd_inp.is_multi_attached() && !rgbd_inp.is_multi_started()) {
						if(currentpointcloud==rgbdpc.size())						
							draw_selected_rgbdpc(ctx, rgbdpc_in_box[i]);
						else if(currentpointcloud == i)
							draw_selected_rgbdpc(ctx, rgbdpc_in_box[i]);
						else
							draw_rgbdpc(ctx, rgbdpc_in_box[i]);
					}
					else {
						draw_rgbdpc(ctx, rgbdpc_in_box[i]);		
					}
				}
				

				}
			}
			if (showPCoutsideBboxmode)
			{
				if (rgbdpc_outside_box.size() != 0) {
					for (int i = 0; i < rgbdpc_outside_box.size(); i++) {
						if (rgbd_inp.is_multi_attached() && !rgbd_inp.is_multi_started())
						{
							if (currentpointcloud == rgbdpc.size())
								draw_selected_rgbdpc(ctx, rgbdpc_outside_box[i]);
							else if (currentpointcloud == i)
								draw_selected_rgbdpc(ctx, rgbdpc_outside_box[i]);
							else
								draw_rgbdpc(ctx, rgbdpc_outside_box[i]);
						}
						else
							draw_rgbdpc(ctx, rgbdpc_outside_box[i]);

					}
				}
			}
		}		
		else if(showallpcmode)
		{
			if (rgbdpc.size() != 0) {
	
				for (int i = 0; i < rgbdpc.size(); i++)
				{	
					if (rgbd_inp.is_multi_attached() && !rgbd_inp.is_multi_started()) {
						if (currentpointcloud == rgbdpc.size())
							draw_selected_rgbdpc(ctx, rgbdpc[i]);
						else if (currentpointcloud == i)
							draw_selected_rgbdpc(ctx, rgbdpc[i]);
						else
							draw_rgbdpc(ctx, rgbdpc[i]);
					}
					else
						draw_rgbdpc(ctx, rgbdpc[i]);
							
				}
				
			}
		}
		if (showvolumemode || showcentermassmode)
		{

			Vox->draw_voxels(ctx, showvolumemode);
			
			
		}
		else if(showmeshmode){
		
			Vox->drawmesh(ctx);
		}
		
		float max_scene_extent = 100;
		if (sky_prog.is_created()) {
			glDepthMask(GL_FALSE);
			glDisable(GL_CULL_FACE);
			img_tex.enable(ctx, 1);
			sky_prog.enable(ctx);
			sky_prog.set_uniform(ctx, "img_tex", 1);
			ctx.push_modelview_matrix();
			ctx.mul_modelview_matrix(cgv::math::scale4<double>(
				max_scene_extent, max_scene_extent, max_scene_extent));
			ctx.tesselate_unit_cube();
			ctx.pop_modelview_matrix();
			sky_prog.disable(ctx);
			img_tex.disable(ctx);
			glEnable(GL_CULL_FACE);
			glDepthMask(GL_TRUE);
		}
		
		// draw static boxes
		cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
		renderer.set_render_style(style);
		renderer.set_box_array(ctx, boxes);
		renderer.set_color_array(ctx, box_colors);
		if (renderer.validate_and_enable(ctx)) {
			glDrawArrays(GL_POINTS, 0, (GLsizei)boxes.size());
		}
		renderer.disable(ctx);





		
		
	
}


/// construct boxes that represent a table of dimensions tw,td,th and leg width tW
void vr_rgbd::construct_table(float tw, float td, float th, float tW)
{
	// construct table
	rgb table_clr(0.3f, 0.2f, 0.0f);
	boxes.push_back(box3(
		vec3(-0.5f*tw - 2*tW, th, -0.5f*td - 2*tW), 
		vec3( 0.5f*tw + 2*tW, th + tW, 0.5f*td + 2*tW)));
	box_colors.push_back(table_clr);

	boxes.push_back(box3(vec3(-0.5f*tw, 0, -0.5f*td), vec3(-0.5f*tw - tW, th, -0.5f*td - tW)));
	boxes.push_back(box3(vec3(-0.5f*tw, 0, 0.5f*td), vec3(-0.5f*tw - tW, th, 0.5f*td + tW)));
	boxes.push_back(box3(vec3(0.5f*tw, 0, -0.5f*td), vec3(0.5f*tw + tW, th, -0.5f*td - tW)));
	boxes.push_back(box3(vec3(0.5f*tw, 0, 0.5f*td), vec3(0.5f*tw + tW, th, 0.5f*td + tW)));
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
}
/// construct boxes that represent a room of dimensions w,d,h and wall width W
void vr_rgbd::construct_room(float w, float d, float h, float W, bool walls, bool ceiling)
{
	// construct floor
	boxes.push_back(box3(vec3(-0.5f*w, -W, -0.5f*d), vec3(0.5f*w, 0, 0.5f*d)));
	box_colors.push_back(rgb(0.2f, 0.2f, 0.2f));

	if (walls) {
		// construct walls
		boxes.push_back(box3(vec3(-0.5f*w, -W, -0.5f*d - W), vec3(0.5f*w, h, -0.5f*d)));
		box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));
		boxes.push_back(box3(vec3(-0.5f*w, -W, 0.5f*d), vec3(0.5f*w, h, 0.5f*d + W)));
		box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));

		boxes.push_back(box3(vec3(0.5f*w, -W, -0.5f*d - W), vec3(0.5f*w + W, h, 0.5f*d + W)));
		box_colors.push_back(rgb(0.5f, 0.8f, 0.5f));
	}
	if (ceiling) {
		// construct ceiling
		boxes.push_back(box3(vec3(-0.5f*w - W, h, -0.5f*d - W), vec3(0.5f*w + W, h + W, 0.5f*d + W)));
		box_colors.push_back(rgb(0.5f, 0.5f, 0.8f));
	}
}

/// construct boxes for environment
void vr_rgbd::construct_environment(float s, float ew, float ed, float eh, float w, float d, float h)
{
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	unsigned n = unsigned(ew / s);
	unsigned m = unsigned(ed / s);
	for (unsigned i = 0; i < n; ++i) {
		float x = i * s - 0.5f*ew;
		for (unsigned j = 0; j < m; ++j) {
			float z = j * s - 0.5f*ed;
			if ( (x + s > -0.5f*w && x < 0.5f*w) && (z + s > -0.5f*d && z < 0.5f*d) )
				continue;
			float h = 0.2f*(std::max(abs(x)-0.5f*w,0.0f)+std::max(abs(z)-0.5f*d,0.0f))*distribution(generator)+0.1f;
			boxes.push_back(box3(vec3(x, 0, z), vec3(x+s, h, z+s)));
			box_colors.push_back(
				rgb(0.3f*distribution(generator)+0.3f, 
					0.3f*distribution(generator)+0.2f, 
					0.2f*distribution(generator)+0.1f));
		}
	}
}

/// construct boxes that can be moved around
void vr_rgbd::construct_movable_boxes(float tw, float td, float th, float tW, size_t nr)
{
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	std::uniform_real_distribution<float> signed_distribution(-1, 1);
	for (size_t i = 0; i < nr; ++i) {
		float x = distribution(generator);
		float y = distribution(generator);
		vec3 extent(distribution(generator), distribution(generator), distribution(generator));
		extent += 0.1f;
		extent *= std::min(tw, td)*0.2f;

		vec3 center(-0.5f*tw + x * tw, th + tW, -0.5f*td + y * td);
		movable_boxes.push_back(box3(-0.5f*extent, 0.5f*extent));
		movable_box_colors.push_back(rgb(distribution(generator), distribution(generator), distribution(generator)));
		movable_box_translations.push_back(center);
		quat rot(signed_distribution(generator), signed_distribution(generator), signed_distribution(generator), signed_distribution(generator));
		rot.normalize();
		movable_box_rotations.push_back(rot);
	}
}

void vr_rgbd::device_select() {
	if (device_idx == -1)
		device_mode = Protocol;
	else if (device_idx == -2)
		device_mode = No_Device;
	else device_mode = Has_Device;
	//std::cout<<"device_id:"<< device_idx <<std::endl;
	if (device_mode == Has_Device) {
		unsigned nr = rgbd_input::get_nr_devices();
		// if the number of device is zero
		if (nr == 0) {
			device_mode = No_Device;
			update_member(&device_mode);
		}
		else {
		
		}
	
	}


}





#include <cgv/base/register.h>
//cgv::base::object_registration<vr_rgbd> vr_rgbd_reg("vr_rgbd");
cgv::base::object_registration<vr_rgbd> vr_rgbd_reg("");
///@}