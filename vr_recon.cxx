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
		construct_room(w, d, h, W, false, false);
		construct_table(tw, td, th, tW);
		construct_environment(0.2f, 3 * w, 3 * d, h, w, d, h);
		construct_movable_boxes(tw, td, th, tW, 20);
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
	//cur_pc.resize(rgbd_inp.get_nr_devices());
	rgbdpc.resize(rgbd_inp.get_nr_devices());
	rgbdpc_in_box.resize(rgbd_inp.get_nr_devices());

	//viewconepos1.resize(rgbd_inp.get_nr_devices());
	//viewconepos2.resize(rgbd_inp.get_nr_devices());

	cam_fine_r.resize(rgbd_inp.get_nr_devices());
	cam_fine_t.resize(rgbd_inp.get_nr_devices());

	cam_coarse_t.resize(rgbd_inp.get_nr_devices());
	cam_coarse_r.resize(rgbd_inp.get_nr_devices());
	manualcorrect_translation.resize(rgbd_inp.get_nr_devices());
	manualcorrect_rotation.resize(rgbd_inp.get_nr_devices());
	trees.resize(rgbd_inp.get_nr_devices());

	imageplanes.resize(rgbd_inp.get_nr_devices());
	for (int i = 0; i < cam_fine_r.size(); i++) {
		cam_coarse_r[i].identity();
		cam_coarse_t[i] = vec3(0, 0, 0);

		cam_fine_r[i].identity();
		cam_fine_t[i] = vec3(0, 0, 0);

		manualcorrect_rotation[i].identity();
		manualcorrect_translation[i] = vec3(0, 0, 0);
		rgbdpc[i].cam_pos = vec3(0, 0, 0);

		imageplanes[i].resize(576);
		for (int j = 0; j < 576; j++)
			imageplanes[i][j].resize(640);
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
		imageplanes.clear();
		rgbdpc_in_box.clear();
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





	//rgbd_inp.set_near_mode(near_mode);                                                //???
	//InputStreams is = IS_NONE;
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

	/*if (ir_stream_format_idx == -1)
		use_default = true;
	else
		sfs.push_back(ir_stream_formats[ir_stream_format_idx]);*/

	//if (use_default) {
	//	/*sfs.clear();
	//	if (!rgbd_inp.start(InputStreams(is), sfs)) {
	//		cgv::gui::message("could not start kinect device");
	//		return;
	//	}
	//	else {*/
	//		for (const auto& sf : sfs) {
	//			auto ci = std::find(color_stream_formats.begin(), color_stream_formats.end(), sf);
	//			if (ci != color_stream_formats.end()) {
	//				color_stream_format_idx = ci - color_stream_formats.begin();
	//				update_member(&color_stream_format_idx);
	//			}
	//			auto di = std::find(depth_stream_formats.begin(), depth_stream_formats.end(), sf);
	//			if (di != depth_stream_formats.end()) {
	//				depth_stream_format_idx = di - depth_stream_formats.begin();
	//				update_member(&depth_stream_format_idx);
	//			}
	//			auto ii = std::find(ir_stream_formats.begin(), ir_stream_formats.end(), sf);
	//			if (ii != ir_stream_formats.end()) {
	//				ir_stream_format_idx = ii - ir_stream_formats.begin();
	//				update_member(&ir_stream_format_idx);
	//			}
	//		}
	//	//}
	//
	//}
	
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
	rgbd_controller_index = 0;
	controller_orientation.identity();
	controller_position = vec3(0, 1.5f, 0);
	in_calibration = false;
	zoom_in = false;
	zoom_out = false;
	save_pointcloud = true;
	registration_started = false;
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
	record_frame = false;
	record_all_frames = false;
	record_key_frames = false;
	clear_all_frames = false;
	trigger_is_pressed = false;
	recording_fps = 5;
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
	setboundingboxmode = false;
	showvoxelizationmode = false;
	boundingboxisfixed = false;
	manualcorrectmode = false;
	manualcorrectstarted = false;
	generate_pc_from_files = false;
	int mode = 0;
	currentpointcloud = 0;
	Radius_SelectMode = 0.1;
	currentcamera = -1;

	BoundingBoxlength = 2.0;
	BoundingBoxheight = 2.0;
	BoundingBoxstep = 0.05;
	//testmat.identity();
	//testtran=vec3(0,0,0);
	num_recorded_pc = 0;
	num_loaded_pc = 0;
}

vr_rgbd::~vr_rgbd()
{

}
	size_t vr_rgbd::construct_point_cloud()
	{
		intermediate_pc.clear();
		const unsigned short* depths = reinterpret_cast<const unsigned short*>(&depth_frame_2.frame_data.front());
		const unsigned char* colors = reinterpret_cast<const unsigned char*>(&color_frame_2.frame_data.front());
	
		rgbd_inp.map_color_to_depth(depth_frame_2, color_frame_2, warped_color_frame_2);
		colors = reinterpret_cast<const unsigned char*>(&warped_color_frame_2.frame_data.front());
		
		int i = 0;
		for (int y = 0; y < depth_frame_2.height; ++y)
			for (int x = 0; x < depth_frame_2.width; ++x) {
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
	
	size_t vr_rgbd::construct_multi_point_cloud(int index)//
	{
		//for (int index_device = 0; index_device < rgbd_inp.nr_multi_de(); index_device++) {//
			
		//if (index == 0) {
		//intermediate_pc.clear();
		
		//}
			
		const unsigned short* depths = reinterpret_cast<const unsigned short*>(&depth_frame_2.frame_data.front());
		const unsigned char* colors = reinterpret_cast<const unsigned char*>(&color_frame_2.frame_data.front());

		rgbd_inp.map_color_to_depth(depth_frame_2, color_frame_2, warped_color_frame_2, index);
		colors = reinterpret_cast<const unsigned char*>(&warped_color_frame_2.frame_data.front());

		
		int i = 0;
		
		mat3 pc_cam_r = manualcorrect_rotation[index] * cam_fine_r[index] * cam_coarse_r[index];
		vec3 pc_cam_t = manualcorrect_rotation[index] * cam_fine_r[index] * cam_coarse_t[index] + manualcorrect_rotation[index] * cam_fine_t[index] + manualcorrect_translation[index];

		float t;

		intermediate_rgbdpc[index].clear();
		intermediate_rgbdpc_bbox[index].clear();
		for (int y =0; y < depth_frame_2.height; ++y)
			for (int x = 0; x < depth_frame_2.width; ++x) {//
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
					
					
						/*if (x == 256 && y == 256) {
							std::cout<<depths[i]<<std::endl;
							std::cout << p << std::endl;
						}*/				

					//get depth image info
					//imageplanes[index][y][x].depthsquare = p.length();//p[0] * p[0] + p[1] * p[1] + p[2] * p[2];
					//imageplanes[index][y][x].pixelcolor = c;
									
						/*p[0] = p[0]  / p[2];
						p[1] = p[1]  / p[2];
						p[2] = 1;*/

						v.color = c;						
						
						t = p[1];
						p[1] = p[2];
						p[2] = t;


						p = pc_cam_r * p + pc_cam_t;
						/*p = cam_coarse_r[index] * p;															
						p = p + cam_coarse_t[index];	
						p = cam_fine_r[index] * p;
						p = p + cam_fine_t[index];
						p = manualcorrect_rotation[index] * p;
						p = p + manualcorrect_translation[index];*/
						v.point = p;
						if (boundingboxisfixed|| setboundingboxmode)
						{
							if(p[0]<pcbb.pos2[0] &&p[1] < pcbb.pos2[1] &&p[2] < pcbb.pos2[2] && 
								p[0] > pcbb.pos1[0] && p[1] > pcbb.pos1[1] && p[2] > pcbb.pos1[2] )
							intermediate_rgbdpc_bbox[index].add_point(v.point, v.color);
						}
						else
							intermediate_rgbdpc[index].add_point(v.point, v.color);					
					}
						
				}//get depth image info
				/*else {
					imageplanes[index][y][x].depthsquare = -1;
					imageplanes[index][y][x].pixelcolor = rgba8(0, 0, 0, 255);
				}*/
				++i;
			}
		
		//std::cout << "im:" << imageplanes[index].size() << std::endl;
		/*std::cout << "depths:" << depths[130000] << std::endl;*/
		intermediate_rgbdpc_bbox[index].cam_pos = pc_cam_r * vec3(0, 0, 0) + pc_cam_t;
		intermediate_rgbdpc[index].cam_pos = pc_cam_r * vec3(0, 0, 0) + pc_cam_t;
		return intermediate_rgbdpc[index].get_nr_Points();
	
	}
	//void vr_rgbd::getviewconeposition(vec3 &a, mat3 r, vec3 t) {
	//	/*float temp;
	//	temp = a[1];
	//	a[1] = a[2];
	//	a[2] = temp;*/
	//	a= r * a;
	//	a = a + t;
	//	return;

	//}




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
		for (int i = 0; i < 10; i++) {
					
		if (fn.empty())
			return;
		FILE* fp = fopen(fn.c_str(), "wb");
		if (!fp)
			return;
		
		my_pc.write_pc(fn);
		
		fclose(fp);
		
		}
		return;
	}
	void vr_rgbd::Record_PC_FromOneCam(int cam)
	{
		
			rgbdpc[cam].cam_rotation = manualcorrect_rotation[cam] * cam_fine_r[cam] * cam_coarse_r[cam];
			rgbdpc[cam].cam_translation = manualcorrect_rotation[cam] * cam_fine_r[cam] * cam_coarse_t[cam] + manualcorrect_rotation[cam] * cam_fine_t[cam] + manualcorrect_translation[cam];

		
		
			rgbdpc_in_box[cam].cam_rotation = manualcorrect_rotation[cam] * cam_fine_r[cam] * cam_coarse_r[cam];
			rgbdpc_in_box[cam].cam_translation = manualcorrect_rotation[cam] * cam_fine_r[cam] * cam_coarse_t[cam] + manualcorrect_rotation[cam] * cam_fine_t[cam] + manualcorrect_translation[cam];

		
		if (rgbdpc.size() == 0) {
			std::cout << "no pointcloud in the scene" << std::endl;
			return;
		}
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
		rgbdpc[rgbdpc.size() - 1].cam_pos = vec3(0, 0, 0);
		rgbdpc_in_box[rgbdpc_in_box.size() - 1].cam_pos = vec3(0, 0, 0);
		cam_coarse_t.resize(rgbdpc.size());
		cam_coarse_r.resize(rgbdpc.size());

		cam_fine_r.resize(rgbdpc.size());
		cam_fine_t.resize(rgbdpc.size());

		manualcorrect_rotation.resize(rgbdpc.size());
		manualcorrect_translation.resize(rgbdpc.size());
		imageplanes.resize(rgbdpc.size());
		for (int i = 0; i < cam_fine_r.size(); i++) {
			//cam_coarse_r[i].identity();
			//cam_coarse_t[i] = vec3(0, 0, 0);

			cam_coarse_r[i] = rgbdpc[i].cam_rotation;
			cam_coarse_t[i] = rgbdpc[i].cam_translation;

			cam_fine_r[i].identity();
			cam_fine_t[i] = vec3(0, 0, 0);

			manualcorrect_rotation[i].identity();
			manualcorrect_translation[i] = vec3(0, 0, 0);
		}

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
		std::string fn;		
			fn = cgv::gui::file_open_dialog("source point cloud(*.lbypc;*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt)", "Point cloud files:*.lbypc;*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt;");
			pc_load_dir.push_back( fn.substr(0, fn.length() - 7));
		
		if (fn.empty())
			return;

		//char nlpc = fn[fn.length() - 7];
		num_loaded_pc = 0;
		//std::cout<< pc_load_dir [0]<<std::endl;
		//fn = pc_load_dir[pc_load_dir.size()-1] + to_string(num_loaded_pc) + ".lbypc";
		

		rgbd_pointcloud source_pc;
		source_pc.read_pc(fn);

		rgbdpc.push_back(source_pc);
		rgbdpc_in_box.resize(rgbdpc.size());

		cam_coarse_t.resize(rgbdpc.size());
		cam_coarse_r.resize(rgbdpc.size());

		cam_fine_r.resize(rgbdpc.size());
		cam_fine_t.resize(rgbdpc.size());
		manualcorrect_rotation.resize(rgbdpc.size());
		manualcorrect_translation.resize(rgbdpc.size());
		imageplanes.resize(rgbdpc.size());
		//cam_coarse_r[rgbdpc.size()-1].identity();
		//cam_coarse_t[rgbdpc.size() - 1] = vec3(0, 0, 0);

		cam_coarse_r[rgbdpc.size() - 1]= rgbdpc[rgbdpc.size()-1].cam_rotation;
		cam_coarse_t[rgbdpc.size() - 1] = rgbdpc[rgbdpc.size() - 1].cam_translation;

		cam_fine_r[rgbdpc.size() - 1].identity();
		cam_fine_t[rgbdpc.size() - 1] = vec3(0, 0, 0);
		manualcorrect_rotation[rgbdpc.size() - 1].identity();
		manualcorrect_translation[rgbdpc.size() - 1] = vec3(0, 0, 0);
		

		trees.resize(rgbdpc.size());
		//generate_pc_from_files = true;
		post_redraw();

		std::cout << "rgbdpc.size() :" << rgbdpc.size() << std::endl;
		//num_loaded_pc++;
	}
	void vr_rgbd::load_recorded_pc(int index) {
		if (rgbd_inp.is_multi_started() || rgbd_inp.is_started()) {
			std::cout<<"camera(s) is running!" << std::endl;
			return;
		}
		
		std::string fn;
		rgbd_pointcloud mytemppc;//, mytemppc2;
		
		fn = pc_load_dir[index] + to_string(num_loaded_pc) + ".lbypc";
		

		if (fn.empty())
			return;
		

		//rgbdpc[index].read_pc(fn);
		mytemppc.read_pc(fn);
		/*for (int i = 0; i < mytemppc.get_nr_Points(); i++) {
			vec3 p = mytemppc.pnt(i);
			p = cam_coarse_r[index] * p;
			p = p + cam_coarse_t[index];
			p = cam_fine_r[index] * p;
			p = p + cam_fine_t[index];
			p = manualcorrect_rotation[index] * p;
			p = p + manualcorrect_translation[index];
			mytemppc2.add_point(p, mytemppc.clr(i));
		}*/
		rgbdpc[index] = mytemppc;
		cam_coarse_r[index] = rgbdpc[index].cam_rotation;
		cam_coarse_t[index] = rgbdpc[index].cam_translation;
		//rgbdpc[index] = mytemppc2;
		
		post_redraw();

		
		
	
	}



	void vr_rgbd::clear_current_point_cloud() 
	{
		
		generate_pc_from_rgbd = false;
		stop_all_rgbd();
		current_pc.clear();
		rgbdpc.clear();
		post_redraw();
	}

	void vr_rgbd::temp_test() {

		//std::cout << pcbb.pos1 << std::endl;
		//std::cout << pcbb.pos2 << std::endl;
		
		if(!showvoxelizationmode)
			showvoxelizationmode = true;
		else
			showvoxelizationmode = false;
		


		//mat3 c;
		//c.identity();
		//std::cout<<c[0]<<c[1]<<c[2]<<c[3]<<c[4]<<c[5]<<c[6]<<std::endl;
		// 
		// 
		// 
		// 
		// 
		//cgv::math::fmat<float, 3, 3> r; cgv::math::fvec<float, 3> t;
		///*registerPointCloud(rgbdpc[0], rgbdpc[1], r, t);*/
		//SICP* sicp = new SICP();
		//sicp->set_source_cloud(rgbdpc[0]);
		//sicp->set_target_cloud(rgbdpc[1]);
		//std::cout << "sicp->parameters.p" << sicp->parameters.p << std::endl;
		//sicp->register_point_to_point(r, t);
		//std::cout<<"r:" << r << std::endl;
		//std::cout << "t:" << t << std::endl;
		////rgbdpc[0].do_transformation(r, t);
		//
		//vec3 mean = accumulate(&rgbdpc[0].pnt(0), &rgbdpc[0].pnt(0) + rgbdpc[0].get_nr_Points(), vec3(0, 0, 0)) / ((float)rgbdpc[0].get_nr_Points());
		//std::cout << "mean:" << mean << std::endl;
		////need to de-mean for rotation
		//rgbdpc[0].do_transformation(-mean); 
		////do rotation
		////rgbdpc[0].do_transformation(cgv::math::quaternion<float>(r));
		//rgbdpc[0].do_transformation(r);
		////do translation and reapply mean
		//rgbdpc[0].do_transformation(t + mean);
		//rgbdpc[0].cam_pos = vec3(0, 0, 0);
		//post_redraw();
		
		
		/*ICP* icp = new ICP();		
		r.identity();
		t.zeros();
		icp->set_source_cloud(rgbdpc[0]);
		icp->set_target_cloud(rgbdpc[1]);
		icp->set_iterations(20);
		icp->set_eps(1e-10);
		std::cout << "run there!" << std::endl;
		icp->reg_icp(r, t);


		std::cout << "rotation" << r << std::endl;
		std::cout << "translation" << t << std::endl;

		rgbdpc[0].do_transformation(r, t);*/

		/*GoICP mygoicp;
		mygoicp.initializeRegistration(rgbdpc[0]);
		mygoicp.initializeDistanceComputation(rgbdpc[1]);
		mygoicp.registerPointcloud();
		mygoicp.clear();
		rgbdpc[0].do_transformation(mygoicp.optimal_rotation, mygoicp.optimal_translation);*/

	}
	

	void vr_rgbd::registerPointCloud(rgbd_pointcloud target, rgbd_pointcloud& source, cgv::math::fmat<float, 3, 3>& r, cgv::math::fvec<float, 3>& t,int source_index) {

		//ICP* icp = new ICP();
		///*cgv::math::fmat<float, 3, 3> r;
		//cgv::math::fvec<float, 3> t;*/
		//r.identity();
		//t.zeros();
		//std::cout << "source.labels.size():" << source.labels.size() << std::endl;
		//std::cout << "target.labels.size():" << target.labels.size() << std::endl;
		//if (source.labels.size() == 0 || target.labels.size() == 0) {
		//	std::cout << "there is no label points in source or target pc" << std::endl;
		//	return;
		//}

		//rgbd_pointcloud sourcelabelpoints, targetlabelpoints;
		//for (int i = 0; i < source.labels.size(); i++) {
		//	sourcelabelpoints.add_point(source.pnt(source.lab(i)), source.clr(source.lab(i)));
		//	//std::cout << "source labels:" << source.lab(i) << std::endl;
		//}
		//for (int i = 0; i < target.labels.size(); i++) {
		//	targetlabelpoints.add_point(target.pnt(target.lab(i)), target.clr(target.lab(i)));
		//	//std::cout << "target labels:" << target.lab(i) << std::endl;
		//}

		//icp->set_source_cloud(sourcelabelpoints);
		//icp->set_target_cloud(targetlabelpoints);
		//icp->set_iterations(20);
		//icp->set_eps(1e-10);
		//std::cout << "run there!" << std::endl;
		//icp->reg_icp(r, t);



		//std::cout << "rotation" << r << std::endl;
		//std::cout << "translation" << t << std::endl;

		//source.do_transformation(r, t);
		
		//source.cam_pos = r * source.cam_pos + t ;
		/*cam_fine_t[source_index] = r * cam_fine_t[source_index] + t;
		cam_fine_r[source_index] = r* cam_fine_r[source_index];*/
		

		////testmat = testmat * r;
		////testvec = testvec +t;

		//return;

		//================================================================

		/*GoICP mygoicp;
		
		mygoicp.initializeRegistration(source);
		mygoicp.initializeDistanceComputation(target);
		
		std::cout << "mygoicp.registerPointcloud() :" << mygoicp.registerPointcloud() << std::endl;

		//source.cam_pos = mygoicp.optimal_rotation * source.cam_pos +mygoicp.optimal_translation ;
		/*cam_fine_t[source_index] = mygoicp.optimal_rotation * cam_fine_t[source_index] + mygoicp.optimal_translation;
		cam_fine_r[source_index] = mygoicp.optimal_rotation* cam_fine_r[source_index];*/
		
		//source.do_transformation(mygoicp.optimal_rotation, mygoicp.optimal_translation);*/
		//===============================================================
		
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
	/*void vr_rgbd::generate_pc(std::vector<vertex> rgbd_points, rgbd_pointcloud& pc1) {
		pc1.clear();
		for (int i = 0; i < rgbd_points.size(); i++)
		{
			pc1.add_point(rgbd_points[i].point, rgbd_points[i].color);
		}
		return;
	}*/
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

	/*void vr_rgbd::start_select_points() {
		for (int i = 0; i < rgbdpc.size(); i++)
			generate_pc(cur_pc[i], rgbdpc[i]);

	}*/



	frame_type vr_rgbd::read_rgb_frame()    //should be a thread
	{
		return color_frame;
	}
	frame_type vr_rgbd::read_depth_frame()
	{
		return depth_frame;
	}
	
	///cast vertex to point_cloud
	/*void vr_rgbd::copy_pointcloud(const std::vector<vertex> input, point_cloud &output){
		for (unsigned int i = 0; i < input.size(); i++){
			point_cloud_types::Pnt temp;
			temp[0] = input.at(i).point[0];
			temp[1] = input.at(i).point[1];
			temp[2] = input.at(i).point[2];
			point_cloud_types::Clr tempcolor;
			tempcolor[0] = input.at(i).color[0];
			tempcolor[1] = input.at(i).color[1];
			tempcolor[2] = input.at(i).color[2];
			output.P.push_back(temp);
			output.C.push_back(tempcolor);
		}
	}*/
	///cast point_cloud to vertex
	//void vr_rgbd::pc2vertex(const point_cloud &input, std::vector<vertex> &output) {
	//	for (unsigned int i = 0; i < input.get_nr_points(); i++) {
	//		vertex temp;
	//		temp.point[0] = input.pnt(i).x();
	//		temp.point[1] = input.pnt(i).y();
	//		temp.point[2] = input.pnt(i).z();
	//		/*temp.color[0] = input.clr(i)[0];
	//		temp.color[1] = input.clr(i)[1];
	//		temp.color[2] = input.clr(i)[2];*/
	//		temp.color[0] = 0.5;
	//		temp.color[1] = 0.0;
	//		temp.color[2] = 0.0;
	//		output.push_back(temp);
	//	}
	//}
	///here should be const point cloud
	//void vr_rgbd::write_pcs_to_disk(int i)
	//{
	//	if (!intermediate_pc.empty())
	//	{
	//		//define point cloud type, wirte to disk
	//		point_cloud *pc_save = new point_cloud();
	//		pc_save->has_clrs = true;
	//		copy_pointcloud(intermediate_pc, *pc_save);
	//		///pathname
	//		std::string filename = pc_file_path + std::to_string(i) + ".obj";
	//		pc_save->write(filename);
	//	}
	//}
	//size_t vr_rgbd::read_pc_queue(const std::string filename, std::string content)
	//{
	//	cgv::utils::file::read(filename, content, false);
	//	//read pcs from disk
	//	return 0;
	//}
	





	//void vr_rgbd::generate_rdm_pc(point_cloud &pc1, point_cloud& pc2) {
	//	mat3 rotate_m;
	//	rotate_m.identity();
	//	double theta = M_PI / 8;  // The angle of rotation in radians
	//	rotate_m.set_col(0, vec3(std::cos(theta), -sin(theta), 0));
	//	rotate_m.set_col(1, vec3(sin(theta), std::cos(theta), 0));
	//	rotate_m.set_col(2, vec3(0, 0, 1));
	//	for (int i = 0; i < 10000; i++) {
	//		point_cloud_types::Pnt origin;
	//		origin.zeros();
	//		origin.x() = 1024 * rand() / (RAND_MAX + 1.0f);
	//		origin.y() = 1024 * rand() / (RAND_MAX + 1.0f);
	//		origin.z() = 1024 * rand() / (RAND_MAX + 1.0f);
	//		pc1.pnt(i) = origin;
	//		origin = rotate_m * origin;
	//		pc2.pnt(i) = origin + vec3(0.0, 0.4, 0.4);
	//	}
	//}

	//void  vr_rgbd::test_icp() {
	//	cgv::pointcloud::ICP* icp = new cgv::pointcloud::ICP();
	//	cgv::math::fmat<float, 3, 3> r;
	//	cgv::math::fvec<float, 3> t;
	//	r.identity();
	//	t.zeros();
	//	point_cloud* sourcePC = new point_cloud();
	//	point_cloud* targetPC = new point_cloud();
	//	sourcePC->resize(10000);
	//	targetPC->resize(10000);
	//	generate_rdm_pc(*sourcePC, *targetPC);
	//	icp->set_source_cloud(*sourcePC);
	//	icp->set_target_cloud(*targetPC);
	//	//icp->set_source_cloud(*targetPC);
	//	//icp->set_target_cloud(*sourcePC);
	//	icp->set_iterations(5);
	//	icp->set_num_random(3);
	//	icp->set_eps(1e-10);
	//	icp->reg_icp(r, t);
	//}

	//void vr_rgbd::construct_TSDtree()
	//{
	//	//using pc queue to construct the TSDtree
	//}
	bool vr_rgbd::record_this_frame(double t)
	{
		if (!(record_frame || record_all_frames || trigger_is_pressed))
			return false;
		static double last_recording_time = -1;
		if (t - last_recording_time < 1.0 / recording_fps)
			return false;
		last_recording_time = t;
		return true;
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
		//if (rgbd_inp.is_started()) {
			if (rgbd_inp.is_started()) {

				bool new_frame;
				bool found_frame = false;
				bool depth_frame_changed = false;
				bool color_frame_changed = false;			
				do {
					new_frame = false;
					bool new_color_frame_changed = rgbd_inp.get_frame(rgbd::IS_COLOR, color_frame, 0);
					
					if (new_color_frame_changed) {
						
						++nr_color_frames;
						color_frame_changed = new_color_frame_changed;
						new_frame = true;
						update_member(&nr_color_frames);
					}
					bool new_depth_frame_changed = rgbd_inp.get_frame(rgbd::IS_DEPTH, depth_frame, 0);
					if (new_depth_frame_changed) {
						
						++nr_depth_frames;
						depth_frame_changed = new_depth_frame_changed;
						new_frame = true;
						update_member(&nr_depth_frames);
					}
					if (new_frame)
						found_frame = true;
				} while (new_frame);
				if (found_frame)
					post_redraw();
				if (color_frame.is_allocated() && depth_frame.is_allocated() &&
					(color_frame_changed || depth_frame_changed)) {
					
					if (!future_handle.valid()) { //
						if (!in_calibration) {
							color_frame_2 = color_frame;
							depth_frame_2 = depth_frame;
							
						}
						if (zoom_out && !zoom_in)
						{
							controller_orientation_pc = controller_orientation * 2;
							controller_position_pc = controller_position;
						}
						else if(zoom_in && !zoom_out)
						{
							controller_orientation_pc = controller_orientation * 0.5;
							controller_position_pc = controller_position;
						}
						else {
							controller_orientation_pc = controller_orientation;
							controller_position_pc = controller_position;
							
						}
						future_handle = std::async(&vr_rgbd::construct_point_cloud, this);
						//construct_point_cloud();
						//current_pc = intermediate_pc;
						
						//post_redraw();
						
					}
					
				}
			//}
		}


		if (rgbd_inp.is_multi_started()) {
			
			for (int mm = 0; mm < rgbd_inp.nr_multi_de();mm++ )			
			{
				
					bool new_color_frame_changed = rgbd_inp.get_frame(rgbd::IS_COLOR, color_frame, 0, mm);		
					
					bool new_depth_frame_changed = rgbd_inp.get_frame(rgbd::IS_DEPTH, depth_frame, 0, mm);		
					
				if (color_frame.is_allocated() && depth_frame.is_allocated())  //&&(color_frame_changed || depth_frame_changed)
					{
					color_frame_2 = color_frame;
					depth_frame_2 = depth_frame;
					vr_rgbd::construct_multi_point_cloud(mm);
					//future_handle = std::async(&vr_rgbd::construct_point_clouds, this);	
				}	
				
			}
			if (generate_pc_from_rgbd) 
			{
			//current_pc = intermediate_pc;
				if (boundingboxisfixed||setboundingboxmode) {
					rgbdpc_in_box = intermediate_rgbdpc_bbox;
					
					for (int i = 0; i < rgbdpc.size(); i++) {
						rgbdpc_in_box[i].cam_pos = intermediate_rgbdpc[i].cam_pos;

					}
				
				}
				else{
				rgbdpc = intermediate_rgbdpc;
				for (int i = 0; i < rgbdpc.size(); i++) {
					rgbdpc[i].cam_pos = intermediate_rgbdpc[i].cam_pos;
				
				}
				
				if (record_pc_started && num_recorded_pc <= 1000) {
					for (int i = 0; i < rgbdpc.size(); i++)
					{

						Record_PC_FromOneCam(i);

					}
					num_recorded_pc++;

				}
				} 
				post_redraw();
			}
			else {
				/*for (int i = 0; i < rgbdpc.size(); i++)
					rgbdpc[i].clear();*/

				rgbdpc.clear();
			}
			
		}
		else {
			if (mytime == 0)
				mytime++;
			else
				mytime--;
			if (num_loaded_pc < total_loaded_pc) {
				
			if (generate_pc_from_files&& mytime==0)
			{
			
				for(int i=0;i<rgbdpc.size();i++)
					load_recorded_pc(i);
				num_loaded_pc++;
			}
			}
			else {
				generate_pc_from_files = false;
				update_member(&generate_pc_from_files);
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
		string device_def = "enums='no advice=-2;protocol=-1";
		for (unsigned i = 0; i < n; ++i) {
			device_def += ",";
			device_def += rgbd_input::get_serial(i);
		}
		device_def += "'";
		num_devices = n;

		

		connect_copy(add_control("device", (DummyEnum&)device_idx, "dropdown", device_def)->value_change, rebind(this, &vr_rgbd::device_select));

		//std::cout << "num_devices:"<< num_devices << std::endl;
		//std::cout << "////////////////////////////////"  << std::endl;
		add_member_control(this, "currentcamera", currentcamera, "value_slider", "min=-1;max=2;ticks=true");

		add_gui("rgbd_protocol_path", rgbd_protocol_path, "directory", "w=150");
		add_member_control(this, "rgbd_started", rgbd_started, "check");

		add_member_control(this, "color_stream_format", (DummyEnum&)color_stream_format_idx, "dropdown", get_stream_format_enum(color_stream_formats));
		add_member_control(this, "depth_stream_format", (DummyEnum&)depth_stream_format_idx, "dropdown", get_stream_format_enum(depth_stream_formats));		
		//add_member_control(this, "ir_stream_format", (DummyEnum&)ir_stream_format_idx, "dropdown", get_stream_format_enum(ir_stream_formats));
		add_member_control(this, "attach all devices", all_devices_attached, "check");
		add_member_control(this, "rgbd_multi_started", rgbd_multi_started, "check");
		add_member_control(this, "get_tracker_positions", get_tracker_positions, "check");
		//connect_copy(add_control("position_scale", position_scale, "value_slider", "min=0.05;max=10;log=true;ticks=true")->value_change, rebind(static_cast<drawable*>(this), &drawable::post_redraw));
		//connect_copy(add_control("rotation_scale", rotation_scale, "value_slider", "min=0.01;max=1;log=true;ticks=true")->value_change, rebind(static_cast<drawable*>(this), &drawable::post_redraw));
		connect_copy(add_button("save current pointcloud")->click, rebind(this, &vr_rgbd::save_all_pc));
		connect_copy(add_button("load one pointcloud")->click, rebind(this, &vr_rgbd::load_pc));
		connect_copy(add_button("load recorded pointcloud")->click, rebind(this, &vr_rgbd::start_load_recorded_pc));
		add_member_control(this, "start to record pc", record_pc_started, "check");
		add_member_control(this, "play recorded pc", generate_pc_from_files, "check");
		connect_copy(add_button("temp test")->click, rebind(this, &vr_rgbd::temp_test));

		connect_copy(add_button("clear pc")->click, rebind(this, &vr_rgbd::clear_current_point_cloud));

		
		add_member_control(this, "record_frame", record_frame, "check");
		add_member_control(this, "record_all_frames", record_all_frames, "check");
		add_member_control(this, "clear_all_frames", clear_all_frames, "check");
		add_member_control(this, "trigger_is_pressed", trigger_is_pressed, "check");
		add_member_control(this, "recording_fps", recording_fps, "value_slider", "min=1;max=30;ticks=true;log=true");
		add_member_control(this, "in_calibration", in_calibration, "check");
		add_member_control(this, "zoom_in", zoom_in, "check");
		add_member_control(this, "zoom_out", zoom_out, "check");
		add_member_control(this, "save_pc", save_pointcloud, "check");
		add_member_control(this, "register_pc", registration_started, "check");

		add_member_control(this, "rgbd_controller_index", rgbd_controller_index, "value_slider", "min=0;max=3;ticks=true");
		

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
			rh.reflect_member("rgbd_controller_index", rgbd_controller_index) &&
			rh.reflect_member("zoom_in", zoom_in) &&
			rh.reflect_member("zoom_out", zoom_out) &&
			rh.reflect_member("save_pc", save_pointcloud) &&
			rh.reflect_member("register_pc", registration_started) &&
			rh.reflect_member("recording_fps", recording_fps) &&
			rh.reflect_member("ray_length", ray_length) &&
			rh.reflect_member("record_frame", record_frame) &&
			rh.reflect_member("record_all_frames", record_all_frames) &&
			rh.reflect_member("clear_all_frames", clear_all_frames) &&
			rh.reflect_member("rgbd_started", rgbd_started) &&
			rh.reflect_member("generate_pc_from_files", generate_pc_from_files) &&       //????????????
			
			rh.reflect_member("rgbd_multi_started", rgbd_multi_started) &&
			rh.reflect_member("all_devices_attached", all_devices_attached) &&
			rh.reflect_member("record_pc_started", record_pc_started) &&
			//rh.reflect_member("position_scale", position_scale) &&
			//rh.reflect_member("rotation_scale", rotation_scale) &&

			rh.reflect_member("get_tracker_positions", get_tracker_positions) &&
			rh.reflect_member("rgbd_protocol_path", rgbd_protocol_path);
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
		if (member_ptr == &rgbd_protocol_path) {
			rgbd_inp.stop();
			rgbd_inp.detach();
			rgbd_inp.attach_path(rgbd_protocol_path);
			if (rgbd_started)
				start_rgbd();
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
	/*ir_stream_formats.clear();
	rgbd_inp.query_stream_formats(IS_INFRARED, ir_stream_formats);
	if (find_control(ir_stream_format_idx))
		find_control(ir_stream_format_idx)->multi_set(get_stream_format_enum(ir_stream_formats));*/
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


					

					if (selectPointsmode) {
						vec3 ray_origin, ray_direction, ray_end;
						
						vrke.get_state().controller[1].put_ray(&ray_origin(0), &ray_direction(0));												
						ray_end=ray_origin + sphere_distance*ray_direction;
						if (boundingboxisfixed)
							select_feature_points(rgbdpc_in_box[currentpointcloud],ray_end,Radius_SelectMode);
						else
							std::cout<< "no boundingbox " <<std::endl;
					}else if (setboundingboxmode) {
						if(boundingboxisfixed)
							boundingboxisfixed = false;
						else
							boundingboxisfixed = true;

					}
					else if (manualcorrectmode) {
						manualcorrectstarted = true;
					}


					break;
				case cgv::gui::KA_RELEASE:
					
						manualcorrectstarted = false;
					
					break;
				}
			}
			if (ci == 1 && vrke.get_key() == vr::VR_DPAD_DOWN) {
				switch (vrke.get_action()) {
				case cgv::gui::KA_PRESS :
					//rgbd_2_controller_orientation_start_calib = controller_orientation; // V^0 = V
					//rgbd_2_controller_position_start_calib = controller_position;       // r^0 = r
					//in_calibration = true;
					//update_member(&in_calibration);

					
					if (selectPointsmode) {
						vec3 ray_origin, ray_direction, ray_end;

						vrke.get_state().controller[1].put_ray(&ray_origin(0), &ray_direction(0));
						ray_end = ray_origin + sphere_distance * ray_direction;
						if (boundingboxisfixed)
							cancell_selected_feature_points(rgbdpc_in_box[currentpointcloud], ray_end, Radius_SelectMode);
						else
							std::cout << "no boundingbox " << std::endl;
					}
					
					break;
				case cgv::gui::KA_RELEASE:
					
					break;
				}
			}
			

			if (ci == 1 && vrke.get_key() == vr::VR_DPAD_LEFT)
			{
				switch (vrke.get_action()) {
				case cgv::gui::KA_PRESS :
					/*zoom_in = true;
					update_member(&zoom_in);*/

					
					if (selectPointsmode) {
						Radius_SelectMode = Radius_SelectMode * 0.9;
						}

					break;
				case cgv::gui::KA_RELEASE:
					/*zoom_in = false;
					update_member(&zoom_in);*/
					break;
				}
			}
			if (ci == 1 && vrke.get_key() == vr::VR_DPAD_RIGHT)
			{
				switch (vrke.get_action()) {
				case cgv::gui::KA_PRESS:
					/*zoom_out = true;
					update_member(&zoom_out);*/
					
					if (selectPointsmode) {
						Radius_SelectMode = Radius_SelectMode * 1.11;
					}

					break;
				case cgv::gui::KA_RELEASE:
					/*zoom_out = false;
					update_member(&zoom_out);*/
					break;
				}
			}
			if (ci == 0 && vrke.get_key() == vr::VR_DPAD_UP)
			{
				switch (vrke.get_action()) {
				case cgv::gui::KA_PRESS:
					
					if (selectPointsmode)
					{
						currentpointcloud = (currentpointcloud + 1) % rgbdpc.size();
					}else if (manualcorrectmode)
					{
						rgbdpc_in_box[currentpointcloud].renderColors = rgbdpc_in_box[currentpointcloud].Colors;
						currentpointcloud = (currentpointcloud + 1) % rgbdpc.size();
						static const rgba8 mycolor = rgba8(0, 255, 0, 255);
						for (int i = 0; i < rgbdpc_in_box[currentpointcloud].get_nr_Points(); i++)
						{
							rgbdpc_in_box[currentpointcloud].renderColors[i] = mycolor;
						}
					}
					
					
					break;
				case cgv::gui::KA_RELEASE:
					
					break;
				}
			}
			if (ci == 0 && vrke.get_key() == vr::VR_DPAD_DOWN)
			{
				switch (vrke.get_action()) {
				case cgv::gui::KA_PRESS:
					
					if (selectPointsmode)
					{
						currentpointcloud = (currentpointcloud + rgbdpc.size()-1) % rgbdpc.size();
					}
					
					break;
				case cgv::gui::KA_RELEASE:
					
					break;
				}
			}

			if (ci == 0 && vrke.get_key() == vr::VR_DPAD_RIGHT)
			{
				switch (vrke.get_action()) {
				case cgv::gui::KA_PRESS:
					
					if (selectPointsmode) {
						cgv::math::fmat<float, 3, 3> r;
						cgv::math::fvec<float, 3> t;
						r.identity();
						t = vec3(0,0,0);
						/*registerPointCloud(rgbdpc[(currentpointcloud + 1) % rgbdpc.size()], rgbdpc[currentpointcloud], r, t);
						else */
						if(boundingboxisfixed)
							
						{
							int PCwithMaxpoints = 0;
							for(int i=0;i< rgbdpc_in_box.size();i++)
							if(rgbdpc_in_box[i].get_nr_Points()> rgbdpc_in_box[PCwithMaxpoints].get_nr_Points())
								PCwithMaxpoints = i;
							
							 
							//registerPointCloud(rgbdpc_in_box[(currentpointcloud + 1) % rgbdpc.size()], rgbdpc_in_box[currentpointcloud], r, t);
							for (int i = 0; i < rgbdpc_in_box.size(); i++)
							{
								if (i != PCwithMaxpoints) {
									registerPointCloud(rgbdpc_in_box[PCwithMaxpoints], rgbdpc_in_box[i], r, t, i);
									
								}
							}
							
							//registerPointCloud(rgbdpc_in_box[PCwithMaxpoints],rgbdpc_in_box[(PCwithMaxpoints + 2) % rgbdpc.size()],  r, t,);

						}
						build_tree_feature_points(rgbdpc[currentpointcloud], currentpointcloud);
						/*cam_rotation[currentpointcloud] = r * cam_rotation[currentpointcloud];
						cam_translation[currentpointcloud] = r * cam_translation[currentpointcloud] + t;*/

					}
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
					if (boundingboxisfixed) {
						delete_selected_points(rgbdpc_in_box[currentpointcloud]);
						build_tree_feature_points(rgbdpc_in_box[currentpointcloud], currentpointcloud);
				
					}
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
					
					/*if (manualcorrect_translation.size() == 0) {
						std::cout << "no camera linked" << std::endl;
						break;
					}*/
					if(mode==0)
					{
						selectPointsmode = false;
						setboundingboxmode = false;
						manualcorrectmode = false;
						if (boundingboxisfixed) {
							rgbdpc_in_box[currentpointcloud].renderColors = rgbdpc_in_box[currentpointcloud].Colors;
						}
						std::cout << "no mode " << std::endl;
					}				
					else if (mode==1) {
						selectPointsmode = false;
						manualcorrectmode = false;
						setboundingboxmode = true;
						
						std::cout << "setboundingboxmode :" << setboundingboxmode << std::endl;
						
					}
					else if (mode == 2 && boundingboxisfixed) {
						selectPointsmode = true;
						setboundingboxmode = false;
						manualcorrectmode = false;
						std::cout << "selectPointsmode :" << selectPointsmode << std::endl;
						if (rgbdpc.size() == 0)
							std::cout << "no point cloud" << std::endl;
						else {													
								for (int j = 0; j < rgbdpc_in_box.size(); j++)
									build_tree_feature_points(rgbdpc_in_box[j], j);
								for (int i = 0; i < rgbdpc_in_box.size(); i++)
									rgbdpc_in_box[i].set_render_color();
							
						}
						
					}
					else if (mode == 3 && boundingboxisfixed)
					{
						selectPointsmode = false;
						setboundingboxmode = false;
						manualcorrectmode = true;
						static const rgba8 mycolor = rgba8(0, 255, 0, 255);
						for (int i = 0; i < rgbdpc_in_box[currentpointcloud].get_nr_Points(); i++)
						{
							rgbdpc_in_box[currentpointcloud].renderColors[i]= mycolor;
						}
						
					}
					if (!boundingboxisfixed)
						mode = (mode + 1) % 2;
					else mode=(mode+1)%4;
					
					/*clear_all_frames = true;
					update_member(&clear_all_frames);*/
				}
					break;
				case cgv::gui::KA_RELEASE:
					/*clear_all_frames = false;
					update_member(&clear_all_frames);*/
					break;
				}
			}
			return true;
		}
		case cgv::gui::EID_THROTTLE:
		{
			cgv::gui::vr_throttle_event& vrte = static_cast<cgv::gui::vr_throttle_event&>(e);
			if ((vrte.get_last_value() > 0.5f) != (vrte.get_value() > 0.5f)) {
				trigger_is_pressed = (vrte.get_value() > 0.5f);
				update_member(&trigger_is_pressed);
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
					rgbdpc_in_box[currentpointcloud].do_transformation(-last_pos);
					rgbdpc_in_box[currentpointcloud].do_transformation(controllerRotation);
					rgbdpc_in_box[currentpointcloud].do_transformation(pos);
					rgbdpc[currentpointcloud].cam_pos = rotation * (rgbdpc[currentpointcloud].cam_pos- pos) + pos- last_pos;
					

					manualcorrect_translation[currentpointcloud]= controllerRotation * manualcorrect_translation[currentpointcloud] -controllerRotation * last_pos +pos;
					manualcorrect_rotation[currentpointcloud]= controllerRotation* manualcorrect_rotation[currentpointcloud];
				}


				post_redraw();
			}

			if (ci != -1) {
				vec3 origin, direction;
				vrpe.get_state().controller[ci].put_ray(&origin(0), &direction(0));
				if (state[ci] == IS_GRAB) {
					// in grab mode apply relative transformation to grabbed boxes
					
					// get previous and current controller position
					
					// get rotation from previous to current orientation
					// this is the current orientation matrix times the
					// inverse (or transpose) of last orientation matrix:
					// vrpe.get_orientation()*transpose(vrpe.get_last_orientation())
					
					// iterate intersection points of current controller
					for (size_t i = 0; i < intersection_points.size(); ++i) {
						if (intersection_controller_indices[i] != ci)
							continue;
						// extract box index
						
						//unsigned bi = intersection_box_indices[i];
						//// update translation with position change and rotation
						//movable_box_translations[bi] = 
						//	rotation * (movable_box_translations[bi] - last_pos) + pos;
						//// update orientation with rotation, note that quaternions
						//// need to be multiplied in oposite order. In case of matrices
						//// one would write box_orientation_matrix *= rotation
						//movable_box_rotations[bi] = quat(rotation) * movable_box_rotations[bi];
						//// update intersection points
						//intersection_points[i] = rotation * (intersection_points[i] - last_pos) + pos;
						//============================================
						






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

		cgv::render::ref_box_renderer(ctx, 1);
		cgv::render::ref_sphere_renderer(ctx, 1);	

		//std::cout << "================================1231" << std::endl;
		if (!sky_prog.is_created()) {
			sky_prog.build_program(ctx, "glsl/sky.glpr");
			img_tex.create_from_images(ctx, data_dir + "/skybox/cm_{xp,xn,yp,yn,zp,zn}.jpg");//
			
		}
		if (!floor_prog.is_created()) {
			floor_prog.build_program(ctx, "glsl/floor.glpr");
			flo_tex.create_from_image(ctx, data_dir + "/res/floor.jpg");//
			
		}
		Vox->init_voxelization(ctx);

		//=======================delete===========================
		
		//=======================delete==============================



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
	rgb c(0.5, 0.5, 0.5);
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
	pr.set_position_array(ctx, pc.getPoints());
	pr.set_color_array(ctx, pc.getrenderColors());
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

	//mat3 r = manualcorrect_rotation[index] * cam_fine_r[index] * cam_coarse_r[index];
	//vec3 t = manualcorrect_rotation[index] * cam_fine_r[index] * cam_coarse_t[index] + manualcorrect_rotation[index] * cam_fine_t[index] + manualcorrect_translation[index];
	
	/*getviewconeposition(a, cam_coarse_r[cc], cam_coarse_t[cc]);
	getviewconeposition(b, cam_coarse_r[cc], cam_coarse_t[cc]);
	getviewconeposition(c, cam_coarse_r[cc], cam_coarse_t[cc]);
	getviewconeposition(d, cam_coarse_r[cc], cam_coarse_t[cc]);
	getviewconeposition(e, cam_coarse_r[cc], cam_coarse_t[cc]);*/
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
	
	//=======================delete===========================

	//draw_grid(ctx, vec3(0.83623,- 0.728815,2.74123), vec3(2.83623,1.27119,4.74123), 0.05);
	// 
	// 




	if (rgbdpc.size() > 2) {
		//std::cout << rgbdpc[0].cam_rotation << std::endl;
		//Voxelization a;		
		
		auto start_draw = std::chrono::steady_clock::now();

		std::vector<rgbd_pointcloud> allpc;

		for (int i = 0; i < rgbdpc.size(); i++) {
			allpc.push_back(setboundingbox(rgbdpc[i], vec3(0.83623, -0.728815, 2.74123), vec3(2.83623, 1.271185, 4.74123)));
		}
		//std::cout << "1" << std::endl;		

		Vox->init_surface_from_PC(allpc, vec3(0.83623, -0.728815, 2.74123), vec3(2.83623, 1.271185, 4.74123), 0.02);
		
		//std::cout << "2" << std::endl;

		std::vector<vec3> l;
		l.push_back(rgbdpc[0].cam_rotation * vec3(0, 0, 0) + rgbdpc[0].cam_translation);
		//l.push_back(vec3(1.83623, 0.271185, 5));
		l.push_back(rgbdpc[1].cam_rotation * vec3(0, 0, 0) + rgbdpc[1].cam_translation);
		l.push_back(rgbdpc[2].cam_rotation * vec3(0, 0, 0) + rgbdpc[2].cam_translation);

		

		

		Vox->traverse_voxels(ctx, l);

		
		Vox->draw_voxels(ctx);

		auto stop_draw = std::chrono::steady_clock::now();
		std::chrono::duration<double> diff_draw;
		diff_draw = stop_draw - start_draw;
		std::cout << diff_draw.count() << std::endl;
	}	




	//=======================delete===========================




	/*std::vector<Mat> inver_r;
	inver_r.resize(3);
	inver_r[0].identity();
	inver_r[1].identity();
	inver_r[2].identity();
	std::vector<vec3> inver_t;
	inver_t.resize(3);
	inver_t[0] = vec3(0, 0, 0);
	inver_t[1] = vec3(0, 0, 0);
	inver_t[2] = vec3(0, 0, 0);
	std::vector< std::vector<std::vector<depthpixel>>> mydepthimageplane;
	mydepthimageplane.resize(3);
	mydepthimageplane[0].resize(576);
	mydepthimageplane[1].resize(576);
	mydepthimageplane[2].resize(576);
	for (int y = 0; y < 576; y++)
	{
		mydepthimageplane[0][y].resize(640);
		mydepthimageplane[1][y].resize(640);
		mydepthimageplane[2][y].resize(640);
	}
	for (int i = 0; i < 3; i++)
		for (int i2 = 0; i2 < 576; i2++)
			for (int i3 = 0; i3 < 640; i3++) {
				mydepthimageplane[i][i2][i3].depthsquare = 0;
				mydepthimageplane[i][i2][i3].pixelcolor = rgba8(0, 0, 0, 255);
			}
	Voxelization v;
	float step1 = 0.1;

	v.init_voxelization_from_image(ctx, step1, vec3(0,0,0), vec3(1,1,1), inver_r, inver_t, mydepthimageplane);
*/
	

//=======================delete===========================


		if (show_points) {
			auto& pr = cgv::render::ref_point_renderer(ctx);
			pr.set_render_style(point_style);
			pr.set_y_view_angle((float)vr_view_ptr->get_y_view_angle());

			if(current_pc.size()!=0)
				draw_pc(ctx, current_pc);
			
			

			size_t begin = 0;
			size_t end = recorded_pcs.size();
			if (end > max_nr_shown_recorded_pcs)
				begin = end - max_nr_shown_recorded_pcs;
			
			for (size_t i=begin; i<end; ++i)
				draw_pc(ctx, recorded_pcs[i]);
		}
		

		if (vr_view_ptr) {
			
			std::vector<vec3> P;
			std::vector<rgb> C;
			const vr::vr_kit_state* state_ptr = vr_view_ptr->get_current_vr_state();

			if (rgbdpc.size() != 0) {
				if (currentcamera == -1) {
					for (int i = 0; i < rgbdpc.size(); i++) {
						mat3 r_viewing = manualcorrect_rotation[i] * cam_fine_r[i] * cam_coarse_r[i];
						vec3 t_viewing = manualcorrect_rotation[i] * cam_fine_r[i] * cam_coarse_t[i] + manualcorrect_rotation[i] * cam_fine_t[i] + manualcorrect_translation[i];
						draw_viewingcone(ctx, i, P, C, r_viewing, t_viewing);
					}
				}
				else {
					
					mat3 r_viewing = manualcorrect_rotation[currentcamera] * cam_fine_r[currentcamera] * cam_coarse_r[currentcamera];
					vec3 t_viewing = manualcorrect_rotation[currentcamera] * cam_fine_r[currentcamera] * cam_coarse_t[currentcamera] + manualcorrect_rotation[currentcamera] * cam_fine_t[currentcamera] + manualcorrect_translation[currentcamera];
					draw_viewingcone(ctx, currentcamera, P, C, r_viewing, t_viewing);
				}
			}
			if (state_ptr) {
				if (selectPointsmode ) {
					if (state_ptr->controller[1].status == vr::VRS_TRACKED)
					{
						vec3 ray_origin, ray_direction;
						std::vector<vec4> sphere;
						std::vector<rgb> color;
						state_ptr->controller[1].put_ray(&ray_origin(0), &ray_direction(0));
						vec3 sphere_center = ray_origin + sphere_distance * ray_direction;


						sphere.push_back(vec4(sphere_center, Radius_SelectMode));
						color.push_back(rgb(0, 0, 1));
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
				}
				else if (setboundingboxmode&& !boundingboxisfixed) {
					vec3 ray_origin, ray_direction;
					state_ptr->controller[1].put_ray(&ray_origin(0), &ray_direction(0));
					vec3 box_center = ray_origin + ray_length * ray_direction;
					
					
					vec3 pos1, pos2;
					pos1 = box_center - vec3(BoundingBoxlength /2, BoundingBoxlength / 2, BoundingBoxheight / 2);
					pos2 = box_center + vec3(BoundingBoxlength / 2, BoundingBoxlength / 2, BoundingBoxheight / 2);
					

					draw_boudingbox(ctx, pos1, pos2);
					pcbb.pos1 = pos1;
					pcbb.pos2 = pos2;
					std::cout<<"pcbb.pos1"<<pcbb.pos1<<std::endl;
					std::cout<<"pcbb.pos2"<<pcbb.pos2<<std::endl;
					pcbb.step = BoundingBoxstep;


					/*if (rgbdpc.size() > 0) {
					for (int i = 0; i < rgbdpc.size(); i++) {
						rgbdpc_in_box[i] = setboundingbox(rgbdpc[i], pos1, pos2);
					}
					}*/
					
					


				}
				else {
					if (boundingboxisfixed) {
						draw_boudingbox(ctx, pcbb.pos1, pcbb.pos2);


					}



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

		if (setboundingboxmode) {
			if (rgbdpc_in_box.size() != 0) {
				for (int i = 0; i < rgbdpc_in_box.size(); i++)
					draw_rgbdpc(ctx, rgbdpc_in_box[i]);
			}
		}
		else if (selectPointsmode) {
			if (!boundingboxisfixed) {
				if (boundingboxisfixed) {
					if (rgbdpc_in_box.size() != 0) {
						draw_selected_rgbdpc(ctx, rgbdpc_in_box[currentpointcloud]);
					}
				}
				else {
				if (rgbdpc.size() != 0) {
					draw_selected_rgbdpc(ctx, rgbdpc[currentpointcloud]);
				}
				}
			}
			else
			{
				if (rgbdpc.size() != 0) {
					draw_selected_rgbdpc(ctx, rgbdpc_in_box[currentpointcloud]);
				}

			}
					
			
		}
		else if (manualcorrectmode) {
			for (int i = 0; i < rgbdpc_in_box.size(); i++)
			{
				if(i== currentpointcloud)
					draw_selected_rgbdpc(ctx, rgbdpc_in_box[currentpointcloud]);
				else
					draw_rgbdpc(ctx, rgbdpc_in_box[i]);
			}
		}
		
		else if (showvoxelizationmode)
		{
			if (rgbdpc.size() > 0) {
			//std::cout << rgbdpc[0].cam_rotation << std::endl;
			//Voxelization a;

			//std::cout << "1:"<< a.voxel_size<< std::endl;
			Vox->init_voxelization(ctx);
			std::vector<rgbd_pointcloud> o;

			for (int i = 0; i < rgbdpc.size(); i++) {
				o.push_back(setboundingbox(rgbdpc[i], vec3(0.83623, -0.728815, 2.74123), vec3(2.83623, 1.271185, 4.74123)));
			}
			//std::cout << "1" << std::endl;
			Vox->init_surface_from_PC(o, vec3(0.83623, -0.728815, 2.74123), vec3(2.83623, 1.271185, 4.74123), 0.05);
			//std::cout << "2" << std::endl;

			std::vector<vec3> l;
			l.push_back(rgbdpc[0].cam_rotation * vec3(0, 0, 0) + rgbdpc[0].cam_translation);
			//l.push_back(vec3(1.83623, 0.271185, 5));
			l.push_back(rgbdpc[1].cam_rotation * vec3(0, 0, 0) + rgbdpc[1].cam_translation);
			l.push_back(rgbdpc[2].cam_rotation * vec3(0, 0, 0) + rgbdpc[2].cam_translation);
			Vox->traverse_voxels(ctx, l);


			Vox->draw_voxels(ctx);
			}
		}
		else{
			
		if (rgbdpc.size() != 0) {

			//===================delete================

			if (rgbdpc.size()<3&& showvoxelizationmode){

			//===================delete================

			//std::cout << "manualcorrect_rotation[0]:" << manualcorrect_rotation[0] << std::endl;
			for (int i = 0; i < rgbdpc.size(); i++)
			{	
				if (record_pc_started&& num_recorded_pc<=1000) {				
							Record_PC_FromOneCam(i);				
				}
				draw_rgbdpc(ctx, rgbdpc[i]);
				
				
			}
			if (record_pc_started && num_recorded_pc <= 1000)
				num_recorded_pc++;

			//===================delete================

			}

			//===================delete================
		}
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
		/*if (floor_prog.is_created()) {
			std::cout<<"aaaaaa"<<std::endl;
			glDepthMask(GL_FALSE);
			glDisable(GL_CULL_FACE);
			flo_tex.enable(ctx, 1);
			std::cout << "2222" << std::endl;
			floor_prog.enable(ctx);
			std::cout << "333" << std::endl;
			floor_prog.set_uniform(ctx, "tex", 1);
			ctx.push_modelview_matrix();
			ctx.mul_modelview_matrix(cgv::math::scale4<double>(
				max_scene_extent, max_scene_extent, max_scene_extent));
			ctx.tesselate_unit_cube();
			ctx.pop_modelview_matrix();
			floor_prog.disable(ctx);
			flo_tex.disable(ctx);
			glEnable(GL_CULL_FACE);
			glDepthMask(GL_TRUE);
		}*/



		



		/*boxes.clear();
		boxes.
		box_colors.
		cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
		renderer.set_render_style(style);
		renderer.set_box_array(ctx, boxes);
		renderer.set_color_array(ctx, box_colors);
		if (renderer.validate_and_enable(ctx)) {
			glDrawArrays(GL_POINTS, 0, (GLsizei)boxes.size());
		}
		renderer.disable(ctx);*/





		/*std::vector<vec3> myline;
		std::vector<rgb> mycolor;
		
			vec3 origin=(0,0,1);
			vec3 end = (0, 0, 2);
			vec3 origin1 = (0, 0, 3);
			vec3 end1 = (0, 0, 4);
			vec3 origin2 = (0, 0, 0);
			vec3 end2 = (2, 2, 2);
			myline.push_back(origin);
			myline.push_back(end);
			myline.push_back(origin1);
			myline.push_back(end1);
			myline.push_back(origin2);
			myline.push_back(end2);
			rgb mycol(1.0, 1.0, 1.0);
			mycolor.push_back(mycol);
			*/
			
			//
			
			
				//cgv::render::shader_program& prog = ctx.ref_default_shader_program();
				//int pi = prog.get_position_index();
				//int ci = prog.get_color_index();
				//cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, myline);
				//cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
				//cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ci, mycolor);
				//cgv::render::attribute_array_binding::enable_global_array(ctx, ci);
				//glLineWidth(3);
				//prog.enable(ctx);
				//glDrawArrays(GL_LINES, 0, (GLsizei)myline.size());
				//prog.disable(ctx);
				//cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
				//cgv::render::attribute_array_binding::disable_global_array(ctx, ci);
				////glLineWidth(1);

				//myline.clear();
				//mycolor.clear();

			




		/*voxel::Voxelization myvoxel;
		std::vector<vec3> ret;
		float step = 1.2;
		vec3 min = vec3(0, 0, 0);
		vec3 max = vec3(1, 1, 1);
		rgbd_pointcloud rp1;
		rp1.add_point(min);
		myvoxel.voxelize(ctx, step, ret, min, max, rp1);*/

		//========================================================
		/*std::cout<<"000000000" << std::endl;
		std::vector<box3> boxes1;
		boxes1.clear();
		vec3 min11 = vec3(-1, -1, -1);
		vec3 max11 = vec3(0, 0, 0);
		boxes1.push_back(box3(min11, max11));
		cgv::render::voxel_renderer& renderer1 = cgv::render::ref_voxel_renderer(ctx);

		std::cout << "11111111111" << std::endl;

		std::vector<rgb> box_colors1;
		box_colors1.clear();
		rgb table_clr11(0.3f, 0.2f, 0.0f);
	
		box_colors1.push_back(table_clr11);
		renderer1.enable(ctx);
		renderer1.set_render_style(style);
		std::cout << "222222222222" << std::endl;
		renderer1.set_voxel_array(ctx, boxes1);
		renderer1.set_color_array(ctx, box_colors1);
		std::cout << "3333333333333" << std::endl;
		

		if (renderer1.validate_and_enable(ctx)) {
			glDrawArrays(GL_POINTS, 0, (GLsizei)boxes1.size());
		}

		

		renderer1.disable(ctx);*/

		//======================================================================


		//// draw static boxes
	/*	boxes.clear();
		vec3 min1 = vec3(0,0,0);
		vec3 max1 = vec3(1, 1, 1);
		boxes.push_back(box3(min1, max1));
		cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
		box_colors.clear();
		rgb table_clr1(0.3f, 0.2f, 0.0f);
		box_colors.push_back(table_clr1);

		renderer.set_render_style(style);
		

		renderer.set_box_array(ctx, boxes);
		renderer.set_color_array(ctx, box_colors);
		if (renderer.validate_and_enable(ctx)) {
			glDrawArrays(GL_POINTS, 0, (GLsizei)boxes.size());
		}
		renderer.disable(ctx);*/

		//// draw dynamic boxes 
		//renderer.set_render_style(movable_style);
		//renderer.set_box_array(ctx, movable_boxes);
		//renderer.set_color_array(ctx, movable_box_colors);
		//renderer.set_translation_array(ctx, movable_box_translations);
		//renderer.set_rotation_array(ctx, movable_box_rotations);
		//if (renderer.validate_and_enable(ctx)) {
		//	glDrawArrays(GL_POINTS, 0, (GLsizei)movable_boxes.size());
		//}
		//renderer.disable(ctx);

		// draw intersection points
		if (!intersection_points.empty()) {
			auto& sr = cgv::render::ref_sphere_renderer(ctx);
			sr.set_position_array(ctx, intersection_points);
			sr.set_color_array(ctx, intersection_colors);
			sr.set_render_style(srs);
			if (sr.validate_and_enable(ctx)) {
				glDrawArrays(GL_POINTS, 0, (GLsizei)intersection_points.size());
				sr.disable(ctx);
			}
		}
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