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

void vr_rgbd::start_multi_rgbd()
{
	
	if (!rgbd_inp.is_multi_attached())
	{
		
		if (rgbd::rgbd_input::get_nr_devices() == 0)
		{
			
			return;
		}
		
		vector<std::string> ser;
		
		for (int nr_de=0;nr_de< rgbd::rgbd_input::get_nr_devices();nr_de++) 	
		{
			ser.push_back(rgbd::rgbd_input::get_serial(nr_de));
		}
		
		
		if (!rgbd_inp.multi_attach(ser))
		{			
			return;
		}
		std::cout << "size of multi_rgbd:" << rgbd_inp.nr_multi_de() << std::endl;

	}

	if (rgbd_inp.is_multi_attached())
		std::cout << "these are attached" << std::endl;
	else
		std::cout << "these are not attached" << std::endl;
	
	
	std::vector<std::vector<rgbd::stream_format>> multi_stream_formats(rgbd_inp.nr_multi_de());
	rgbd_multi_started = rgbd_inp.multi_start(rgbd::IS_COLOR_AND_DEPTH, multi_stream_formats);
	if (rgbd_multi_started)
		std::cout << "they are started" << std::endl;
	else
		std::cout << "they are not started" << std::endl;
	for (int mc = 0;mc<rgbd_inp.get_nr_devices();mc++) {
		manualcorrect_translation.push_back((0, 0, 0));
		manualcorrect_rotation.push_back((0, 0, 0));
	}
	current_corrected_cam =-1;
	cur_pc.resize(rgbd_inp.get_nr_devices());
	update_member(&rgbd_multi_started);
	
}
void vr_rgbd::stop_multi_rgbd() 
{
	std::cout << "nr of attached devices" << rgbd_inp.nr_multi_de() << std::endl;
	if (!rgbd_inp.is_multi_started())
		return;
	//rgbd_inp.stop();//
	
	rgbd_multi_started = !rgbd_inp.stop();
	rgbd_inp.detach();
	
	
	manualcorrect_translation.clear();
	manualcorrect_rotation.clear();
	current_corrected_cam =-1;
	translationmode = false;
	rotationmode = false;
	std::cout << "nr of attached devices"<<rgbd_inp.nr_multi_de() << std::endl;
	current_pc.clear();
	update_member(&rgbd_multi_started);
}




void vr_rgbd::stop_rgbd()
{
	if (!rgbd_inp.is_started())
		return;
	
	rgbd_inp.stop();
	rgbd_inp.detach();
	
	rgbd_started = rgbd_inp.stop();
	update_member(&rgbd_started);
}
void vr_rgbd::set_rgbd_pos()
{
	
	get_camera_pos_1 = true;
	get_camera_pos_2 = true;
	
	std::cout << "camera1 ori:" << std::endl << camera_ori_1 << std::endl;
	//std::cout << "qqqqqqqqqqqqqqqqqqqqqqqqqqqqq:" << camera_ori_1(0,1) << std::endl;
	std::cout << "camera2 ori:" <<std::endl << camera_ori_2 << std::endl;

	std::cout << "camera1 pos:" << camera_pos_1 << std::endl;
	std::cout << "camera2 pos:" << camera_pos_2 << std::endl;
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

	srs.radius = 0.005f;
	state[0] = state[1] = state[2] = state[3] = IS_NONE;
	rgbd_started = false;
	rgbd_multi_started = false;
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

	device_mode = No_Device;
	
	device_idx = -2;
	num_devices = 0;

	pc_file_path = QUOTE_SYMBOL_VALUE(INPUT_DIR) " / .. / data";

	connect(cgv::gui::get_animation_trigger().shoot, this, &vr_rgbd::timer_event);

	
	get_camera_pos_1 = false;
	camera_pos_1 = (0, 0, 0);
	camera_ori_1.identity();
	get_camera_pos_2 = false;
	camera_pos_2 = (0, 0, 0);
	camera_ori_2.identity();
	get_camera_pos_3 = false;
	camera_pos_3 = (0, 0, 0);
	camera_ori_3.identity();
	no_controller=false;

	translationmode=false;
	rotationmode=false;

	rotation_scale = 0.1;
	position_scale = 0.1;
	generate_pc_from_rgbd = true;
	//mvp.identity();
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
			
		if(index ==0)
			intermediate_pc.clear();
		const unsigned short* depths = reinterpret_cast<const unsigned short*>(&depth_frame_2.frame_data.front());
		const unsigned char* colors = reinterpret_cast<const unsigned char*>(&color_frame_2.frame_data.front());

		rgbd_inp.map_color_to_depth(depth_frame_2, color_frame_2, warped_color_frame_2, index);
		colors = reinterpret_cast<const unsigned char*>(&warped_color_frame_2.frame_data.front());


		//std::vector<mat3> my_rotation_matrix;
		//mat3 mrm = camera_ori_1;
		//mrm[5] += manualcorrect_rotation[index][2];
		//mrm[4] = sqrt(1- mrm[5]* mrm[5]- mrm[3] * mrm[3]);
		//mrm[8] += manualcorrect_rotation[index][2];
		//mrm[7] = sqrt(1 - mrm[8] * mrm[8] - mrm[6] * mrm[6]);
		
		/*mrm[5] += manualcorrect_rotation[index][0];
		mrm[4] = sqrt(1 - mrm[5] * mrm[5] - mrm[3] * mrm[3]);

		mrm[5] += manualcorrect_rotation[index][1];
		mrm[4] = sqrt(1 - mrm[5] * mrm[5] - mrm[3] * mrm[3]);*/

		int i = 0;
		std::vector<vertex> temppc;
		for (int y = 0; y < depth_frame_2.height; ++y)
			for (int x = 0; x < depth_frame_2.width; ++x) {
				vec3 p;
				if (rgbd_inp.map_depth_to_point(x, y, depths[i], &p[0], index)) {//,index&p[0]
					// flipping y to make it the same direction as in pixel y coordinate
							
					if (index == 0 )
					{
						float t;
						t = p[1];
						p[1] = p[2];
						p[2] = -t;				
						p = camera_ori_1 * p;
						//p = mrm[index] * p;
						p[1] = -p[1];		
						p = p + camera_pos_1;
						p = p + manualcorrect_translation[index];

					}

					if (index == 1)
					{
						float t;
						t = p[1];
						p[1] = p[2];
						p[2] = -t;
						p = camera_ori_2*p  ;
						p[1] = -p[1];
						p = p + camera_pos_2;
						p = p + manualcorrect_translation[index];
					}

					if (index == 2)
					{
						float t;
						t = p[1];
						p[1] = p[2];
						p[2] = -t;
						p = camera_ori_3 * p;
						p[1] = -p[1];
						p = p + camera_pos_3;
						p = p + manualcorrect_translation[index];
					}

					rgba8 c(colors[4 * i + 2], colors[4 * i + 1], colors[4 * i], 255);
					vertex v;
					//filter points without color for 32 bit formats
					static const rgba8 filter_color = rgba8(0, 0, 0, 255);
					if (!(c == filter_color)) {
						v.color = c;
						v.point = p;
					}
									
					temppc.push_back(v);			
					intermediate_pc.push_back(v);
				}
				++i;
			}
		cur_pc[index] = temppc;
		
		return intermediate_pc.size();
	
	}



	//size_t vr_rgbd::construct_point_clouds() {
	//	intermediate_pc.clear();
	//	
	//	for (int index = 0; index < rgbd_inp.nr_multi_de();index++) {
	//	const unsigned short* depths = reinterpret_cast<const unsigned short*>(&depth_frames[index].frame_data.front());
	//	const unsigned char* colors = reinterpret_cast<const unsigned char*>(&color_frames[index].frame_data.front());
	//	rgbd_inp.map_color_to_depth(depth_frames[index], color_frames[index], warped_color_frame_2, index);
	//	colors = reinterpret_cast<const unsigned char*>(&warped_color_frame_2.frame_data.front());
	//	int i = 0;
	//	for (int y = 0; y < depth_frames[index].height; ++y)
	//		for (int x = 0; x < depth_frames[index].width; ++x) {
	//			vec3 p;
	//			if (rgbd_inp.map_depth_to_point(x, y, depths[i], &p[0], index)) {//,index&p[0]
	//				// flipping y to make it the same direction as in pixel y coordinate

	//				if (index == 0)
	//				{
	//					float t;
	//					t = p[1];
	//					p[1] = p[2];
	//					p[2] = -t;
	//					p = camera_ori_1 * p;
	//					p[1] = -p[1];
	//					p = p + camera_pos_1;
	//					p = p + manualcorrect_translation[index];
	//				}
	//				if (index == 1)
	//				{
	//					float t;
	//					t = p[1];
	//					p[1] = p[2];
	//					p[2] = -t;
	//					p = camera_ori_2 * p;
	//					p[1] = -p[1];
	//					p = p + camera_pos_2;
	//					p = p + manualcorrect_translation[index];
	//				}
	//				if (index == 2)
	//				{
	//					float t;
	//					t = p[1];
	//					p[1] = p[2];
	//					p[2] = -t;
	//					p = camera_ori_3 * p;
	//					p[1] = -p[1];
	//					p = p + camera_pos_3;
	//					p = p + manualcorrect_translation[index];
	//				}
	//				rgba8 c(colors[4 * i + 2], colors[4 * i + 1], colors[4 * i], 255);
	//				vertex v;
	//				//filter points without color for 32 bit formats
	//				static const rgba8 filter_color = rgba8(0, 0, 0, 255);
	//				if (!(c == filter_color)) {
	//					v.color = c;
	//					v.point = p;
	//				}
	//				intermediate_pc.push_back(v);
	//			}
	//			++i;
	//		}


	//	}
	//	
	//	return intermediate_pc.size();
	//}

	void vr_rgbd::save_current_pc()
	{

		if (current_pc.size()==0){
			std::cout<<"no pointcloud in the scene"<<std::endl;
			return;
		}


		rgbd_pointcloud my_pc;
		if (save_time == rgbd_inp.get_nr_devices()) {
		for (int i =0;i<current_pc.size();i++) 
		{
			my_pc.add_point(current_pc[i].point, current_pc[i].color);
		}
		save_time = 0;
		}
		else {
			for (int i = 0; i < cur_pc[save_time].size(); i++)
			{
				my_pc.add_point(cur_pc[save_time][i].point, cur_pc[save_time][i].color);
			}
			save_time++;
		}
		std::string fn = cgv::gui::file_save_dialog("point cloud", "Point Cloud Files (lbypc,ply,bpc,apc,obj):*.txt;*.lbypc");
		if (fn.empty())
			return;
		FILE* fp = fopen(fn.c_str(), "wb");
		if (!fp)
			return;
		my_pc.write_pc(fn);	
		fclose(fp);
		return;
		


	}

	void vr_rgbd::load_current_pc() 
	{
		
		std::string fn = cgv::gui::file_open_dialog("source point cloud(*.lbypc;*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt)", "Point cloud files:*.lbypc;*.obj;*.pobj;*.ply;*.bpc;*.lpc;*.xyz;*.pct;*.points;*.wrl;*.apc;*.pnt;*.txt;");
		if (fn.empty())
			return;
		clear_current_point_cloud();
		source_pc.read_pc(fn);
		vector<vertex> temp_pc;
		for (int i = 0; i < source_pc.get_nr_Points(); i++)
		{
			vertex v;
			v.point = source_pc.Points[i];
			v.color = source_pc.Colors[i];
			temp_pc.push_back(v);
		}
		current_pc = temp_pc;
		post_redraw();
		
	}
	void vr_rgbd::clear_current_point_cloud() 
	{
		
		generate_pc_from_rgbd = false;
		stop_multi_rgbd();
		current_pc.clear();
	}






	frame_type vr_rgbd::read_rgb_frame()    //should be a thread
	{
		return color_frame;
	}
	frame_type vr_rgbd::read_depth_frame()
	{
		return depth_frame;
	}
	///cast vertex to point_cloud
	void vr_rgbd::copy_pointcloud(const std::vector<vertex> input, point_cloud &output){
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
	}
	///cast point_cloud to vertex
	void vr_rgbd::pc2vertex(const point_cloud &input, std::vector<vertex> &output) {
		for (unsigned int i = 0; i < input.get_nr_points(); i++) {
			vertex temp;
			temp.point[0] = input.pnt(i).x();
			temp.point[1] = input.pnt(i).y();
			temp.point[2] = input.pnt(i).z();
			/*temp.color[0] = input.clr(i)[0];
			temp.color[1] = input.clr(i)[1];
			temp.color[2] = input.clr(i)[2];*/
			temp.color[0] = 0.5;
			temp.color[1] = 0.0;
			temp.color[2] = 0.0;
			output.push_back(temp);
		}
	}
	///here should be const point cloud
	void vr_rgbd::write_pcs_to_disk(int i)
	{
		if (!intermediate_pc.empty())
		{
			//define point cloud type, wirte to disk
			point_cloud *pc_save = new point_cloud();
			pc_save->has_clrs = true;
			copy_pointcloud(intermediate_pc, *pc_save);
			///pathname
			std::string filename = pc_file_path + std::to_string(i) + ".obj";
			pc_save->write(filename);
		}
	}
	size_t vr_rgbd::read_pc_queue(const std::string filename, std::string content)
	{
		cgv::utils::file::read(filename, content, false);
		//read pcs from disk
		return 0;
	}
	void vr_rgbd::registrationPointCloud() {
		cgv::pointcloud::ICP *icp = new cgv::pointcloud::ICP();
		if (recorded_pcs.size() >= 1) {
			cgv::math::fmat<float, 3, 3> r;
			cgv::math::fvec<float, 3> t;
			r.identity();
			t.zeros();
			point_cloud *sourcePC = new point_cloud();
			point_cloud* sourcecopy = new point_cloud();
			point_cloud *targetPC = new point_cloud();
			sourcePC->resize(intermediate_pc.size());
			targetPC->resize(recorded_pcs.front().size());
			sourcecopy->resize(intermediate_pc.size());
			copy_pointcloud(recorded_pcs.front(), *targetPC);
			copy_pointcloud(intermediate_pc, *sourcePC);
			icp->set_source_cloud(*sourcePC);
			icp->set_target_cloud(*targetPC);
			icp->set_iterations(5);
			icp->set_eps(1e-10);
			icp->reg_icp(r, t);
			for (int i = 0; i < sourcePC->get_nr_points(); i++)
			{
				sourcePC->pnt(i) = r * sourcePC->pnt(i) + t;
			}
			intermediate_pc.clear();
			pc2vertex(*sourcePC, intermediate_pc);
			std::cout << "size: " << recorded_pcs.size() << " "<< intermediate_pc.size()<<std::endl;
		}		
	}

	void vr_rgbd::generate_rdm_pc(point_cloud &pc1, point_cloud& pc2) {
		mat3 rotate_m;
		rotate_m.identity();
		double theta = M_PI / 8;  // The angle of rotation in radians
		rotate_m.set_col(0, vec3(std::cos(theta), -sin(theta), 0));
		rotate_m.set_col(1, vec3(sin(theta), std::cos(theta), 0));
		rotate_m.set_col(2, vec3(0, 0, 1));
		for (int i = 0; i < 10000; i++) {
			point_cloud_types::Pnt origin;
			origin.zeros();
			origin.x() = 1024 * rand() / (RAND_MAX + 1.0f);
			origin.y() = 1024 * rand() / (RAND_MAX + 1.0f);
			origin.z() = 1024 * rand() / (RAND_MAX + 1.0f);
			pc1.pnt(i) = origin;
			origin = rotate_m * origin;
			pc2.pnt(i) = origin + vec3(0.0, 0.4, 0.4);
		}
	}

	void  vr_rgbd::test_icp() {
		cgv::pointcloud::ICP* icp = new cgv::pointcloud::ICP();
		cgv::math::fmat<float, 3, 3> r;
		cgv::math::fvec<float, 3> t;
		r.identity();
		t.zeros();
		point_cloud* sourcePC = new point_cloud();
		point_cloud* targetPC = new point_cloud();
		sourcePC->resize(10000);
		targetPC->resize(10000);
		generate_rdm_pc(*sourcePC, *targetPC);
		icp->set_source_cloud(*sourcePC);
		icp->set_target_cloud(*targetPC);
		//icp->set_source_cloud(*targetPC);
		//icp->set_target_cloud(*sourcePC);
		icp->set_iterations(5);
		icp->set_num_random(3);
		icp->set_eps(1e-10);
		icp->reg_icp(r, t);
	}

	void vr_rgbd::construct_TSDtree()
	{
		//using pc queue to construct the TSDtree
	}
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
						//future_handle = std::async(&vr_rgbd::construct_point_cloud, this);
						//construct_point_cloud();
						//current_pc = intermediate_pc;
						//post_redraw();
						//std::cout<<"run this"<<std::endl;
					}
					
				}
			//}
		}
















		if (rgbd_inp.is_multi_started()) {
			for (int m = 0; m < rgbd_inp.nr_multi_de(); m++)			
			{
					bool new_color_frame_changed = rgbd_inp.get_frame(rgbd::IS_COLOR, color_frame, 0, m);				
					bool new_depth_frame_changed = rgbd_inp.get_frame(rgbd::IS_DEPTH, depth_frame, 0, m);				
				if (color_frame.is_allocated() && depth_frame.is_allocated())  //&&(color_frame_changed || depth_frame_changed)
					{
					color_frame_2 = color_frame;
					depth_frame_2 = depth_frame;
					vr_rgbd::construct_multi_point_cloud(m);
					//future_handle = std::async(&vr_rgbd::construct_point_clouds, this);	
				}	
			}
			if (generate_pc_from_rgbd) 
			{
			current_pc = intermediate_pc;post_redraw();
			}
			else {
				current_pc.clear();
			}
			
		}

















		//if (rgbd_inp.is_multi_started()) {
		//	//color_frames.clear();
		//	//depth_frames.clear();
		//	//color_frames_2.clear();
		//	//depth_frames_2.clear();
		//	for (int m = 0; m < rgbd_inp.nr_multi_de(); m++)
		//	{
		//		
		//		bool new_color_frame_changed = rgbd_inp.get_frame(rgbd::IS_COLOR, mycolor_frame, 0, m);
		//		bool new_depth_frame_changed = rgbd_inp.get_frame(rgbd::IS_DEPTH, mydepth_frame, 0, m);
		//		if (mycolor_frame.is_allocated() && mydepth_frame.is_allocated())
		//		{
		//			if (color_frames.size() == 0|| color_frames.size() == 1) {
		//				color_frames.push_back(mycolor_frame);
		//				depth_frames.push_back(mydepth_frame);}
		//			else {
		//				color_frames[m]=mycolor_frame;
		//				depth_frames[m]=mydepth_frame;
		//			}
		//		}					
		//	}			
		//	if(color_frames.size()>0 && depth_frames.size() > 0 && color_frames.size()== depth_frames.size())
		//	{
		//		color_frames_2 = color_frames;
		//		depth_frames_2 = depth_frames;				
		//		if (!future_handle.valid()) 
		//		{			
		//		future_handle = std::async(&vr_rgbd::construct_point_clouds, this);
		//		}			
		//		//vr_rgbd::construct_point_clouds();
		//	}
		//}
			//current_pc = intermediate_pc; 
			//post_redraw();
		

}

std::string vr_rgbd::get_type_name() const
{
		return "vr_rgbd";
}
void vr_rgbd::create_gui()
{
	
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
		
		add_gui("rgbd_protocol_path", rgbd_protocol_path, "directory", "w=150");
		add_member_control(this, "rgbd_started", rgbd_started, "check");
		add_member_control(this, "depth_stream_format", (DummyEnum&)depth_stream_format_idx, "dropdown", get_stream_format_enum(depth_stream_formats));
		add_member_control(this, "rgbd_multi_started", rgbd_multi_started, "check");
		add_member_control(this, "get_tracker_positions", get_tracker_positions, "check");
		connect_copy(add_control("position_scale", position_scale, "value_slider", "min=0.05;max=10;log=true;ticks=true")->value_change, rebind(static_cast<drawable*>(this), &drawable::post_redraw));
		connect_copy(add_control("rotation_scale", rotation_scale, "value_slider", "min=0.01;max=1;log=true;ticks=true")->value_change, rebind(static_cast<drawable*>(this), &drawable::post_redraw));
		connect_copy(add_button("save pointcloud")->click, rebind(this, &vr_rgbd::save_current_pc));
		connect_copy(add_button("load pointcloud")->click, rebind(this, &vr_rgbd::load_current_pc));


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
			rh.reflect_member("rgbd_multi_started", rgbd_multi_started) &&
			rh.reflect_member("position_scale", position_scale) &&
			rh.reflect_member("rotation_scale", rotation_scale) &&
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
				stop_multi_rgbd();
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
					if (translationmode&& current_corrected_cam !=-1)
					{		
						manualcorrect_translation[current_corrected_cam][2] += position_scale;

					}
					if (rotationmode && current_corrected_cam != -1)
					{
						manualcorrect_rotation[current_corrected_cam][2] += rotation_scale;

					}
					break;
				case cgv::gui::KA_RELEASE:
					
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
					if (translationmode && current_corrected_cam != -1)
					{	
						manualcorrect_translation[current_corrected_cam][2] -= position_scale;
					}
					if (rotationmode && current_corrected_cam != -1)
					{
						manualcorrect_rotation[current_corrected_cam][2] -= rotation_scale;

					}
					break;
				case cgv::gui::KA_RELEASE:
					/*rgbd_2_controller_orientation = transpose(rgbd_2_controller_orientation_start_calib)*controller_orientation*rgbd_2_controller_orientation;
					rgbd_2_controller_position = transpose(rgbd_2_controller_orientation_start_calib)*((controller_orientation*rgbd_2_controller_position + controller_position) - rgbd_2_controller_position_start_calib);
					in_calibration = false;
					update_member(&in_calibration);*/
					break;
				}
			}
			

			if (ci == 1 && vrke.get_key() == vr::VR_DPAD_LEFT)
			{
				switch (vrke.get_action()) {
				case cgv::gui::KA_PRESS :
					/*zoom_in = true;
					update_member(&zoom_in);*/
					if (translationmode && current_corrected_cam != -1)
					{		
						manualcorrect_translation[current_corrected_cam][0] += position_scale;
					}
					if (rotationmode && current_corrected_cam != -1)
					{
						manualcorrect_rotation[current_corrected_cam][0] += rotation_scale;

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
					if (translationmode && current_corrected_cam != -1)
					{
						manualcorrect_translation[current_corrected_cam][0] -= position_scale;					
					}
					if (rotationmode && current_corrected_cam != -1)
					{
						manualcorrect_rotation[current_corrected_cam][0] -= rotation_scale;

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
					//if (get_camera_pos_1 == false) {
					//	vec3 p = vec3(vrke.get_state().controller[1].pose[9], vrke.get_state().controller[1].pose[10], vrke.get_state().controller[1].pose[11]);
					//	camera_translation_1 = p;
					//	std::cout<<"p positon:"<<p<< std::endl;
					//	vec3 p1 = vec3(vrke.get_state().controller[1].pose[0], vrke.get_state().controller[1].pose[1], vrke.get_state().controller[1].pose[2]);
					//	vec3 p2 = vec3(vrke.get_state().controller[1].pose[3], vrke.get_state().controller[1].pose[4], vrke.get_state().controller[1].pose[5]);
					//	vec3 p3 = vec3(vrke.get_state().controller[1].pose[6], vrke.get_state().controller[1].pose[7], vrke.get_state().controller[1].pose[8]);
					//	std::cout << "p1p2p3:"<< p1 << std::endl;
					//	std::cout << "       "<< p2 << std::endl;
					//	std::cout << "       "<< p3 << std::endl;
					//	
					//	//get_camera_pos_1 = true;
					//}else if(camera_pos_2 == false)
					//{
					//	vec3 p = vec3(vrke.get_state().controller[1].pose[9], vrke.get_state().controller[1].pose[10], vrke.get_state().controller[1].pose[11]);
					//	camera_translation_2 = p;

					//	

					//	camera_pos_2 = true;
					//}
					//else if (camera_pos_3 == false)
					//{
					//	vec3 p = vec3(vrke.get_state().controller[1].pose[9], vrke.get_state().controller[1].pose[10], vrke.get_state().controller[1].pose[11]);
					//	camera_translation_3 = p;
					//	camera_pos_3 = true;
					//}
					if (translationmode && current_corrected_cam != -1)
					{
						manualcorrect_translation[current_corrected_cam][1] += position_scale;
						
					}
					if (rotationmode && current_corrected_cam != -1)
					{
						manualcorrect_rotation[current_corrected_cam][1] += rotation_scale;

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
					
					if (translationmode && current_corrected_cam != -1)
					{
						manualcorrect_translation[current_corrected_cam][1] -= position_scale;
						
					}
					if (rotationmode && current_corrected_cam != -1)
					{
						manualcorrect_rotation[current_corrected_cam][1] -= rotation_scale;

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
				//std::cout << "this is link controller" << std::endl;
					if (manualcorrect_rotation.size() == 0) {
						std::cout << "no camera linked" << std::endl;
						break;
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
					
					if (manualcorrect_translation.size() == 0) {
						std::cout << "no camera linked" << std::endl;
						break;
					}
					if (translationmode) {
						if (current_corrected_cam < manualcorrect_translation.size())
						{
							current_corrected_cam = current_corrected_cam+ 1;
						}
						if (current_corrected_cam == manualcorrect_translation.size() )
						{
							current_corrected_cam = 0;
						}
						std::cout << "manualcorrect_translation size :  " << manualcorrect_translation.size() << std::endl;
						std::cout << "current camera is : camera " << current_corrected_cam << std::endl;
					}
					if (!translationmode){
						translationmode = true;
						rotationmode = false;
						current_corrected_cam = 0;
						std::cout << "cam::" << translationmode << std::endl;
					}
					


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









			if ((!no_controller  && ci == 2)|| (no_controller &&ci==0))
			{
				
				camera_pos_1=vrpe.get_position();
				camera_ori_1 = vrpe.get_orientation();


				/*mat3 cam_ori;
				cam_ori = vrpe.get_orientation();*/
				/*camera_ori_1(0, 0) = cam_ori(0, 0);
				camera_ori_1(0, 1) = cam_ori(0, 1);
				camera_ori_1(0, 2) = cam_ori(0, 2);
				camera_ori_1(1, 0) = cam_ori(2, 0);
				camera_ori_1(1, 1) = cam_ori(2, 1);
				camera_ori_1(1, 2) = cam_ori(2, 2);*/
				/*camera_ori_1(2, 0) = cam_ori(1, 0);
				camera_ori_1(2, 1) = cam_ori(1, 1);
				camera_ori_1(2, 2) = cam_ori(1, 2);*/

				
			}
			if ((!no_controller  && ci == 3) || (no_controller  && ci == 1))
			{

				camera_pos_2 = vrpe.get_position();
				camera_ori_2 = vrpe.get_orientation();

			}
			if ((no_controller == false && ci == 4) || (no_controller == true && ci == 2))
			{

				camera_pos_3 = vrpe.get_position();
				camera_ori_3 = vrpe.get_orientation();

			}













			//if (ci == rgbd_controller_index) {
			//	//controller_orientation = vrpe.get_orientation();
			//	//controller_position = vrpe.get_position();
			//}




			if (ci != -1) {
				if (state[ci] == IS_GRAB) {
					// in grab mode apply relative transformation to grabbed boxes

					// get previous and current controller position
					vec3 last_pos = vrpe.get_last_position();
					vec3 pos = vrpe.get_position();
					// get rotation from previous to current orientation
					// this is the current orientation matrix times the
					// inverse (or transpose) of last orientation matrix:
					// vrpe.get_orientation()*transpose(vrpe.get_last_orientation())
					mat3 rotation = vrpe.get_rotation_matrix();
					// iterate intersection points of current controller
					for (size_t i = 0; i < intersection_points.size(); ++i) {
						if (intersection_controller_indices[i] != ci)
							continue;
						// extract box index
						unsigned bi = intersection_box_indices[i];
						// update translation with position change and rotation
						movable_box_translations[bi] = 
							rotation * (movable_box_translations[bi] - last_pos) + pos;
						// update orientation with rotation, note that quaternions
						// need to be multiplied in oposite order. In case of matrices
						// one would write box_orientation_matrix *= rotation
						movable_box_rotations[bi] = quat(rotation) * movable_box_rotations[bi];
						// update intersection points
						intersection_points[i] = rotation * (intersection_points[i] - last_pos) + pos;
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

		
		if (!sky_prog.is_created()) {
			sky_prog.build_program(ctx, "glsl/sky.glpr");
			img_tex.create_from_images(ctx, data_dir + "/skybox/cm_{xp,xn,yp,yn,zp,zn}.jpg");//
			
		}
		
		return true;

		
			
}
void vr_rgbd::clear(cgv::render::context& ctx)
{
		cgv::render::ref_point_renderer(ctx, -1);
		cgv::render::ref_box_renderer(ctx, -1);
		cgv::render::ref_sphere_renderer(ctx, -1);
		sky_prog.destruct(ctx);
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

void vr_rgbd::draw(cgv::render::context& ctx)
{
		if (show_points) {
			auto& pr = cgv::render::ref_point_renderer(ctx);
			pr.set_render_style(point_style);
			pr.set_y_view_angle((float)vr_view_ptr->get_y_view_angle());
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
			if (state_ptr) {
				for (int ci = 0; ci < 2; ++ci) if (state_ptr->controller[ci].status == vr::VRS_TRACKED) {
					vec3 ray_origin, ray_direction;
					state_ptr->controller[ci].put_ray(&ray_origin(0), &ray_direction(0));
					P.push_back(ray_origin);
					P.push_back(ray_origin + ray_length * ray_direction);
					rgb c(float(1 - ci), 0.5f*(int)state[ci], float(ci));
					C.push_back(c);
					C.push_back(c);
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

			




		//// draw static boxes
		//cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
		//renderer.set_render_style(style);
		//renderer.set_box_array(ctx, boxes);
		//renderer.set_color_array(ctx, box_colors);
		//if (renderer.validate_and_enable(ctx)) {
		//	glDrawArrays(GL_POINTS, 0, (GLsizei)boxes.size());
		//}
		//renderer.disable(ctx);

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



}





#include <cgv/base/register.h>
//cgv::base::object_registration<vr_rgbd> vr_rgbd_reg("vr_rgbd");
cgv::base::object_registration<vr_rgbd> vr_rgbd_reg("");
///@}