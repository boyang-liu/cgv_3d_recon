#pragma once

#include <cgv/base/node.h>
#include <cgv/signal/rebind.h>
#include <cgv/base/register.h>
#include <cgv/gui/event_handler.h>
#include <cgv/gui/trigger.h>
#include <cgv/math/ftransform.h>
#include <cgv/utils/scan.h>
#include <cgv/utils/options.h>
#include <cgv/gui/provider.h>
#include <cgv/render/drawable.h>
#include <cgv/render/shader_program.h>
#include <cgv/render/frame_buffer.h>
#include <cgv/render/attribute_array_binding.h>
#include <cgv_gl/box_renderer.h>
#include <cgv_gl/point_renderer.h>
#include <cgv_gl/sphere_renderer.h>
#include <cgv_gl/arrow_renderer.h>
#include <cgv/media/mesh/simple_mesh.h>
#include <cgv_gl/gl/mesh_render_info.h>
#include <rgbd_input.h>
#include <random>
#include <future>
#include <iostream>
#include <chrono>
#include <point_cloud/point_cloud.h>
#include <point_cloud/ICP.h>
#include <point_cloud/SICP.h>
#include <cg_vr/vr_events.h>
#include <vr/vr_state.h>
#include <vr/vr_kit.h>
#include <vr/vr_driver.h>
#include <cgv/defines/quote.h>
#include <numeric>
#include <random>
#include <rgbd_input.h>
#include <rgbd_mouse.h>
#include "rgbd_depth.h"
#include "PCBoundingbox.h"
#include "3DReconstruction.h"

///@ingroup VR
///@{

/**@file
   example plugin for vr usage
*/

// these are the vr specific headers
#include <vr/vr_driver.h>
#include <cg_vr/vr_server.h>
#include <vr_view_interactor.h>
#include "plugins/vr_rgbd/intersection.h"

//#include <opencv2/opencv.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/features2d/features2d.hpp>

// different interaction states for the controllers
enum InteractionState { IS_NONE, IS_OVER, IS_GRAB };

class vr_scanning : public cgv::base::node,
	public cgv::render::drawable,
	public cgv::gui::event_handler,
	public cgv::gui::provider
{
protected:
	/// internal members used for data storage
	rgbd::frame_type color_frame, depth_frame, warped_color_frame;
	rgbd::frame_type color_frame_2, depth_frame_2, ir_frame_2, warped_color_frame_2;
	///
	size_t nr_depth_frames, nr_color_frames;
	///
	bool record_frame;
	///
	bool record_all_frames;
	///
	bool record_key_frames;
	bool clear_all_frames;
	bool in_calibration;
	bool zoom_in;
	bool zoom_out;
	bool save_pointcloud;
	int key_frame_step;
	/// intermediate point cloud and to be rendered point cloud
	point_cloud intermediate_pc, current_pc;

	rgbd_depth intermediate_depth;



	/// list of recorded point clouds
	std::vector<point_cloud> recorded_pcs;
	/// list of recorded warped color frames
	std::vector<rgbd::frame_type> recorded_frames;
	/// translations of recorded point clouds
	std::vector<quat> rotations;
	/// rotations of recorded point clouds
	std::vector<vec3> translations;
	/// rendering style for points
	cgv::render::point_render_style point_style;
	/// counter of storing point cloud

	/// counter of pc
	int counter_pc;
	/// registration
	bool registration_started;

	int rgbd_controller_index;
	/// current pose of the controller
	mat3 controller_orientation;
	vec3 controller_position;
	/// pose of controller when last point cloud was acquire; this is used for contruction of point cloud in parallel
	/// thread
	mat3 controller_orientation_pc;
	vec3 controller_position_pc;
	/// current calibration pose mapping from rgbd coordinates to controller coordinates
	mat3 rgbd_2_controller_orientation;
	vec3 rgbd_2_controller_position;
	/// calibration pose mapping from rgbd coordinates to controller coordinates stored at the time when freezing the
	/// point cloud for calibration
	mat3 rgbd_2_controller_orientation_start_calib;
	vec3 rgbd_2_controller_position_start_calib;
	///
	bool show_points;
	unsigned max_nr_shown_recorded_pcs;
	bool trigger_is_pressed;
	float recording_fps;
	bool remap_color;

//	cv::Mat src_img, tgt_img;
//	std::vector<cv::Mat> record_imgs;
	bool is_filled, is_filled_2;
	///
	std::future<size_t> future_handle;
	///

	/// path to be set for pc files
	std::string pc_file_path;
	///
	bool rgbd_started;
	std::string rgbd_protocol_path;
	///
	rgbd::rgbd_input rgbd_inp;
	// store the scene as colored boxes
	std::vector<box3> boxes;
	std::vector<rgb> box_colors;

	// rendering style for boxes
	cgv::render::box_render_style style;

	// length of to be rendered rays
	float ray_length;

	// keep reference to vr_view_interactor
	vr_view_interactor* vr_view_ptr;

	// store the movable boxes
	std::vector<box3> movable_boxes;
	std::vector<rgb> movable_box_colors;
	std::vector<vec3> movable_box_translations;
	std::vector<quat> movable_box_rotations;

	// intersection points
	std::vector<vec3> intersection_points;
	std::vector<rgb> intersection_colors;
	std::vector<int> intersection_box_indices;
	std::vector<int> intersection_controller_indices;

	// camera pose points
	std::vector<vec3> cam_pose_points;
	std::vector<rgb> cam_pose_colors;
	std::vector<rgb> cam_cord_x_clr;
	std::vector<rgb> cam_cord_y_clr;
	std::vector<rgb> cam_cord_z_clr;
	std::vector<mat3> cam_pose_ori;
	std::vector<int> cam_pose_box_indices;
	std::vector<int> cam_pose_controller_indices;

	std::vector<vec3> cam_cord_x;
	std::vector<vec3> cam_cord_y;
	std::vector<vec3> cam_cord_z;

	// state of current interaction with boxes for each controller
	InteractionState state[4];

	// render style for interaction
	cgv::render::sphere_render_style srs;
	cgv::render::box_render_style movable_style;
	cgv::render::arrow_render_style ars;
	// declare aam for arrow renderer
	cgv::render::attribute_array_manager a_manager;
	// sicp;
	cgv::pointcloud::SICP::ComputationMode sicp_computation_mode;
	cgv::pointcloud::SICP sicp;

	// compute intersection points of controller ray with movable boxes
	void compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color);
	void on_device_change(void* kit_handle, bool attach);
	/// construct boxes that represent a table of dimensions tw,td,th and leg width tW
	void construct_table(float tw, float td, float th, float tW);
	/// construct boxes that represent a room of dimensions w,d,h and wall width W
	void construct_room(float w, float d, float h, float W, bool walls, bool ceiling);
	/// construct boxes for environment
	void construct_environment(float s, float ew, float ed, float eh, float w, float d, float h);
	/// construct boxes that represent a table of dimensions tw,td,th and leg width tW
	void construct_movable_boxes(float tw, float td, float th, float tW, size_t nr);
	/// construct a scene with a table
	void build_scene(float w, float d, float h, float W, float tw, float td, float th, float tW);

	void generate_point_cloud(point_cloud& pc);

	void start_rgbd();

	void stop_rgbd();

	PCBoundingbox pcbb;

	Reconstruction* recon = new  Reconstruction();

public:
	vr_scanning();
	size_t generate_depths();
	void generate_mesh();
	size_t construct_point_cloud();
	rgbd::frame_type read_rgb_frame(); // should be a thread
	rgbd::frame_type read_depth_frame();
	/// here should be const point cloud
	void write_pcs_to_disk(int i);
	void read_pc_queue(const std::string filename, std::string content);
	void registrationPointCloud();
	void generate_rdm_pc(point_cloud& pc1, point_cloud& pc2);
	void test_icp();
	void construct_TSDtree();
	bool record_this_frame(double t);
	void timer_event(double t, double dt);
	void test();
	void test2();

	std::string get_type_name() const;

	void create_gui();
	bool self_reflect(cgv::reflect::reflection_handler& rh);
	void on_set(void* member_ptr);
	void stream_help(std::ostream& os);
	bool handle(cgv::gui::event& e);
	bool init(cgv::render::context& ctx);
	void clear(cgv::render::context& ctx);
	void draw_viewingcone(cgv::render::context& ctx, std::vector<vec3>& P, std::vector<rgb>& C, mat3 r, vec3 t);
	void draw_pc(cgv::render::context& ctx, const point_cloud& pc);
	void draw_boudingbox(cgv::render::context& ctx, vec3& pos1, vec3& pos2);
	void draw(cgv::render::context& ctx);
	void on_reg_SICP_cb();
	
	void open_image();
	void change_filled();
	void change_filled_2();
};

#include <cgv/config/lib_end.h>