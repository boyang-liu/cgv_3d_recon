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
#include <cgv/media/mesh/simple_mesh.h>
#include <cgv_gl/gl/mesh_render_info.h>
#include <rgbd_input.h>
#include <random>
#include <future>
#include <iostream>
#include <chrono>
#include <cg_vr/vr_events.h>
#include <vr/vr_state.h>
#include <vr/vr_kit.h>
#include <vr/vr_driver.h>
#include <cgv/defines/quote.h>

#include <vr/vr_driver.h>
#include <cg_vr/vr_server.h>
#include <vr_view_interactor.h>
#include "intersection.h"
#include "k4a/k4a.h"
#include <plugins/rgbd_kinect_azure/rgbd_kinect_azure.h>
#include "ICP.h"
#include "rgbd_pointcloud.h"
#include "PCBoundingbox.h"
#include "GoICP.h"

using namespace rgbd;



/// the plugin class vr_rgbd inherits like other plugins from node, drawable and provider
class vr_rgbd :
	public cgv::base::node,
	public cgv::render::drawable,
	public cgv::gui::event_handler,
	public cgv::gui::provider
{
protected:
	enum InteractionState
	{
		IS_NONE,
		IS_OVER,
		IS_GRAB
	};
	
	struct vertex : public cgv::render::render_types
	{
		vec3  point;
		rgba8 color;
	};
	/// internal members used for data storage
	rgbd::frame_type color_frame, depth_frame, warped_color_frame;
	rgbd::frame_type color_frame_2, depth_frame_2, ir_frame_2, warped_color_frame_2;


	rgbd::frame_type mycolor_frame, mydepth_frame;

	std::vector<rgbd::frame_type> color_frames, depth_frames, warped_color_frames;
	std::vector<rgbd::frame_type> color_frames_2, depth_frames_2, warped_color_frames_2;
	///
	size_t nr_depth_frames;
	size_t nr_color_frames;
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
	std::vector<vertex> intermediate_pc, current_pc, test_pc;
	std::vector<std::vector<vertex>> cur_pc;


	/// list of recorded point clouds
	std::vector<std::vector<vertex> > recorded_pcs;
	/// translations of recorded point clouds
	std::vector<quat> rotations;
	/// rotations of recorded point clouds
	std::vector<vec3> translations;
	/// rendering style for points
	cgv::render::point_render_style point_style;
	///counter of storing point cloud

	///counter of pc
	int counter_pc;
	///registration
	bool registration_started;

	int rgbd_controller_index;
	/// current pose of the controller
	mat3 controller_orientation;
	vec3 controller_position;
	/// pose of controller when last point cloud was acquire; this is used for contruction of point cloud in parallel thread
	mat3 controller_orientation_pc;
	vec3 controller_position_pc;
	/// current calibration pose mapping from rgbd coordinates to controller coordinates 
	mat3 rgbd_2_controller_orientation;
	vec3 rgbd_2_controller_position;

	//mat3 rgbd_2_controller_orientation2;
	//vec3 rgbd_2_controller_position2;

	/// calibration pose mapping from rgbd coordinates to controller coordinates stored at the time when freezing the point cloud for calibration
	mat3 rgbd_2_controller_orientation_start_calib;
	vec3 rgbd_2_controller_position_start_calib;

	//mat3 rgbd_2_controller_orientation_start_calib2;
	//vec3 rgbd_2_controller_position_start_calib2;

	///
	bool show_points;
	unsigned max_nr_shown_recorded_pcs;
	bool trigger_is_pressed;
	float recording_fps;
	///
	std::future<size_t> future_handle;

	/// 

	/// path to be set for pc files
	std::string pc_file_path;
	///
	bool rgbd_started;
	bool rgbd_multi_started;
	bool all_devices_attached;
	std::string rgbd_protocol_path;
	bool get_tracker_positions;
	/// 
	rgbd::rgbd_input rgbd_inp;

	// store the scene as colored boxes
	std::vector<box3> boxes;
	std::vector<rgb> box_colors;
	rgbd::rgbd_kinect_azure rka_inp;
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
	std::vector<rgb>  intersection_colors;
	std::vector<int>  intersection_box_indices;
	std::vector<int>  intersection_controller_indices;

	// state of current interaction with boxes for each controller
	InteractionState state[4];

	// render style for interaction
	cgv::render::sphere_render_style srs;
	cgv::render::box_render_style movable_style;

	// compute intersection points of controller ray with movable boxes
	void compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color);

	/// register on device change events
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

	void generate_point_cloud(std::vector<vertex>& pc);
	/// start the rgbd device
	void start_rgbd();
	void start_multi_rgbd();
	void attach_all_devices();
	void stop_all_rgbd();
	
	void detach_all_devices();
	/// stop rgbd device
	void stop_rgbd();
	void set_rgbd_pos();

	bool coarseregistrationmode;
	std::vector<vec3> cam_coarse_t;
	std::vector<mat3> cam_coarse_r;

	
	
	std::vector<vec3> camera_pos;
	std::vector<mat3> camera_ori;

	std::vector<vec3> viewconepos1;
	std::vector<vec3> viewconepos2;

	bool no_controller;
	std::vector<vec3> manualcorrect_translation;
	std::vector<vec3> manualcorrect_rotation;
	float position_scale;
	float rotation_scale;
	//controller mode for immersive interaction
	bool translationmode;
	bool rotationmode;
	bool setboundingboxmode;
	bool selectPointsmode;
	bool boundingboxisfixed;

	bool record_pc_started;
	int current_corrected_cam;
	int depth_stream_format_idx;
	int color_stream_format_idx;
	int ir_stream_format_idx;
	std::vector<rgbd::stream_format> depth_stream_formats;
	std::vector<rgbd::stream_format> color_stream_formats;	
	//std::vector<rgbd::stream_format> ir_stream_formats;


	cgv::render::shader_program sky_prog;
	cgv::render::shader_program floor_prog;
	cgv::render::texture flo_tex;
	cgv::render::texture img_tex;
	//mat4 mvp;
	char* cgv_data = getenv("CGV_DATA");
	std::string data_dir = std::string(cgv_data);
	bool generate_pc_from_rgbd;
	
	std::vector<std::shared_ptr<ann_tree>> trees;

	int save_time = 0;
	int currentpointcloud;
	int currentcamera;

	int num_recorded_pc;
	int num_loaded_pc;
	std::string pc_load_dir;
	std::vector<rgbd_pointcloud> intermediate_rgbdpc;
	std::vector<rgbd_pointcloud> rgbdpc;
	
	std::vector<rgbd_pointcloud> rgbdpc_in_box;
	std::vector<int> knn;
	float Radius_SelectMode;
	float sphere_distance ;
	
	float BoundingBoxlength;
	float BoundingBoxheight;
	PCBoundingbox pcbb;
	std::vector<cgv::math::fmat<float, 3, 3>> cam_rotation;
	std::vector < cgv::math::fvec<float, 3>> cam_translation;

	vec3 viewpoint1 =vec3(0,0,0);
	vec3 viewpoint2 = vec3(0, 0, 0);



	//mat3 testmat;
	//vec3 testtran;



public:
	vr_rgbd();
	~vr_rgbd();
	size_t construct_point_cloud();
	//for multiple devices' point cloud
	size_t construct_multi_point_cloud(int index);//
	//size_t construct_point_clouds();

	void getviewconeposition(vec3 &a, mat3 r, vec3 t);

	void save_all_pc();
	void Record_PC_FromOneCam(int cam);
	void Record_PC_FromAllcams();
	void load_pc();
	void load_recorded_pc();
	void clear_current_point_cloud();

	void temp_test();
	
	frame_type read_rgb_frame();
	//should be a thread
	frame_type read_depth_frame();
	///cast vertex to point_cloud
	//void copy_pointcloud(const std::vector<vertex> input, point_cloud& output);
	///cast point_cloud to vertex
	//void pc2vertex(const point_cloud& input, std::vector<vertex>& output);
	//void write_pcs_to_disk(int i);
	//size_t read_pc_queue(const std::string filename, std::string content);
	
	void registerPointCloud(rgbd_pointcloud target, rgbd_pointcloud &source, cgv::math::fmat<float, 3, 3>& r, cgv::math::fvec<float, 3>& t);
	//void generate_pc(std::vector<vertex>, rgbd_pointcloud& pc1);
	void vr_rgbd::build_tree_feature_points(rgbd_pointcloud& pc1, int i);
	void select_feature_points(rgbd_pointcloud& pc1, vec3 p, float radius);
	void cancell_selected_feature_points(rgbd_pointcloud& pc1, vec3 p, float radius);
	//void start_select_points();


	//void generate_rdm_pc(point_cloud& pc1, point_cloud& pc2);
	//void  test_icp();
	//void construct_TSDtree();
	bool record_this_frame(double t);
	void timer_event(double t, double dt);
	std::string get_type_name() const;
	void create_gui();
	bool self_reflect(cgv::reflect::reflection_handler& rh);
	void on_set(void* member_ptr);
	void stream_help(std::ostream& os);
	bool handle(cgv::gui::event& e);
	bool init(cgv::render::context& ctx);
	void clear(cgv::render::context& ctx);
	rgbd_pointcloud setboundingbox(rgbd_pointcloud pc1, vec3 pos1, vec3 pos2);
	void draw_boudingbox(cgv::render::context& ctx, vec3& pos1, vec3& pos2);
	void draw_pc(cgv::render::context& ctx, const std::vector<vertex>& pc);
	void draw_rgbdpc(cgv::render::context& ctx, const rgbd_pointcloud& pc);
	void draw_selected_rgbdpc(cgv::render::context& ctx, const rgbd_pointcloud& pc);
	void draw(cgv::render::context& ctx);
	void draw_viewingcone(cgv::render::context& ctx, int cc, std::vector<vec3>& P, std::vector<rgb>& C);
	enum DeviceMode {No_Device,Protocol,Has_Device};

	 

private:
	int device_idx;
	DeviceMode device_mode;
	int num_devices;

	//std::vector<int> device_indices ;
protected:
	void device_select();
	void update_stream_formats();
	//void set_devices();
	//void capture_multi_device();
};