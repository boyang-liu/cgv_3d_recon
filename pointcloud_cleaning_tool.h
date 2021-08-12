#pragma once

#include <cgv/base/node.h>
#include <cgv/math/fvec.h>
#include <cgv/media/color.h>
#include <cgv/gui/event_handler.h>
#include <cgv/gui/provider.h>
#include <cgv/data/data_view.h>
#include <cgv/render/drawable.h>
#include <cgv/render/shader_program.h>
#include <cgv/render/texture.h>
#include <cgv/render/render_types.h>
#include <cgv_gl/point_renderer.h>
#include <cgv_gl/rounded_cone_renderer.h>
#include <cgv_gl/surfel_renderer.h>
#include <cgv_gl/clod_point_renderer.h>
#include <cgv/render/shader_program.h>
#include <cgv_gl/sphere_renderer.h>
#include <cgv_gl/arrow_renderer.h>
#include <cgv_gl/box_wire_renderer.h>

#include <point_cloud.h>
#include <octree.h>
#include "chunks.h"
#include "history.h"
#include "buffer_action_structs.h"
#include "intersection.h"

#include <string>
#include <sstream>
#include <mutex>
#include <future>
#include <unordered_set>
#include <typeindex>
#include <array>

// these are the vr specific headers
#include <vr/vr_driver.h>
#include <cg_vr/vr_server.h>
#include <vr_view_interactor.h>
#include <vr_render_helpers.h>
#include <cg_vr/vr_events.h>

#include "lib_begin.h"

enum class LoDMode {
	OCTREE = 1,	//good quality lods
	RANDOM_POISSON = 2, //lower quality but generation is way faster
	INVALID = -1
};

enum class point_label {
	DELETED = 0x00000000,	//deleted points must not be in any group
	VISIBLE = 0x00000001,	//meber of first group with normal clod rendering (the default group) and label 0
	SELECTED_BIT = 0x00000002,  //points with a special highlighting
	GROUP_MASK = 0x0000FFFF,// there are 16 point groups. A point can be in all of them simultaneously, groups can be used to better implement selection mechanisms
	LABEL_MASK = 0xFFFF0000,// there are also 16 bit that can be used for labels
	UNDEFINED = -1,
	//some examples of label definitions
	LABEL_0 = 0x00000001, //normal visible point with label 0
	LABEL_1 = 0x00010001, //normal visible point with label 1
	LABEL_2 = 0x00020001,
	LABEL_3 = 0x00030001,
	LABEL_4 = 0x00040001,
	LABEL_5 = 0x00050001,
	LABEL_6 = 0x00060001,
	LABEL_7 = 0x00070001,
	LABEL_8 = 0x00000001,
	LABEL_9 = 0x00010001,
	LABEL_10 = 0x00020001,
	LABEL_11 = 0x00030001,
	LABEL_12 = 0x00040001,
	LABEL_13 = 0x00050001,
	LABEL_14 = 0x00060001,
	LABEL_15 = 0x00070001
};

/// encodes a number as label
inline uint32_t make_label(uint32_t label_id, const point_label& group) {
	return (label_id << 16) | ((uint32_t)group &  (uint32_t)point_label::GROUP_MASK);
}
/// decodes a number from a label by removing the group information
inline uint32_t id_from_label(uint32_t label) {
	return label >> 16;
}

enum InteractionMode {
	TELEPORT = 0,
	LABELING = 1,
	TRANSFORMING = 2,
	CONFIG = 3,
	NUM_OF_INTERACTIONS
};

class pointcloud_cleaning_tool;




struct indexed_point : public cgv::render::clod_point_renderer::Point {
	size_t index;
};

enum point_attributes {
	PA_LABELS = 1,
	PA_POSITIONS = 2,
	PA_COLORS = 4,
	PA_LODS = 8,
	PA_ALL = -1
};

enum selection_shape {
	SS_NONE = 0,
	SS_SPHERE = 1,
	SS_PLANE = 2,
	SS_CUBOID = 3,
	NUM_OF_SHAPES
};

// this is a VR application using the clod_point_renderer for demonstration purposes
class pointcloud_cleaning_tool :
	public cgv::base::node,
	public cgv::render::drawable,
	public cgv::gui::event_handler,
	public cgv::gui::provider
{
public:
	//use internal point format
	using LODPoint = cgv::render::clod_point_renderer::Point;

	pointcloud_cleaning_tool();

	/// overload to return the type name of this object. By default the type interface is queried over get_type.
	std::string get_type_name() const { return "pointcloud_cleaning_tool"; }

	bool self_reflect(cgv::reflect::reflection_handler& rh);

	void on_set(void* member_ptr);
	
	void on_register();
	
	void unregister();

	bool init(cgv::render::context& ctx);

	void init_frame(cgv::render::context& ctx);
	/// overload to draw the content of this drawable
	void draw(cgv::render::context& ctx);
	///
	void finish_draw(cgv::render::context& ctx);
	/// helper functions for gui rendering 
	void render_a_handhold_arrow(cgv::render::context& ctx, rgb c, float r);
	///
	void render_palette_sphere_on_rhand(cgv::render::context& ctx);
	///
	void render_palette_for_labeling_on_lhand(cgv::render::context& ctx);
	///
	void render_palette_plane_on_rhand(cgv::render::context& ctx);
	///
	void render_palette_cube_on_rhand(cgv::render::context& ctx);
	///
	void clear(cgv::render::context& ctx);
	/// 
	bool handle(cgv::gui::event& e);
	///
	void schedule_a_labeling_of_points(int32_t label, vec3 position, float radius, int32_t group_mask);
	///
	void schedule_a_labeling_of_points_in_cube(int32_t label, vec3 position, vec3 max_ext, vec3 min_ext, int32_t group_mask);
	/// 
	void schedule_a_labeling_of_points_by_clipping_plane(int32_t label, vec3 position, mat3 plane_ori, int32_t group_mask);
	///
	void schedule_a_labeling_of_selected_points(int32_t new_label, int32_t expected_label, int32_t group_mask);
	///
	void schedule_a_rollback();
	///
	void stream_help(std::ostream& os);
	///
	void colorize_with_height();
	///
	void print_point_cloud_info();
	///
	void compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color);
	///
	void create_gui();

protected:
	// move reshaped and reordered point cloud data to opengl buffers, does octree generation if required
	// requires source_pc_labels to hold the labels of the points in source_pc, source_pc_labels can be deleted afterwarts
	void prepare_point_cloud() noexcept;

	void on_point_cloud_fit_table();

	void automatic_scale_to_fit_table();
	
	void on_load_point_cloud_cb();
	void parallel_saving();
	void on_save_point_cloud_cb();
	void on_clear_point_cloud_cb();
	void on_randomize_position_cb();
	void on_point_cloud_style_cb();
	void on_lod_mode_change();
	void on_move_to_center();

	void construct_table(float tw, float td, float th, float tW);
	void construct_room(float w, float d, float h, float W, bool walls, bool ceiling);
	void construct_environment(float s, float ew, float ed, float w, float d, float h);
	void build_scene(float w, float d, float h, float W, float tw, float td, float th, float tW);
	void clear_scene();
	void build_test_object_32();

	point_cloud build_test_point_cloud(int x, int y, int z, int grid_size, float cube_size);

	/// assigns labels to points inside the sphere defined by position and radius
	void label_points_in_sphere(cgv::render::context& ctx, const GLint label, const int32_t point_group_mask, vec3 position, const float radius, const GLuint points);
	/// assigns labels to points inside the cube defined by position and length
	void label_points_in_cube(cgv::render::context& ctx, const GLint label, const int32_t point_group_mask, vec3 position, const vec3 max_extent, const vec3 min_extent, const GLuint points);
	/// assigns labels to points on one side of clipping plane
	void label_points_by_clipping(cgv::render::context& ctx, const GLint label, const int32_t point_group_mask, vec3 position, mat3 plane_ori, const GLuint points);

	/// processes all point of the point cloud, assigns new labels to points where point_label&point_group_mask == expected_label
	/// this enables for example deletion of marked points (new_label = point_label::DELETED)
	void label_selected_points(cgv::render::context& ctx, const GLint new_label, const GLint expected_label, const int32_t point_group_mask, const GLuint points, const unsigned num_points);

	/// a quick test that enables to label the point cloud without a VR device 
	void test_labeling_of_points();

	// revert last labeling
	void rollback_last_operation(cgv::render::context& ctx);

	// copies data from the label buffer into the point_labels vector
	void sync_data(int flags= point_attributes::PA_ALL);
	
	// write points from chunks to a point_cloud object
	void copy_chunks_to_point_cloud(point_cloud& dest);

	//convert any point format compliant with the octree generator to a lod point
	template <typename T>
	static std::vector<LODPoint> to_lod_points(const std::vector<T>& pnts) {
		std::vector<LODPoint> lod_pnts;
		lod_pnts.reserve(pnts.size());

		for (auto& pnt : pnts) {
			lod_pnts.emplace_back();
			LODPoint& p = lod_pnts.back();
			p.position() = pnt.position();
			p.color() = pnt.color();
			p.level() = pnt.level();
		}
		return lod_pnts;
	}
private:
	struct vertex {
		vec3  position;
		float radius;
		rgba  color;
	};

	std::unordered_set<void*> rebuild_ptrs;

	std::string ply_path;
	point_cloud source_pc;
	cgv::render::point_render_style source_prs;
	cgv::render::surfel_render_style source_srs;
	cgv::render::rounded_cone_render_style rcrs;
	cgv::render::sphere_render_style srs;
	cgv::render::arrow_render_style ars;

	size_t max_points = 1000000;
	float radius_for_test_labeling;

	float rot_intensity;
	float trans_intensity;
	bool renderer_out_of_date = true;
	bool recolor_point_cloud = true;
	int data_sync_attributes = 0;
	int data_sync_mode = 0;

	bool pointcloud_fit_table;
	bool put_on_table;
	bool color_based_on_lod;
	bool show_environment;
	bool show_table;
	bool enable_label_history;
	bool all_chunks_are_considered;

	static constexpr int num_culling_protection_zones = 2;

	static constexpr float min_level_hue = 230.0/360.0;
	static constexpr float max_level_hue = 1.0;
	
	int lod_mode = (int)LoDMode::OCTREE;
	int interaction_mode = (int)InteractionMode::TELEPORT;

	bool gui_model_positioning = false;
	//float model_scale = 1.f;
	//vec3 model_position= vec3(0);
	//vec3 model_rotation = vec3(0);
	dmat4 model_transform;
	dmat4 addi_model_transform;
	dmat4 concat_mat;
	double auto_positioning_scale = 1.f;
	bool recompute_centroid;
	int max_num_chunks = 100;

	//cgv::render::clod_point_render_style cp_style;
	cgv::render::rounded_cone_render_style cone_style;
	

	// environment geometry
	std::vector<box3> boxes;
	std::vector<rgb> box_colors;
	cgv::render::box_render_style style;
	constexpr static float table_height = 0.7f;

	// octree base lod generator
	cgv::pointcloud::octree_lod_generator<indexed_point> lod_generator;
	// storage for generated lod points
	chunks<LODPoint> chunked_points;

	vr_view_interactor* vr_view_ptr;

	vec3 pos;
	mat3 ori;
	vec3 c_pos;

	vertex p;

	std::chrono::duration<double> diff0_ave, diff1_ave, diff2_ave, diff3_ave;
	int idiff = 0;

	//coordinate of the controller
	std::vector<vertex> coordinate_c;

	std::vector<vec3> points;
	std::vector<rgb> colors;
	std::vector<vec3> normals;

	std::vector<vec3> forward_points;
	std::vector<rgb> forward_colors;
	std::vector<vec3> forward_normals;

	/* stuff regarding point labels */
	
	//shader programs for labeled point rendering with sphere, cuboid and plane selection
	cgv::render::shader_program labeled_points_ss_draw_prog, labeled_points_sc_draw_prog, labeled_points_sp_draw_prog, labeled_points_sn_draw_prog;
	cgv::render::shader_program labeling_tool_prog, selection_relabel_prog;
	cgv::render::shader_program label_rollback_prog;
	bool use_label_prog = true;

	std::vector<GLint> point_labels; // point labels that downloaded from GPU
	GLuint point_label_buffer = 0;
	// stores the old labels and indices of points affected by the last labeling action
	history label_history;

	bool init_label_buffer = true;

	bool gui_culling_protection_zone = false;
	
	/*gui rendering */
	//float point_selection_radii[2];
	static constexpr unsigned point_selection_hand = 1; //which hand is used for point selection (0=left, 1=right)
	vec3 point_selection_center[2];
	mat3 point_selection_center_ori[2];
	int32_t point_selection_group_mask[2];
	render_types::rgba point_selection_colors[2];

	vec3 initial_offset_rhand;
	vec3 curr_offset_rhand;
	cgv::render::sphere_render_style sphere_style_rhand;
	cgv::render::sphere_render_style sphere_style_lhand;
	cgv::render::box_render_style cube_style_rhand;
	cgv::render::box_render_style plane_style_rhand;
	//cube definition
	float cube_length;
	box3 cube_rhand;
	quat cube_ori_right;

	//plane definition
	box3 plane_rhand;
	box3 plane_right;
	/// selection plane orientation (rotate vec3(1,0,0) for the normal)
	quat plane_ori_right;
	
	std::vector<rgba> PALETTE_COLOR_MAPPING;
	std::vector<vec3> palette_lefthand_object_positions;
	std::vector<vec3> palette_lefthand_palette_initialpose_positions;
	std::vector<rgba> palette_lefthand_object_colors;
	//picked_sphere_index means the index of sphere that is rendered on left hand
	int picked_sphere_index;
	int picked_label;
	selection_shape point_selection_shape; //tool shape for point selection
	float radius_adjust_step;
	
	bool gui_chunks = true;
	bool use_chunks = false;
	bool draw_chunk_bounding_boxes = false;
	float chunk_cube_size = 1.0;
	bool auto_chunk_cube_size = true;
	std::stringstream chunk_render_message_ss;
	std::string chunk_render_message;

	// queue for stored actions
	//std::vector<point_labeling_tool_commit> queued_actions;
	std::vector<std::unique_ptr<buffer_action>> queued_actions;

	// IO 
	std::thread* writing_thread;

	// controller positions and orientations
	std::array<mat34,2> controller_poses;

	float pre_dist_btw_two_hands, dist_btw_two_hands;
	bool is_scaling;
	bool is_rotating_moving;
	bool is_tp_floor;
	std::vector<box3> floor_box;
	std::vector<rgb> floor_box_clr;

	// intersection points
	std::vector<vec3> intersection_points;
	std::vector<rgb>  intersection_colors;
	std::vector<int>  intersection_box_indices;
	std::vector<int>  intersection_controller_indices;
};

#include <cgv/config/lib_end.h>