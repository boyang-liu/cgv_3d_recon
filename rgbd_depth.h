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
#include <cgv/media/mesh/simple_mesh.h>
#include <cgv_gl/gl/mesh_render_info.h>
#include <random>
#include <future>
#include <iostream>
#include <chrono>
//#include <point_cloud/ICP.h>
#include <cg_vr/vr_events.h>
#include <vr/vr_state.h>
#include <vr/vr_kit.h>
#include <vr/vr_driver.h>
#include <cgv/defines/quote.h>

#include <vr/vr_driver.h>
#include <cg_vr/vr_server.h>
#include <vr_view_interactor.h>
#include <numeric>
#define BYTE_COLORS
	typedef float Crd;
	typedef cgv::math::fvec<float, 3> vec3;
	/// type of color components
	typedef cgv::type::uint8_type ClrComp;
	typedef cgv::media::color<float, cgv::media::RGB, cgv::media::OPACITY> RGBA;
	/// 3d point type
	typedef cgv::math::fvec<Crd, 3> Pnt;
	/// 3d normal type
	typedef cgv::math::fvec<Crd, 3> Nml;
	/// 3d direction type
	typedef cgv::math::fvec<Crd, 3> Dir;
	/// 2d texture coordinate type
	typedef cgv::math::fvec<Crd, 2> TexCrd;
	/// 4d homogeneous vector type
	typedef cgv::math::fvec<Crd, 4> HVec;
	/// colors are rgb with floating point coordinates
	typedef cgv::media::color<ClrComp> Clr;
	/// rgba colors used for components
	typedef cgv::media::color<ClrComp, cgv::media::RGB, cgv::media::OPACITY> Rgba;
	/// 3x3 matrix type used for linear transformations
	typedef cgv::math::fmat<Crd, 3, 3> Mat;
	/// 3x4 matrix type used for affine transformations in reduced homogeneous form
	typedef cgv::math::fmat<Crd, 3, 4> AMat;
	/// 4x4 matrix type used for perspective transformations in full homogeneous form
	typedef cgv::math::fmat<Crd, 4, 4> HMat;
	/// type of axis aligned bounding box
	typedef cgv::media::axis_aligned_box<Crd, 3> Box;
	/// unsigned integer type used to represent number of points
	typedef cgv::type::uint32_type Cnt;
	/// singed index type used for interation variables
	typedef cgv::type::int32_type Idx;
	/// 2d pixel position type
	typedef cgv::math::fvec<Idx, 2> PixCrd;
	/// type of pixel coordinate range
	typedef cgv::media::axis_aligned_box<Idx, 2> PixRng;
	/// type of texture coordinate box
	typedef cgv::media::axis_aligned_box<Crd, 2> TexBox;
	/// quaternions used to represent rotations
	typedef cgv::math::quaternion<Crd> Qat;
	/// simple structure to store the point range of a point cloud component

	struct parameters {
		double fx_d, fy_d, cx_d, cy_d, depth_scale;

	};

class rgbd_depth 
{

public:
	
	rgbd_depth() {
		camera_pos = vec3(0, 0, 0);
		camera_rotation.identity();
		camera_translation = vec3(0, 0, 0);
		width = 0;
		height = 0;
		para.fx_d = 0;
		para.fy_d = 0;
		para.cx_d = 0;
		para.cy_d = 0;
		para.depth_scale = 0;



	}
	bool write_rgbd_depth(const std::string& file_name);
	bool read_rgbd_depth(const std::string& file_name);

	bool read_rd(const std::string& file_name);
	bool write_rd(const std::string& file_name);

	bool read_txt(const std::string& file_name);
	bool write_txt(const std::string& file_name);

	int get_nr_pixels() { return width * height; }

	bool V1_map_depth_to_point(int x, int y, int depth, float* point_ptr) const;

	
	parameters para;




	void clear_depth();

	std::vector<int> Pixels;
	
	int width;
	int height;

	vec3 camera_pos;
	Mat camera_rotation;
	vec3 camera_translation;


protected:
	
	
	
	


	
	
private:

};

