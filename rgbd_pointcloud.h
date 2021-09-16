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
#define BYTE_COLORS
struct point_cloud_types
{
	/// common type for point, texture und normal coordinates
	typedef float Crd;
#ifdef BYTE_COLORS
	/// type of color components
	typedef cgv::type::uint8_type ClrComp;
	static ClrComp byte_to_color_component(cgv::type::uint8_type c) { return c; }
	static ClrComp float_to_color_component(double c) { return cgv::type::uint8_type(255 * c); }
	static cgv::type::uint8_type color_component_to_byte(ClrComp c) { return c; }
	static float color_component_to_float(ClrComp c) { return 1.0f / 255 * c; }
#else
	/// type of color components
	typedef float ClrComp;
	static ClrComp byte_to_color_component(cgv::type::uint8_type c) { return c * 1.0f / 255; }
	static ClrComp float_to_color_component(double c) { return float(c); }
	static cgv::type::uint8_type color_component_to_byte(ClrComp c) { return cgv::type::uint8_type(255 * c); }
	static float color_component_to_float(ClrComp c) { return c; }
#endif // BYTE_COLORS
	/// floating point color type
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
	struct component_info
	{
		std::string name;
		size_t index_of_first_point;
		size_t nr_points;
		component_info(size_t _first = 0, size_t _nr = 0) : index_of_first_point(_first), nr_points(_nr) {}
	};
};


class rgbd_pointcloud : public point_cloud_types//:public cgv::render::drawable
{

public:
	

	bool write_pc(const std::string& file_name);
	bool read_pc(const std::string& file_name);
	size_t add_point(const Pnt& p);
	size_t add_point(const Pnt& p, const Rgba& c);
	bool read_lbypc(const std::string& file_name);
	bool write_lbypc(const std::string& file_name);
	bool read_txt(const std::string& file_name);
	bool write_txt(const std::string& file_name);

	Cnt get_nr_Points() const { return (Cnt)Points.size(); };
	const Pnt& pnt(size_t i) const { return Points[i]; }
	Pnt& pnt(size_t i) { return Points[i]; }

	void resize(size_t nr_points);
	void clear();
	void set_render_color();
	void merge_labels(std::vector<int>& a);
	void delete_labels(std::vector<int>& a);

	const Rgba& clr(size_t i) const { return Colors[i] ; }
	/// return i-th color as reference
	Rgba& clr(size_t i) { return Colors[i]; }

	const Rgba& render_clr(size_t i) const { return renderColors[i]; }
	
	
	const Cnt& lab(size_t i) const { return labels[i]; }
	std::vector<int> labels;

	const std::vector<Pnt>& getPoints() const { return Points; }
	const std::vector<Rgba>& getColors() const { return Colors; }
	const std::vector<Rgba>& getrenderColors() const { return renderColors; }
	void do_transformation(Mat& rotation_mat, Pnt translation_vec);
	void rgbd_pointcloud::do_transformation(Mat& rotation_mat);
	void rgbd_pointcloud::do_transformation(Pnt translation_vec);

	void append(const rgbd_pointcloud& pc);


protected:

	//bool read(const std::string& file_name);
	std::vector<Pnt> Points;
	std::vector<Rgba> Colors;

	
	std::vector<Rgba> renderColors;
private:

};

