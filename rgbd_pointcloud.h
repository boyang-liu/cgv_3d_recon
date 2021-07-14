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
#include <point_cloud/ICP.h>
#include <cg_vr/vr_events.h>
#include <vr/vr_state.h>
#include <vr/vr_kit.h>
#include <vr/vr_driver.h>
#include <cgv/defines/quote.h>

#include <vr/vr_driver.h>
#include <cg_vr/vr_server.h>
#include <vr_view_interactor.h>




class rgbd_pointcloud :public cgv::render::drawable
{

public:
	std::vector<vec3> Points;
	std::vector<rgba8> Colors;

	bool write_pc(const std::string& file_name);
	bool read_pc(const std::string& file_name);
	size_t add_point(const vec3& p);
	size_t add_point(const vec3& p, const rgba8& c);
	bool read_lbypc(const std::string& file_name);
	bool write_lbypc(const std::string& file_name);
	bool read_txt(const std::string& file_name);
	bool write_txt(const std::string& file_name);
	int get_nr_Points();

protected:

	//bool read(const std::string& file_name);

private:

};

