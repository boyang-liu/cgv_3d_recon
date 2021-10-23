#include "voxel_renderer.h"
#include <cgv_gl/gl/gl.h>
#include <cgv_gl/gl/gl_tools.h>

namespace cgv {
	namespace render {

		voxel_render_style::voxel_render_style()
		{
			default_extent = vec3(1.0f);
			relative_anchor = vec3(0.0f);
		}

	

		voxel_renderer& ref_voxel_renderer(context& ctx, int ref_count_change)
		{
			static int ref_count = 0;
			static voxel_renderer r;
			r.manage_singleton(ctx, "voxel_renderer", ref_count, ref_count_change);
			return r;
		}


		render_style* voxel_renderer::create_render_style() const
		{
			return new surface_render_style();
		}


		voxel_renderer::voxel_renderer()
		{
			has_extents = false;
			position_is_center = true;
			has_translations = false;
			has_rotations = false;
		}

		void voxel_renderer::enable_attribute_array_manager(const context& ctx, attribute_array_manager& aam)
		{
			surface_renderer::enable_attribute_array_manager(ctx, aam);
			if (has_attribute(ctx, "extent"))
				has_extents = true;
			if (has_attribute(ctx, "translation"))
				has_translations = true;
			if (has_attribute(ctx, "rotation"))
				has_rotations = true;
		}
		void voxel_renderer::disable_attribute_array_manager(const context& ctx, attribute_array_manager& aam)
		{
			surface_renderer::disable_attribute_array_manager(ctx, aam);
			has_extents = false;
			has_translations = false;
			has_rotations = false;
		}

		void voxel_renderer::set_position_is_center(bool _position_is_center)
		{
			position_is_center = _position_is_center;
		}

		bool voxel_renderer::build_shader_program(context& ctx, shader_program& prog, const shader_define_map& defines)
		{
			return prog.build_program(ctx, "glsl/voxelize.glpr", true, defines);
		}

		bool voxel_renderer::enable(context& ctx)
		{
			if (!surface_renderer::enable(ctx))
				return false;
			ref_prog().set_uniform(ctx, "position_is_center", position_is_center);
			ref_prog().set_uniform(ctx, "has_rotations", has_rotations);
			ref_prog().set_uniform(ctx, "has_translations", has_translations);
			const auto& brs = get_style<voxel_render_style>();
			ref_prog().set_uniform(ctx, "relative_anchor", brs.relative_anchor);
			if (!has_extents)
				ref_prog().set_attribute(ctx, "extent", brs.default_extent);
			return true;
		}
		bool voxel_renderer::disable(context& ctx)
		{
			if (!attributes_persist()) {
				has_extents = false;
				position_is_center = true;
				has_rotations = false;
				has_translations = false;
			}

			return surface_renderer::disable(ctx);
		}


		void voxel_renderer::draw(context& ctx, size_t start, size_t count, bool use_strips, bool use_adjacency, uint32_t strip_restart_index)
		{
			draw_impl(ctx, PT_POINTS, start, count, false, false, -1);
		}

		bool voxel_render_style_reflect::self_reflect(cgv::reflect::reflection_handler& rh)
		{
			return
				rh.reflect_base(*static_cast<surface_render_style*>(this)) &&
				rh.reflect_member("default_extent", default_extent) &&
				rh.reflect_member("relative_anchor", relative_anchor);
		}


		cgv::reflect::extern_reflection_traits<voxel_render_style, voxel_render_style_reflect> get_reflection_traits(const voxel_render_style&)
		{
			return cgv::reflect::extern_reflection_traits<voxel_render_style, voxel_render_style_reflect>();
		}















	}

}