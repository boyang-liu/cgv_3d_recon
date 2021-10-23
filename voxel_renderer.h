#pragma once

#include <cgv_gl/surface_renderer.h>

#include <cgv_gl/gl/lib_begin.h>


namespace cgv { 
	namespace render { 	
		class CGV_API voxel_renderer;

		extern CGV_API voxel_renderer& ref_voxel_renderer(context& ctx, int ref_count_change = 0);

		/// voxels use surface render styles
		struct CGV_API voxel_render_style : public surface_render_style
		{
			/// extent used in case extent array is not specified
			vec3 default_extent;
			/// box anchor position relative to center that corresponds to the position attribute
			vec3 relative_anchor;
			/// default constructor sets default extent to (1,1,1) and relative anchor to (0,0,0)
			voxel_render_style();
		};
	
		

		class CGV_API voxel_renderer : public surface_renderer
		{
		protected:
			
			bool has_extents;
			
			bool has_translations;
			
			bool has_rotations;
			
			bool position_is_center;
			
			render_style* create_render_style() const;
			
			bool build_shader_program(context& ctx, shader_program& prog, const shader_define_map& defines);
			
			
		public:
			
			
			voxel_renderer();
			
			void enable_attribute_array_manager(const context& ctx, attribute_array_manager& aam);
			
			void disable_attribute_array_manager(const context& ctx, attribute_array_manager& aam);
			
			void set_position_is_center(bool _position_is_center);
			
			bool enable(context& ctx);
			
			template <typename T>
			void set_extent(const context& ctx, const T& extent) { has_extents = true; ref_prog().set_attribute(ctx, ref_prog().get_attribute_location(ctx, "extent"), extent); }
			
			template <typename T>
			void set_extent_array(const context& ctx, const std::vector<T>& extents) { has_extents = true;  set_attribute_array(ctx, ref_prog().get_attribute_location(ctx, "extent"), extents); }
			
			template <typename T>
			void set_extent_array(const context& ctx, const T* extents, size_t nr_elements, unsigned stride_in_bytes = 0) { has_extents = true;  set_attribute_array(ctx, ref_prog().get_attribute_location(ctx, "extent"), extents, nr_elements, stride_in_bytes); }
			
			template <typename T>
			void set_voxel(const context& ctx, const cgv::media::axis_aligned_box<T, 3>& box) {
				set_position(ctx, box.get_min_pnt());
				set_extent(ctx, box.get_max_pnt());
				set_position_is_center(false);
			}
			
			template <typename T>
			void set_voxel_array(const context& ctx, const std::vector<cgv::media::axis_aligned_box<T, 3> >& boxes) {
				set_composed_attribute_array(ctx, ref_prog().get_attribute_location(ctx, "position"),
					&boxes.front(), boxes.size(), boxes[0].get_min_pnt());
				ref_composed_attribute_array(ctx, ref_prog().get_attribute_location(ctx, "extent"),
					ref_prog().get_attribute_location(ctx, "position"), &boxes.front(), boxes.size(), boxes[0].get_max_pnt());
				has_positions = true;
				has_extents = true;
				set_position_is_center(false);
			}
			
			template <typename T>
			void set_box_array(const context& ctx, const cgv::media::axis_aligned_box<T, 3>* boxes, size_t count) {
				set_composed_attribute_array(ctx, ref_prog().get_attribute_location(ctx, "position"),
					boxes, count, boxes[0].get_min_pnt());
				ref_composed_attribute_array(ctx, ref_prog().get_attribute_location(ctx, "extent"),
					ref_prog().get_attribute_location(ctx, "position"), boxes, count, boxes[0].get_max_pnt());
				has_positions = true;
				has_extents = true;
				set_position_is_center(false);
			}
			
			template <typename T>
			void set_translation_array(const context& ctx, const std::vector<T>& translations) { has_translations = true; set_attribute_array(ctx, ref_prog().get_attribute_location(ctx, "translation"), translations); }
			
			template <typename T>
			void set_translation_array(const context& ctx, const T* translations, size_t nr_elements, unsigned stride_in_bytes = 0) { has_translations = true; set_attribute_array(ctx, ref_prog().get_attribute_location(ctx, "translation"), translations, nr_elements, stride_in_bytes); }
			
			template <typename T>
			void set_rotation_array(const context& ctx, const std::vector<T>& rotations) { has_rotations = true; set_attribute_array(ctx, ref_prog().get_attribute_location(ctx, "rotation"), rotations); }
			
			template <typename T>
			void set_rotation_array(const context& ctx, const T* rotations, size_t nr_elements, unsigned stride_in_bytes = 0) { has_rotations = true; set_attribute_array(ctx, ref_prog().get_attribute_location(ctx, "rotation"), rotations, nr_elements, stride_in_bytes); }
			
			bool disable(context& ctx);
			
			void draw(context& ctx, size_t start, size_t count,
				bool use_strips = false, bool use_adjacency = false, uint32_t strip_restart_index = -1);
		};
		struct CGV_API voxel_render_style_reflect : public voxel_render_style
		{
			bool self_reflect(cgv::reflect::reflection_handler& rh);
		};
		extern CGV_API cgv::reflect::extern_reflection_traits<voxel_render_style, voxel_render_style_reflect> get_reflection_traits(const voxel_render_style&);
	}
}

#include <cgv/config/lib_end.h>