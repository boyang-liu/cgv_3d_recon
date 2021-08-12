#pragma once

#include <cgv/render/render_types.h>

// stores parameters for actions regarding opengl buffers triggered from places where no opengl context is avaiable or safe to access
struct buffer_action : public cgv::render::render_types {
	enum action_type{
		POINT_LABELING_TOOL = 1,
		POINT_RELABELING = 2,
		ROLLBACK = 3
	};
	
	template<typename T>
	T* get_as() {
		return (T*)this;
	}
	action_type get_type() const {
		return type_p;
	};
protected:
	void set_type(const action_type t) {
		type_p = t;
	}
private:
	action_type type_p;
};

// due to the render context being only avaiable in certain methods, actions on the opengl buffers can not always execute immediately
// on button presses for example, actions have to be stored in a queue and then execute in the init_frame method
// this struct stores the required information for this
// shader: point_labeler_tool.glcs
struct point_labeling_tool_commit : public buffer_action {
	vec3 position;
	float radius = 0.f;
	int32_t label = -1;
	int32_t group_mask = 0;
	vec3 max_ext = vec3(0.f);
	vec3 min_ext = vec3(0.f);
	mat3 plane_ori = mat3(0.f);

	inline point_labeling_tool_commit() {
		set_type(POINT_LABELING_TOOL);
	};
};

struct point_relabel_commit : public buffer_action {
	int32_t new_label = 0;
	int32_t expected_label = 0;
	int32_t point_group_mask = 0;

	inline point_relabel_commit() {
		set_type(POINT_RELABELING);
	};
};

struct rollback_commit : public buffer_action {
	inline rollback_commit() {
		set_type(ROLLBACK);
	};
};