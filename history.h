#pragma once

#include <cgv/render/render_types.h>
#include <cgv_gl/renderer.h>
#include <deque>


struct history_meta_data
{
	GLuint history_start;
	GLuint history_capacity; // allways (power of 2) -1, allows to use & as replacement for mod
	GLuint points_written; // points_written > capacity means the ringbuffer overflowed and the history got corrupted
};

struct label_operation {
	GLint label;
	GLuint index;
};

struct operation_span_element : public cgv::render::render_types {
	unsigned op_start;
	unsigned op_size;

	operation_span_element(const unsigned start=0, const unsigned size=0);
};

class history : public cgv::render::render_types{
	GLuint history_buffer = 0;
	GLuint history_meta_data_buffer = 0;
	unsigned history_buffer_capacity = 0;
	std::deque<operation_span_element> operations_ptrs;

	GLuint history_buffer_pos = 7;
	GLuint history_meta_data_pos = 8;

	cgv::render::shader_program* label_rollback_prog = nullptr;
	//store meta data read by add_rollback_operation
	history_meta_data last_op_meta;
public:
	// stores data commited by shaders to the history as one operation
	void add_rollback_operation();

	history() = default;
	inline history(unsigned logarithmic_size) {
		history_buffer_capacity = (1 << logarithmic_size) - 1;
	}

	// create buffers and shader program
	void init(cgv::render::context& ctx);

	//bind buffers history buffers
	inline void bind(cgv::render::context& ctx) {
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, history_buffer_pos, history_buffer);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, history_meta_data_pos, history_meta_data_buffer);
	}
	
	//allow setting a shader prog used for apply rollbacks
	inline void set_rollback_shader_prog(cgv::render::shader_program* prog) {
		label_rollback_prog = prog;
	}

	//deletes meta data buffers
	void reset_meta_data(cgv::render::context& ctx, uint32_t start = 0, uint32_t size = 0);

	const history_meta_data& get_last_operation_meta_data() const;

	// write labels that are not yet compiled into an operation to ops, return value is true if successfull
	bool get_stored_labels(std::vector<label_operation>& ops);

	//rollback last labeling operation and assign points to group given by point_group
	bool rollback_last_operation(cgv::render::context& ctx);

	void clear(cgv::render::context& ctx);

	//remove all stored operations, buffers stay unchanged
	void remove_all();
};