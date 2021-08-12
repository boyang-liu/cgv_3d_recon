#include "history.h"

namespace {
	bool interval_intersects(const int& start_a, const int& end_a,const int& start_b, const int& end_b) {
		int left_bound = std::max(start_a, start_b);
		int right_bound = std::min(end_a, end_b);
		return right_bound - left_bound > 0;
	}
}

operation_span_element::operation_span_element(const unsigned start, const unsigned size) :
	op_start(start), op_size(size) {}

void history::add_rollback_operation()
{
	auto* meta_ptr = (history_meta_data*)glMapNamedBuffer(history_meta_data_buffer, GL_READ_WRITE);
	if (meta_ptr == NULL)
		return;
	last_op_meta = *meta_ptr;
	//auto meta = *meta_ptr;
	
	// check if there is anything to store
	if (last_op_meta.points_written > history_buffer_capacity) {
		//invalidate history if there is too many to store
		operations_ptrs.clear();
		//reset start and size information on device side
		meta_ptr->points_written = 0;
		meta_ptr->history_start = 0;
	}
	else if (last_op_meta.points_written > 0) {
		//reset start and size information on device side
		meta_ptr->points_written = 0;
		meta_ptr->history_start = (last_op_meta.history_start + last_op_meta.points_written) & last_op_meta.history_capacity;
		
		//write operation span to buffer
		operation_span_element op = operation_span_element(last_op_meta.history_start, last_op_meta.points_written);
		//TODO check if anything was overwritten and remove overwritten operations if there were any

		{
			int start_op = op.op_start;
			int end_op = (op.op_start + op.op_size) & history_buffer_capacity;
			int unbounded_end_op = op.op_start + op.op_size;

			while (!operations_ptrs.empty()) {
				auto chk = operations_ptrs.front();
				
				int start_chk = chk.op_start;
				int end_chk = (chk.op_start + chk.op_size) & history_buffer_capacity;
				int unbounded_end_chk = start_chk + chk.op_size;
				//is fully enclosed

				bool op_wraps = unbounded_end_op > history_buffer_capacity;
				bool chk_wraps = unbounded_end_chk > history_buffer_capacity;

				//fully enclosed, none cross cap
				if (!(op_wraps || chk_wraps)) {
					if (interval_intersects(start_op, end_op, start_chk, end_chk)) {
						operations_ptrs.pop_front();
						continue;
					}
					break;
				}
				//op crossed cap
				else if (op_wraps && !chk_wraps) {
					int shift = history_buffer_capacity + 1;
					if (interval_intersects(start_op, unbounded_end_op, start_chk, end_chk) ||
						interval_intersects(start_op, unbounded_end_op, start_chk + shift, end_chk + shift)) {
						operations_ptrs.pop_front();
						continue;
					}
					break;
				}
				//chk crossed cap
				else if (!op_wraps && chk_wraps) {
					int shift = history_buffer_capacity + 1;
					if (interval_intersects(start_chk, unbounded_end_chk, start_op, end_op) ||
						interval_intersects(start_chk, unbounded_end_chk, start_op + shift, end_op + shift)) {
						operations_ptrs.pop_front();
						continue;
					}
					break;
				}
				else if (op_wraps && chk_wraps) {
					if (interval_intersects(start_op, unbounded_end_op, start_chk, unbounded_end_chk)) {
						operations_ptrs.pop_front();
						continue;
					}
					break;
				}

			}
		}
		operations_ptrs.push_back(op);
	}
	glUnmapNamedBuffer(history_meta_data_buffer);
}

void history::init(cgv::render::context& ctx)
{
	GLuint buffers[2];
	glCreateBuffers(2, buffers);
	history_buffer = buffers[0];
	history_meta_data_buffer = buffers[1];
	glNamedBufferStorage(history_buffer, history_buffer_capacity*sizeof(label_operation), nullptr, GL_DYNAMIC_STORAGE_BIT | GL_MAP_READ_BIT);
	glNamedBufferStorage(history_meta_data_buffer, sizeof(history_meta_data), nullptr, GL_DYNAMIC_STORAGE_BIT | GL_MAP_READ_BIT | GL_MAP_WRITE_BIT);
	reset_meta_data(ctx);
}

void history::reset_meta_data(cgv::render::context& ctx, uint32_t start, uint32_t size)
{
	history_meta_data meta;
	meta.history_capacity = history_buffer_capacity;
	meta.history_start = start;
	meta.points_written = size;
	glNamedBufferSubData(history_meta_data_buffer, 0, sizeof(history_meta_data), &meta);
}

const history_meta_data& history::get_last_operation_meta_data() const
{
	return last_op_meta;
}

bool history::get_stored_labels(std::vector<label_operation>& ops)
{
	if (history_meta_data_buffer == 0 || history_buffer == 0) {
		return false;
	}

	auto* meta_ptr = (history_meta_data*)glMapNamedBuffer(history_meta_data_buffer, GL_READ_ONLY);
	auto* history_ptr = (label_operation*)glMapNamedBuffer(history_buffer, GL_READ_ONLY);

	auto err = glGetError();
	if (err != GL_NO_ERROR) {
		return false;
	}

	auto meta = *meta_ptr;
	glUnmapNamedBuffer(history_meta_data_buffer);

	for (int i = 0; i < meta.points_written; ++i) {
		unsigned index = (i+meta.history_start) & meta.history_capacity;
		ops.push_back(history_ptr[index]);
	}
	glUnmapNamedBuffer(history_buffer);
	return true;
}

bool history::rollback_last_operation(cgv::render::context& ctx)
{
	if (operations_ptrs.empty()) {
		return false;
	}
	
	operation_span_element op = operations_ptrs.back();
	operations_ptrs.pop_back();
	//set meta data
	reset_meta_data(ctx, op.op_start, op.op_size);

	bind(ctx);

	//run rollback
	if (label_rollback_prog && label_rollback_prog->enable(ctx)) {
		// run computation
		glDispatchCompute(16, 1, 1);

		// synchronize
		glMemoryBarrier(GL_ALL_BARRIER_BITS);
		label_rollback_prog->disable(ctx);

		if (!operations_ptrs.empty()) {
			reset_meta_data(ctx, op.op_start, 0);
		}
		else {
			reset_meta_data(ctx, 0, 0);
		}
	}
	else {
		std::cerr << "history::rollback_last_operation: can't enable shader program\n";
		if (!label_rollback_prog)
			std::cerr << "history::rollback_last_operation: use set_rollback_shader_prog to set a shader program\n";
		return false;
	}
	return true;
}

void history::clear(cgv::render::context& ctx)
{
	glDeleteBuffers(1, &history_buffer);
	glDeleteBuffers(1, &history_meta_data_buffer);
	history_buffer = history_meta_data_buffer = 0;
}

void history::remove_all()
{
	operations_ptrs.clear();
}
