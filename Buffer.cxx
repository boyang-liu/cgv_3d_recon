#include "Buffer.h"

#define NULL 0

Buffer::Buffer()
{

}
Buffer::Buffer(GLenum _target, GLenum _usage) : target(_target), usage(_usage)
{
	glGenBuffers(1, &id);
}

Buffer::Buffer(GLenum _target, GLenum _usage, size_t _size) : size(_size), target(_target), usage(_usage)
{
	glGenBuffers(1, &id);
	glBindBuffer(target, id);
	glBufferData(target, size, NULL, usage);
	glBindBuffer(target, 0);
}

Buffer::Buffer(GLenum _target, GLenum _usage, size_t _size, void* data) : size(_size), target(_target), usage(_usage)
{
	glGenBuffers(1, &id);
	glBindBuffer(target, id);
	glBufferData(target, size, data, usage);
	glBindBuffer(target, 0);
}


void Buffer::setData(const void* data) { 
	setSubData(0, size, data); 
}
void Buffer::setSubData(int offset, size_t _size, const void* data) {
	glBindBuffer(target, id);
	glBufferSubData(target, offset, _size, data);
	glBindBuffer(target, 0);
}
void Buffer::getData(void* container) { 
	getSubData(0, size, container); 
}
void Buffer::getSubData(int offset, size_t _size, void* container) {
	glGetNamedBufferSubData(id, offset, _size, container);
}

void Buffer::setBindingPoint(int binding)
{
	glBindBufferBase(target, binding, id);
}
void Buffer::deleteBuffer()
{
	glDeleteBuffers(1, &id);
}