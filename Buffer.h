#pragma once
#ifndef BUFFER_H
#define BUFFER_H
//#define GLEW_STATIC

#include <3rd/glew/GL/glew.h>
//#include <GLFW/glfw3.h>
class Buffer {


public:
	GLuint id;
	size_t size;
	Buffer();

	Buffer(GLenum _target, GLenum _usage);
	Buffer(GLenum _target, GLenum _usage, size_t _size);
	Buffer(GLenum _target, GLenum _usage, size_t _size, void* data);
	void setData(const void* data);
	void setSubData(int offset, size_t _size, const void* data);
	void getData(void* container);
	void getSubData(int offset, size_t _size, void* container);
	void setBindingPoint(int binding);
	void deleteBuffer();
protected:

private:
	GLenum target;
	GLenum usage;

};

#endif // BUFFER_H