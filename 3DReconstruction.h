#pragma once
#include <cgv/render/render_types.h>
#include <cgv/render/shader_program.h>
#include <3rd/glew/GL/glew.h>

#include <cgv/render/render_types.h>
#include "rgbd_depth.h"
#include <cgv_gl/sphere_renderer.h>
#include "Buffer.h"

class Reconstruction : public cgv::render::render_types {

public:
	Reconstruction() {
		resolution = 100;
	};
	bool build_program(cgv::render::context& ctx);
	bool init(rgbd_depth input, vec3 min, vec3 max, ivec3 reso, float voxellength);
	bool init(rgbd_depth input, vec3 min, vec3 max);
	bool init(rgbd_depth input);
	bool tsdf_algorithm(cgv::render::context& ctx,bool isfirstframe);
	bool MC_algorithm(cgv::render::context& ctx);
	bool drawmesh(cgv::render::context& ctx);

	void createBuffers();
	ivec3 resolution;
	rgbd_depth currentdepthimage;
	vec3 min_pos;
	vec3 max_pos;
	float voxel_length;
	float surfacelevel;
	int maxNumVertices;
	int maxNumTriangles;
	GLuint numVertices = 0;
	GLuint numTriangles = 0;
	void updateBuffers();
	void deleteBuffers();
	void deleteAllBuffers();
	int* flattenTriTable();
	void bindbuffer();

protected:
	cgv::render::shader_program tsdf_prog;
	cgv::render::shader_program triangle_prog;
	cgv::render::shader_program marchingcubes_prog;
	cgv::render::shader_program normal_prog;


private:

	Buffer depthimage;
	Buffer tsdfweight;
	Buffer vertices;//gridpoints
	Buffer tables;
	Buffer trianglenormals;
	Buffer triangles;


	Buffer cubeedges;
	Buffer trianglevertices;
	Buffer normals;
	Buffer normalid;



};