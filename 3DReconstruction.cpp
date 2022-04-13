#include "3DReconstruction.h"
#include <iostream>
#include <algorithm>
#include "Tables.h"


#define DPETHIMAGE_SSB_BP             0
#define VERTICES_SSB_BP             1//gridpoints
#define TRITABLES_SSB_BP    2
#define TRINORMALS_SSB_BP      3
#define TRIANGLES_SSB_BP    4
#define CUBE_EDGES_SSB_BP  5
#define NORMALS_SSB_BP    6
#define NORMALID_SSB_BP    7
#define TRIANGLEVERTICES_SSB_BP    8

#define TSDFWEIGHT_SSB_BP 9










bool Reconstruction::build_program(cgv::render::context& ctx) {

	if (!tsdf_prog.build_program(ctx, "glsl/tsdf.glpr", true)) {
		std::cerr << "ERROR in building shader program " << std::endl;
		return false;
	}
	if (!triangle_prog.build_program(ctx, "glsl/Triangles.glpr", true)) {
		std::cerr << "ERROR in building shader program " << std::endl;
		return false;
	}	
	if (!marchingcubes_prog.build_program(ctx, "glsl/marchingcubes.glpr", true)) {
		std::cerr << "ERROR in building shader program " << std::endl;
		return false;
	}
	if (!normal_prog.build_program(ctx, "glsl/Normals.glpr", true)) {
		std::cerr << "ERROR in building shader program " << std::endl;
		return false;
	}












	return true;
}
bool Reconstruction::init(rgbd_depth input, vec3 min, vec3 max, ivec3 reso, float voxellength) {

	if (input.Pixels.size()==0)
		return false;

	currentdepthimage = input;
	min_pos= min;
	max_pos= max;
	resolution = reso;
	voxel_length = voxellength;
	deleteAllBuffers();
	createBuffers();
	bindbuffer();



	return true;
}
bool Reconstruction::init(rgbd_depth input, vec3 min, vec3 max) {

	if (input.Pixels.size() == 0)
		return false;

	currentdepthimage = input;
	min_pos = min;
	max_pos = max;	
	return true;
}
bool Reconstruction::init(rgbd_depth input) {

	if (input.Pixels.size() == 0)
		return false;

	currentdepthimage = input;	
	return true;
}



bool Reconstruction::tsdf_algorithm(cgv::render::context& ctx, bool isfirstframe) {

	if (currentdepthimage.Pixels.size() == 0)
		return false;
		
	ivec3 vereticesGridDims = ivec3(resolution[0] + 1, resolution[1] + 1, resolution[2] + 1);
	int length = vereticesGridDims[0] * vereticesGridDims[1] * vereticesGridDims[2];

	tsdf_prog.set_uniform(ctx, "vereticesGridDims", vereticesGridDims);
	tsdf_prog.set_uniform(ctx, "min_pos", min_pos);
	tsdf_prog.set_uniform(ctx, "max_pos", max_pos);
	tsdf_prog.set_uniform(ctx, "voxel_length", voxel_length);
	tsdf_prog.set_uniform(ctx, "fx_d", float(currentdepthimage.para.fx_d));
	tsdf_prog.set_uniform(ctx, "fy_d", float(currentdepthimage.para.fy_d));
	tsdf_prog.set_uniform(ctx, "cx_d", float(currentdepthimage.para.cx_d));
	tsdf_prog.set_uniform(ctx, "cy_d", float(currentdepthimage.para.cy_d));
	tsdf_prog.set_uniform(ctx, "depth_scale", float(currentdepthimage.para.depth_scale));
	tsdf_prog.set_uniform(ctx, "DepthImageWidth", currentdepthimage.width);
	tsdf_prog.set_uniform(ctx, "DepthImageHeight", currentdepthimage.height);
	mat3 back_mat = transpose(currentdepthimage.camera_rotation);
	tsdf_prog.set_uniform(ctx, "backrotationmatrix", back_mat);
	tsdf_prog.set_uniform(ctx, "cam_pos", currentdepthimage.camera_translation);
	tsdf_prog.set_uniform(ctx, "IsFirstFrame", isfirstframe);

	tsdf_prog.enable(ctx);
	glDispatchCompute(vereticesGridDims[0], vereticesGridDims[1], vereticesGridDims[2]);
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	tsdf_prog.disable(ctx);

	//
	
	//apply marchingcubes algorithm
	marchingcubes_prog.set_uniform(ctx, "cubeGridDims", resolution);
	marchingcubes_prog.set_uniform(ctx, "surfaceLevel", surfacelevel);
	marchingcubes_prog.set_uniform(ctx, "vereticesGridDims", vereticesGridDims);
	marchingcubes_prog.set_uniform(ctx, "min_pos", min_pos);
	marchingcubes_prog.set_uniform(ctx, "max_pos", max_pos);
	marchingcubes_prog.set_uniform(ctx, "side_length", voxel_length);
	marchingcubes_prog.enable(ctx);
	glDispatchCompute(resolution[0], resolution[1], resolution[2]);
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	marchingcubes_prog.disable(ctx);

	//get triangles
	triangle_prog.enable(ctx);
	glDispatchCompute(length, 1, 1);
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	triangle_prog.disable(ctx);


	trianglevertices.getSubData(0, sizeof(GLuint), &numVertices);
	triangles.getSubData(0, sizeof(GLuint), &numTriangles);

	//calculate vertex normal
	normal_prog.enable(ctx);
	glDispatchCompute(numVertices, 1, 1);
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	normal_prog.disable(ctx);


	//std::cout << "numTriangles:" << numTriangles << std::endl;

	//float a[15];
	//trianglevertices.getSubData(sizeof(GLuint), sizeof(float) * 15, &a);
	//for (int i = 0; i < 15; i++) {
	//	std::cout << i << ":" << a[i] << std::endl;
	//	//std::cout << "depth" << i << ":" << a[i] << std::endl;
	//}










	return true;

}







bool Reconstruction::MC_algorithm(cgv::render::context& ctx) {




	return false;
}

bool Reconstruction::drawmesh(cgv::render::context& ctx) {

	// Retrieve the number of generated vertices and triangles
	if (numTriangles == 0)
		return false;
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHTING);
	glColor3f(1.f, 1.f, 1.f);
	glBindBuffer(GL_ARRAY_BUFFER, trianglevertices.id);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, (void*)(sizeof(GLuint)));
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindBuffer(GL_ARRAY_BUFFER, normals.id);
	glEnableClientState(GL_NORMAL_ARRAY);
	glNormalPointer(GL_FLOAT, 0, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, triangles.id);
	glDrawElements(GL_TRIANGLES, numTriangles * 3, GL_UNSIGNED_INT, (void*)(sizeof(GLuint)));
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);


	return true;

}




void Reconstruction::createBuffers() {

	int pixels_length = currentdepthimage.height * currentdepthimage.width;
	int vertices_length = (resolution[0] +1) * (resolution[1] + 1) * (resolution[2] + 1);
	int cube_length = resolution[0] * resolution[1] * resolution[2];

	
	vertices = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, vertices_length * sizeof(float));

	depthimage = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, pixels_length * sizeof(int));

	depthimage.setBufferData(currentdepthimage.Pixels.size() * sizeof(int),&currentdepthimage.Pixels.front());

	tsdfweight = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, vertices_length * sizeof(float));
	trianglenormals = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, maxNumTriangles * 3 * sizeof(float));
	triangles = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, maxNumTriangles * 3 * sizeof(int) + sizeof(GLuint));
	cubeedges = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, cube_length * 13 * sizeof(int));
	trianglevertices = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, maxNumVertices * 3 * sizeof(float) + sizeof(GLuint));
	normals = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, maxNumVertices * 3 * sizeof(float));
	normalid = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, maxNumVertices * 17 * sizeof(int));//can be 13

	//tables
	int* flatTriTable = flattenTriTable();
	tables = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_DRAW, (256 + 256 * 16) * sizeof(int));
	tables.setSubData(0, sizeof(edgeTable), edgeTable);
	tables.setSubData(sizeof(edgeTable), 256 * 16 * sizeof(int), flatTriTable);
	delete[] flatTriTable;


}


void Reconstruction::updateBuffers() {

	int pixels_length = currentdepthimage.height * currentdepthimage.width;
	int vertices_length = (resolution[0] + 1) * (resolution[1] + 1) * (resolution[2] + 1);
	int cube_length = resolution[0] * resolution[1] * resolution[2];

	depthimage = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, pixels_length * sizeof(int));
	trianglenormals = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, maxNumTriangles * 3 * sizeof(float));
	triangles = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, maxNumTriangles * 3 * sizeof(int) + sizeof(GLuint));
	cubeedges = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, cube_length * 13 * sizeof(int));
	trianglevertices = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, maxNumVertices * 3 * sizeof(float) + sizeof(GLuint));
	normals = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, maxNumVertices * 3 * sizeof(float));
	normalid = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, maxNumVertices * 17 * sizeof(int));//can be 13


}


void Reconstruction::deleteBuffers()
{
		depthimage.deleteBuffer();
		trianglenormals.deleteBuffer();
		triangles.deleteBuffer();	
		cubeedges.deleteBuffer();
		trianglevertices.deleteBuffer();
		normals.deleteBuffer();
		normalid.deleteBuffer();
		numVertices = 0;
		numTriangles = 0;
	
}
void Reconstruction::deleteAllBuffers(){

	depthimage.deleteBuffer();
	tsdfweight.deleteBuffer();
	vertices.deleteBuffer();
	trianglenormals.deleteBuffer();
	triangles.deleteBuffer();
	tables.deleteBuffer();
	cubeedges.deleteBuffer();
	trianglevertices.deleteBuffer();
	normals.deleteBuffer();
	normalid.deleteBuffer();
	numVertices = 0;
	numTriangles = 0;
}




int* Reconstruction::flattenTriTable()
{
	int* flatTriTable = new int[256 * 16];
	int flatTriIndex = 0;
	for (int i = 0; i < 256; i++) {
		for (int j = 0; j < 16; j++) {
			flatTriTable[flatTriIndex] = triTable[i][j];
			flatTriIndex++;
		}
	}
	return flatTriTable;
}

void Reconstruction::bindbuffer()
{

	depthimage.setBindingPoint(DPETHIMAGE_SSB_BP);
	tsdfweight.setBindingPoint(TSDFWEIGHT_SSB_BP);
	vertices.setBindingPoint(VERTICES_SSB_BP);
	tables.setBindingPoint(TRITABLES_SSB_BP);	
	trianglenormals.setBindingPoint(TRINORMALS_SSB_BP);	
	triangles.setBindingPoint(TRIANGLES_SSB_BP);
	cubeedges.setBindingPoint(CUBE_EDGES_SSB_BP);
	normals.setBindingPoint(NORMALS_SSB_BP);
	normalid.setBindingPoint(NORMALID_SSB_BP);
	trianglevertices.setBindingPoint(TRIANGLEVERTICES_SSB_BP);
			
	GLuint zero = 0;
	trianglevertices.setSubData(0, sizeof(GLuint), &zero);
	triangles.setSubData(0, sizeof(GLuint), &zero);
	
}


