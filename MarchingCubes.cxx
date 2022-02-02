#include "MarchingCubes.h"

#include <iostream>
#include <algorithm>
#include "Tables.h"


#define GRIDPOINTS_SSB_BP        0
#define CUBES_SSB_BP          1
#define CUBE_EDGES_SSB_BP        2
#define TRITABLES_SSB_BP    3
#define TRINORMALS_SSB_BP      4
#define TRIANGLES_SSB_BP    5
#define VERTICES_SSB_BP    6
#define NORMALS_SSB_BP    7
#define NORMALID_SSB_BP    8

#define OBJBOUNDARY_SSB_BP        9
#define OBJINSIDE_SSB_BP         10
#define DENOISEDOBJ_SSB_BP        11
#define CUBES_SSB_BP        12
//#define BOXARRAY_SSB_BP        13

struct Vector
{
	float x, y, z;
};


bool MarchingCubes::set_signed_weight(std::vector<int> Voxelid,  std::vector<float> Voxel, uvec3 Vox_resolution) {


	return false;
}


bool MarchingCubes::initialize(cgv::render::context& ctx) {

	
	if (!normal_prog.build_program(ctx, "glsl/Normals.glpr", true)) {
		std::cerr << "ERROR in building shader program " << std::endl;
		return false;
	}
	if (!triangle_prog.build_program(ctx, "glsl/Triangles.glpr", true)) {
		std::cerr << "ERROR in building shader program " << std::endl;
		return false;
	}
	if (!GridPoints_prog.build_program(ctx, "glsl/GridPoints.glpr", true)) {
		std::cerr << "ERROR in building shader program " << std::endl;
		return false;
	}
	if (!marchingcubes_prog.build_program(ctx, "glsl/marchingcubes.glpr", true)) {
		std::cerr << "ERROR in building shader program " << std::endl;
		return false;
	}
	return true;
}
void MarchingCubes::resize()
{

	maxNumTriangles = Cubes.size() * 5; // each cube can have 5 triangles at most
	maxNumVertices = Voxel_size[1] * (Voxel_size[0] + 1)*(Voxel_size[2] + 1) + (Voxel_size[1] + 1)*Voxel_size[0] *(Voxel_size[2] + 1)
		+ (Voxel_size[1] + 1)*(Voxel_size[0] + 1)*Voxel_size[2];


}


bool MarchingCubes::resize(ivec3 resolution ,vec3 min,vec3 max)
{	
	min_pos = min;
	max_pos = max;
	if (resolution == Voxel_size)
		return true;
	


	Voxel_size = resolution;
	GridPoints_size = ivec3(Voxel_size[0] + 1, Voxel_size[1] + 1, Voxel_size[2] + 1);
	maxNumTriangles = Cubes.size() * 5; // each cube can have 5 triangles at most
	maxNumVertices = Voxel_size[1] * (Voxel_size[0] + 1)*(Voxel_size[2] + 1) + (Voxel_size[1] + 1)*Voxel_size[0] * (Voxel_size[2] + 1)
		+ (Voxel_size[1] + 1)*(Voxel_size[0] + 1)*Voxel_size[2];

	
	if(hasBuffers)
		deleteBuffers();
	createBuffers();
	bindbuffer();
	

	return true;
}



bool MarchingCubes::initMC(std::vector<float> result_voxelization, ivec3 resolution, vec3 min, vec3 max) {


	//input
	Cubes = result_voxelization;
	//
	resize(resolution, min, max);
	cubes.setData(Cubes.data());
	hasBuffers = true;
	return true;
}
bool MarchingCubes::initMC(std::vector<float> result_voxelization) {

	if (!hasBuffers)
		return false;
	//input
	Cubes = result_voxelization;	
	cubes.setData(Cubes.data());

	return true;
}



bool MarchingCubes::generate(cgv::render::context& ctx)//, std::vector<int> Voxelid
{



	int length = Voxel_size[0] * Voxel_size[1] * Voxel_size[2];
	uvec3 num_groups = uvec3(Voxel_size);
	

	//calculate the gridpoints weight
	//if it is inside the object,its value become 1
	GridPoints_prog.set_uniform(ctx, "cubeGridDims", Voxel_size);
	GridPoints_prog.set_uniform(ctx, "vereticesGridDims", GridPoints_size);
	GridPoints_prog.set_uniform(ctx, "surfaceLevel", surfaceLevel);
	GridPoints_prog.enable(ctx);
	glDispatchCompute(GridPoints_size[0], GridPoints_size[1], GridPoints_size[2]);
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	GridPoints_prog.disable(ctx);


	//apply marchingcubes algorithm
	marchingcubes_prog.set_uniform(ctx, "cubeGridDims", Voxel_size);
	marchingcubes_prog.set_uniform(ctx, "surfaceLevel", surfaceLevel);
	marchingcubes_prog.set_uniform(ctx, "vereticesGridDims", GridPoints_size);
	marchingcubes_prog.set_uniform(ctx, "min_pos", min_pos);
	marchingcubes_prog.set_uniform(ctx, "max_pos", max_pos);
	marchingcubes_prog.enable(ctx);
	glDispatchCompute(Voxel_size[0], Voxel_size[1], Voxel_size[2]);
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	marchingcubes_prog.disable(ctx);

	
	//get triangles

	triangle_prog.enable(ctx);
	glDispatchCompute(length, 1, 1);
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	triangle_prog.disable(ctx);


	vertices.getSubData(0, sizeof(GLuint), &numVertices);
	triangles.getSubData(0, sizeof(GLuint), &numTriangles);

	//calculate vertex normal
	normal_prog.enable(ctx);
	glDispatchCompute(numVertices, 1, 1);
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	normal_prog.disable(ctx);



	
	
	return false;
	
	

	

}
void MarchingCubes::bindbuffer()
{
	cubes.setBindingPoint(CUBES_SSB_BP);
	tables.setBindingPoint(TRITABLES_SSB_BP);
	gridpoints.setBindingPoint(GRIDPOINTS_SSB_BP);
	vertices.setBindingPoint(VERTICES_SSB_BP);
	triangles.setBindingPoint(TRIANGLES_SSB_BP);
	cubeedges.setBindingPoint(CUBE_EDGES_SSB_BP);
	trianglenormals.setBindingPoint(TRINORMALS_SSB_BP);
	normals.setBindingPoint(NORMALS_SSB_BP);
	normalid.setBindingPoint(NORMALID_SSB_BP);
	GLuint zero = 0;
	vertices.setSubData(0, sizeof(GLuint), &zero);
	triangles.setSubData(0, sizeof(GLuint), &zero);


}
int* MarchingCubes::flattenTriTable()
{
	int *flatTriTable = new int[256 * 16];
	int flatTriIndex = 0;
	for (int i = 0; i < 256; i++) {
		for (int j = 0; j < 16; j++) {
			flatTriTable[flatTriIndex] = triTable[i][j];
			flatTriIndex++;
		}
	}
	return flatTriTable;
}



bool MarchingCubes::draw(cgv::render::context& ctx) {

	if (!hasBuffers)
		return false;

	
	

	// Retrieve the number of generated vertices and triangles

	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHTING);
	glColor3f(1.f, 1.f, 1.f);
	glBindBuffer(GL_ARRAY_BUFFER, vertices.id);
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

void MarchingCubes::deleteBuffers()
{
	
	gridpoints.deleteBuffer();
	trianglenormals.deleteBuffer();
	cubes.deleteBuffer();
	triangles.deleteBuffer();
	tables.deleteBuffer();
	cubeedges.deleteBuffer();
	vertices.deleteBuffer();
	normals.deleteBuffer();
	normalid.deleteBuffer();
	numVertices = 0;
	numTriangles = 0;
	//vertices.deleteBuffer();	
}


void MarchingCubes::createBuffers() {

	int length = Voxel_size[0] * Voxel_size[1] * Voxel_size[2];	
	int length_P = GridPoints_size[0] * GridPoints_size[1]  * GridPoints_size[2] ;
	trianglenormals = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, maxNumTriangles * 3 * sizeof(float));
	cubes = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_DRAW, length * sizeof(float));
	
	triangles = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, maxNumTriangles * 3 * sizeof(int) + sizeof(GLuint));
	gridpoints = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, length_P * sizeof(float) );
	cubeedges = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, length * 13 * sizeof(int));
	vertices = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, maxNumVertices * 3 * sizeof(float) + sizeof(GLuint));
	normals = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, maxNumVertices * 3 * sizeof(float));
	normalid = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, maxNumVertices * 17 * sizeof(int));//can be 13


	//tables
	int *flatTriTable = flattenTriTable();
	tables = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_DRAW, (256 + 256 * 16) * sizeof(int));
	tables.setSubData(0, sizeof(edgeTable), edgeTable);
	tables.setSubData(sizeof(edgeTable), 256 * 16 * sizeof(int), flatTriTable);
	delete[] flatTriTable;


}





