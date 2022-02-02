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


struct Vector
{
	float x, y, z;
};


bool MarchingCubes::set_signed_weight(std::vector<int> Voxelid,  std::vector<float> Voxel, uvec3 Vox_resolution) {


	return false;
}


bool MarchingCubes::initialize(cgv::render::context& ctx) {

	if (!denoise_prog.build_program(ctx, "glsl/denoise.glpr", true)) {
		std::cerr << "ERROR in building shader program " << std::endl;
		return false;
	}
	if (!fill_prog.build_program(ctx, "glsl/filling.glpr", true)) {
		std::cerr << "ERROR in building shader program " << std::endl;
		return false;
	}	
	if (!remove_outlier_prog.build_program(ctx, "glsl/remove_outlier.glpr", true)) {
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
	if (!normal_prog.build_program(ctx, "glsl/Normals.glpr", true)) {
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
//
//void MarchingCubes::draw_voxels(cgv::render::context& ctx) {
//
//	if (render_content.size() == 0)
//		return;
//
//	cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
//
//
//	boxes.clear();
//	box_colors.clear();
//	rgba8 clr(255, 255, 255, 255);
//
//	center_gravity = vec3(0, 0, 0);
//	vec3 sum = vec3(0, 0, 0);
//	float sigma_m = 0;
//	for (int i = 1; i <= Voxel_size[0]; i++)
//		for (int j = 1; j <= Voxel_size[1]; j++)
//			for (int k = 1; k <= Voxel_size[2]; k++)
//			{
//				int index = (k - 1) * Voxel_size[1] * Voxel_size[0] + (j - 1) * Voxel_size[0] + i - 1;
//				if (render_content[index] != 0) {
//					vec3 BoxMinPos = min_pos + vec3((i - 1) * side_length, (j - 1) * side_length, (k - 1) * side_length);
//					vec3 BoxMaxPos = min_pos + vec3(i * side_length, j * side_length, k * side_length);
//					boxes.emplace_back(box3(BoxMinPos, BoxMaxPos));
//					box_colors.emplace_back(clr);
//
//					sigma_m += 1;
//					sum = sum + (BoxMinPos + BoxMaxPos) / 2;
//
//				}
//
//			}
//	center_gravity = sum / sigma_m;
//
//	cgv::render::box_render_style style;
//	renderer.set_render_style(style);
//
//
//	renderer.set_box_array(ctx, boxes);
//	renderer.set_color_array(ctx, box_colors);
//	if (renderer.validate_and_enable(ctx)) {
//		//glEnable(GL_BLEND);
//		//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//		//glDepthMask(GL_FALSE);
//		//glDisable(GL_LIGHTING);
//
//		glDrawArrays(GL_POINTS, 0, (GLsizei)boxes.size());
//		//glColor4f(255, 255, 0, 120);
//		////glDrawArrays(GL_POINTS, 0, (GLsizei)numBoxes);
//		//glDisable(GL_BLEND);
//		//glEnable(GL_LIGHTING);
//		//glDepthMask(GL_TRUE);
//	}
//	renderer.disable(ctx);
//	//Object_Boundary.clear();
//
//}

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

/*





bool Voxelization::init_voxelization(cgv::render::context& ctx) {
	if (!fill_prog.build_program(ctx, "glsl/filling.glpr", true)) {
		std::cerr << "ERROR in building shader program " << std::endl;
		return false;
	}

	if (!denoise_prog.build_program(ctx, "glsl/denoise.glpr", true)) {
		std::cerr << "ERROR in building shader program " << std::endl;
		return false;
	}
	if (!remove_outlier_prog.build_program(ctx, "glsl/remove_outlier.glpr", true)) {
		std::cerr << "ERROR in building shader program " << std::endl;
		return false;
	}
	return true;

}

bool Voxelization::init_boundary_from_PC(std::vector<rgbd_pointcloud> pc, vec3 min, vec3 max, float side) {

	return true;
}

bool Voxelization::denoising(cgv::render::context& ctx, int filter_threshold, int kernel_range) {

	return true;

}

bool Voxelization::traverse_voxels(cgv::render::context& ctx, std::vector<vec3> cam_pos) {

	return true;
}
//==========================================================================================================================
void Voxelization::draw_voxels(cgv::render::context& ctx) {

	if (render_content.size() == 0)
		return;

	cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);


	boxes.clear();
	box_colors.clear();
	rgba8 clr(255, 255, 255, 255);

	center_gravity = vec3(0, 0, 0);
	vec3 sum = vec3(0, 0, 0);
	float sigma_m = 0;
	for (int i = 1; i <= Voxel_size[0]; i++)
		for (int j = 1; j <= Voxel_size[1]; j++)
			for (int k = 1; k <= Voxel_size[2]; k++)
			{
				int index = (k - 1) * Voxel_size[1] * Voxel_size[0] + (j - 1) * Voxel_size[0] + i - 1;
				if (render_content[index] != 0) {
					vec3 BoxMinPos = min_pos + vec3((i - 1) * side_length, (j - 1) * side_length, (k - 1) * side_length);
					vec3 BoxMaxPos = min_pos + vec3(i * side_length, j * side_length, k * side_length);
					boxes.emplace_back(box3(BoxMinPos, BoxMaxPos));
					box_colors.emplace_back(clr);

					sigma_m += 1;
					sum = sum + (BoxMinPos + BoxMaxPos) / 2;

				}

			}
	center_gravity = sum / sigma_m;

	cgv::render::box_render_style style;
	renderer.set_render_style(style);


	renderer.set_box_array(ctx, boxes);
	renderer.set_color_array(ctx, box_colors);
	if (renderer.validate_and_enable(ctx)) {
		//glEnable(GL_BLEND);
		//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		//glDepthMask(GL_FALSE);
		//glDisable(GL_LIGHTING);
		
		glDrawArrays(GL_POINTS, 0, (GLsizei)boxes.size());
		//glColor4f(255, 255, 0, 120);
		////glDrawArrays(GL_POINTS, 0, (GLsizei)numBoxes);
		//glDisable(GL_BLEND);
		//glEnable(GL_LIGHTING);
		//glDepthMask(GL_TRUE);
	}
	renderer.disable(ctx);
	//Object_Boundary.clear();

}


bool Voxelization::init(std::vector<rgbd_pointcloud> pc, vec3 min, vec3 max, float side) {



	min_pos = min;
	max_pos = max;
	side_length = side;
	Voxel_size = ceil((max - min) / (float)side_length);
	int length = Voxel_size[0] * Voxel_size[1] * Voxel_size[2];
	Object_Boundary.clear();
	Object_Boundary.resize(length, 0);
	for (int i = 0; i < pc.size(); i++)
		for (int j = 0; j < pc[i].get_nr_Points(); j++)
		{
			ivec3 v1 = ceil((pc[i].pnt(j) - min) / (float)side_length);
			int temp = (v1[2] - 1) * Voxel_size[1] * Voxel_size[0] + (v1[1] - 1) * Voxel_size[0] + v1[0] - 1;
			Object_Boundary[temp] = 1;
		}
	deleteBuffers();

	return true;
}

bool Voxelization::generate(cgv::render::context& ctx, std::vector<vec3> cam_pos)
{


	//init_voxelization(ctx);

	createBuffers();
	bindbuffer();
	//calculate the gridpoints weight
	//if it is inside the object,its value become 1

	int length = Voxel_size[0] * Voxel_size[1] * Voxel_size[2];
	int filter_threshold = 5;
	int kernel_range = 3;
	denoise_prog.set_uniform(ctx, "cubeGridDims", Voxel_size);
	denoise_prog.set_uniform(ctx, "threshold", filter_threshold);
	denoise_prog.set_uniform(ctx, "range", kernel_range);
	denoise_prog.enable(ctx);
	glDispatchCompute(Voxel_size[0], Voxel_size[1], Voxel_size[2]);
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	denoise_prog.disable(ctx);

	fill_prog.set_uniform(ctx, "min_pos", min_pos);
	fill_prog.set_uniform(ctx, "max_pos", max_pos);
	fill_prog.set_uniform(ctx, "side_length", side_length);
	fill_prog.set_uniform(ctx, "cubeGridDims", Voxel_size);
	fill_prog.set_uniform(ctx, "cam_pos1", cam_pos[0]);
	fill_prog.set_uniform(ctx, "cam_pos2", cam_pos[1]);
	fill_prog.set_uniform(ctx, "cam_pos3", cam_pos[2]);
	fill_prog.enable(ctx);
	glDispatchCompute(Voxel_size[0], Voxel_size[1], Voxel_size[2]);
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	fill_prog.disable(ctx);

	//GLuint zero = 0;
	//boxarray.setSubData(0, sizeof(GLuint), &zero);
	filter_threshold = 13;
	kernel_range = 5;
	remove_outlier_prog.set_uniform(ctx, "cubeGridDims", Voxel_size);
	remove_outlier_prog.set_uniform(ctx, "threshold", filter_threshold);
	remove_outlier_prog.set_uniform(ctx, "range", kernel_range);
	remove_outlier_prog.set_uniform(ctx, "side_length", side_length);
	remove_outlier_prog.set_uniform(ctx, "min_pos", min_pos);
	remove_outlier_prog.set_uniform(ctx, "max_pos", max_pos);

	remove_outlier_prog.enable(ctx);
	glDispatchCompute(Voxel_size[0], Voxel_size[1], Voxel_size[2]);
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	remove_outlier_prog.disable(ctx);

	Object_Boundary.clear();
	Object_Boundary.resize(length);
	cubes.getSubData(0, sizeof(float) * length, static_cast<void*>(Object_Boundary.data()));
	render_content = Object_Boundary;

	return true;
}

void Voxelization::bindbuffer()
{

	object_boundary.setBindingPoint(OBJBOUNDARY_SSB_BP);
	filled_object.setBindingPoint(OBJINSIDE_SSB_BP);
	denoised_object.setBindingPoint(DENOISEDOBJ_SSB_BP);
	cubes.setBindingPoint(CUBES_SSB_BP);
	//boxarray.setBindingPoint(BOXARRAY_SSB_BP);

}

void Voxelization::createBuffers()
{

	int length = Voxel_size[0] * Voxel_size[1] * Voxel_size[2];

	object_boundary = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, length * sizeof(float));
	object_boundary.setSubData(0, sizeof(float) * Object_Boundary.size(), Object_Boundary.data());
	filled_object = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, length * sizeof(float));
	denoised_object = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, length * sizeof(float));
	cubes = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, length * sizeof(float));
	//boxarray = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, length *6* sizeof(float)+ sizeof(GLuint));
	//center_mass = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, length * 6 * sizeof(float));
}
void Voxelization::deleteBuffers()
{

	object_boundary.deleteBuffer();
	filled_object.deleteBuffer();
	denoised_object.deleteBuffer();
	cubes.deleteBuffer();
	//boxarray.deleteBuffer();


}
*/