#include "MarchingCubes.h"

#include <iostream>
#include <algorithm>
#include "Tables.h"


#define VERTICES_SSB_BP        0
//#define VERTICES_SSB_BP     1
//#define CUBES_SSB_BP        2
#define TRITABLES_SSB_BP    3
//#define NORMALS_SSB_BP      4
#define TRIANGLES_SSB_BP    5





bool MarchingCubes::get_signed_distance_func() { return false; };

bool MarchingCubes::set_signed_weight(std::vector<int> Voxelid,  std::vector<float> Voxel, uvec3 Vox_resolution) {

	if (Voxelid.size() == 0 || Voxel.size() == 0)
		return false;
	int length_vertives =( Vox_resolution[0]+1)*(Vox_resolution[1] + 1)*(Vox_resolution[2] + 1);
	Vertices.resize(length_vertives,-1);

	
	int x,y,z;
	for (int i = 0; i < Voxelid.size(); i++)
	{
		
		z = int(floor(Voxelid[i] / (Vox_resolution[0] *Vox_resolution[1])));
		y = int(floor((Voxelid[i] - (Vox_resolution[0] *Vox_resolution[1])*z) / Vox_resolution[0]));
		x = int(Voxelid[i] - (Vox_resolution[0] *Vox_resolution[1])*z - Vox_resolution[0] * y);
		
		if (x > 0 && x <= Vox_resolution[0] && y > 0 && y <= Vox_resolution[1] && z > 0 && z <= Vox_resolution[2]) 
			
			if (Voxel[Voxelid[i]] > surfaceLevel  && Voxel[Voxelid[i] - 1] > surfaceLevel
				&& Voxel[Voxelid[i] - Vox_resolution[0]] > surfaceLevel && Voxel[Voxelid[i] - Vox_resolution[0] - 1] > surfaceLevel
				&& Voxel[Voxelid[i] - Vox_resolution[0] * Vox_resolution[1]] > surfaceLevel && Voxel[Voxelid[i] - Vox_resolution[0] * Vox_resolution[1] - 1] > surfaceLevel
				&& Voxel[Voxelid[i] - Vox_resolution[0] * Vox_resolution[1] - Vox_resolution[0]] > surfaceLevel
				&& Voxel[Voxelid[i] - Vox_resolution[0] * Vox_resolution[1] - Vox_resolution[0] - 1] > surfaceLevel
				) {				
			Vertices[x + y * (Vox_resolution[0] + 1) + z * (Vox_resolution[0] + 1)* (Vox_resolution[1] + 1)] = 1;
			}
			
			
				
		
	}
	
	Vertices_size = ivec3(Vox_resolution[0] +1, Vox_resolution[1] +1, Vox_resolution[2] +1);
	Voxel_size = ivec3(Vox_resolution[0] , Vox_resolution[1] , Vox_resolution[2] );
	/*for(int i=0;i< Vertices.size();i++)
		std::cout <<i<< "i" << Vertices [i]<< std::endl;*/
	return false;
}


bool MarchingCubes::init_MC(cgv::render::context& ctx) {

	/*if (!marchingcubes_prog.is_created()) {
		marchingcubes_prog.create(ctx);
		marchingcubes_prog.attach_file(ctx, "glsl/marchingcubes.glcs", cgv::render::ST_COMPUTE);	
		marchingcubes_prog.link(ctx, true);

	}*/
	if (!marchingcubes_prog.build_program(ctx, "glsl/marchingcubes.glpr", true)) {
		std::cerr << "ERROR in building shader program " << std::endl;
		return false;
	}
	return true;
}
void MarchingCubes::resize()
{
	//int x = cubeGrid.x, y = cubeGrid.y, z = cubeGrid.z;
	//// We can deduce the maximum number of vertices the grid can have by counting
	//// the number of edges in the grid, which gives a result just below 3*(w+1)*(h+1)*(d+1)
	//maxNumVertices = y * (x + 1) * (z + 1) + (y + 1) * x * (z + 1) + (y + 1) * (x + 1) * z;
	//maxNumTriangles = cubeGrid.count * 5; // each cube can have 5 triangles at most

	//size.x = cubeGrid.x * cubeSize;
	//size.y = cubeGrid.y * cubeSize;
	//size.z = cubeGrid.z * cubeSize;

	//size.max = std::max(size.x, std::max(size.y, size.z));
}
bool MarchingCubes::resize(int width, int height, int depth, float _cubeSize)
{
	//bool gridChange = width != cubeGrid.x || height != cubeGrid.y || depth != cubeGrid.z;
	//bool cubeChange = cubeSize != _cubeSize;

	////densityGrid = Volume(width + 1, height + 1, depth + 1);
	//cubeGrid = Volume(width, height, depth);
	//cubeSize = _cubeSize;

	//resize();

	///*if (gridChange) {
	//	if (hasPrograms) {
	//		updateDispatchParams();
	//	}
	//	else {
	//		createPrograms();
	//	}
	//	if (hasBuffers) {
	//		deleteBuffers();
	//	}
	//	createBuffers();
	//}*/

	//// returns true if the overall mesh size is different compared to the previous one
	//return gridChange || cubeChange;

	return false;
}
bool MarchingCubes::generate(cgv::render::context& ctx) {
	if (Vertices.size() == 0)
		return false;
	//texture of vertices
	/*Vertices_tex.destruct(ctx);
	cgv::data::data_format Vtex_df(Vertices_size[0], Vertices_size[1], Vertices_size[2], cgv::type::info::TypeId::TI_FLT32, cgv::data::ComponentFormat::CF_R);
	cgv::data::const_data_view Vtex_dv(&Vtex_df, &Vertices.front());
	Vertices_tex.create(ctx, Vtex_dv, 0);
	Vertices_tex.generate_mipmaps(ctx);
	const int V_tex_handle = (const int&)Vertices_tex.handle - 1;
	glBindImageTexture(0, V_tex_handle, 0, GL_TRUE, 0, GL_READ_ONLY, GL_R32F);*/

	//
	maxNumTriangles = Voxel_size[0]*Voxel_size[1]*Voxel_size[2] * 5;
	std::cout << "a1" << std::endl;

	//tables
	//int *flatTriTable = flattenTriTable();
	//glGenBuffers(1, &tables_buffer);
	//glBindBuffer(GL_SHADER_STORAGE_BUFFER, tables_buffer);
	//glBufferData(GL_SHADER_STORAGE_BUFFER, (256 + 256 * 16) * sizeof(int), NULL, GL_DYNAMIC_DRAW);
	////glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	//glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(edgeTable), edgeTable);
	////glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	//glBufferSubData(GL_SHADER_STORAGE_BUFFER, sizeof(edgeTable), 256 * 16 * sizeof(int), flatTriTable);
	//
	//glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	//delete[] flatTriTable;
	//std::cout << "a2" << std::endl;
	//glBindBufferBase(GL_SHADER_STORAGE_BUFFER, TRITABLES_SSB_BP, tables_buffer);

	//vertices
	glGenBuffers(1, &vertices_buffer);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, vertices_buffer);
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(float)*Vertices.size()   , Vertices.data(), GL_DYNAMIC_READ);//+ sizeof(GLuint)COPY
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, VERTICES_SSB_BP, vertices_buffer);

	//triangles
	//glGenBuffers(1, &triangles_buffer);
	//glBindBuffer(GL_SHADER_STORAGE_BUFFER, triangles_buffer);
	//glBufferData(GL_SHADER_STORAGE_BUFFER, maxNumTriangles * sizeof(Triangle) + sizeof(GLuint), NULL, GL_DYNAMIC_COPY);
	////triangles data
	//GLuint zero = 0;
	//glBindBuffer(GL_SHADER_STORAGE_BUFFER, triangles_buffer);
	//glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(int), &zero);
	////glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	//glBindBufferBase(GL_SHADER_STORAGE_BUFFER, TRIANGLES_SSB_BP, triangles_buffer);
	//===========================
	int V_length = Voxel_size[0] * Voxel_size[1] * Voxel_size[2];
	//std::vector<int> results(V.begin(),V.end());
	std::vector<int> results(V_length, 0);
	glNamedBufferData(triangles_buffer, sizeof(int) * V_length, results.data(), GL_DYNAMIC_READ);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, triangles_buffer);
	//===========================
	

	marchingcubes_prog.set_uniform(ctx, "cubeGridDims", Voxel_size);
	/*marchingcubes_prog.set_uniform(ctx, "vereticesGridDims", Vertices_size);
	marchingcubes_prog.set_uniform(ctx, "cubeGridDims", Voxel_size);
	marchingcubes_prog.set_uniform(ctx, "surfaceLevel", surfaceLevel);
	marchingcubes_prog.set_uniform(ctx, "min_pos", min_pos);
	marchingcubes_prog.set_uniform(ctx, "max_pos", max_pos);*/
	
	marchingcubes_prog.enable(ctx);
	uvec3 num_groups = uvec3(Voxel_size);
	glDispatchCompute(num_groups[0], num_groups[1], num_groups[2]);
	
	// do something else
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	marchingcubes_prog.disable(ctx);
	std::cout << "a4" << std::endl;
	// clear 3D image bindings
	

	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);


	/*GLuint* results_ptr = static_cast<GLuint*>(glMapNamedBufferRange(
		triangles_buffer, 0, sizeof(int) * V_length, GL_MAP_READ_BIT));

	std::memcpy(results.data(), results_ptr, results.size() * sizeof(int));
	glUnmapNamedBuffer(triangles_buffer);

	std::vector<float> V_float(results.begin(), results.end());
	std::cout << V_float[0] << std::endl;*/


	//glGetNamedBufferSubData(triangles_buffer, 0, sizeof(GLuint), &numTriangles);
	//glGetNamedBufferSubData(triangles_buffer, sizeof(GLuint), sizeof(Triangle)*numTriangles, &triangles);

	//std::cout << triangles.size() << std::endl;
	return true;
}

bool MarchingCubes::ge(cgv::render::context& ctx)//, std::vector<int> Voxelid
{


	if (Vertices.size()==0)
		return false;
	uvec3 num_groups = uvec3(Voxel_size);
	

	int V_length = num_groups[0] * num_groups[1] * num_groups[2];
	std::vector<int> results(V_length, 0);

	Vertices_tex.destruct(ctx);


	cgv::data::data_format ptex_df(num_groups[0]+1, num_groups[1] + 1, num_groups[2] + 1, cgv::type::info::TypeId::TI_FLT32, cgv::data::ComponentFormat::CF_R);
	cgv::data::const_data_view ptex_dv(&ptex_df, &Vertices.front());
	Vertices_tex.create(ctx, ptex_dv, 0);
	Vertices_tex.generate_mipmaps(ctx);


	const int P_tex_handle = (const int&)Vertices_tex.handle - 1;
	glBindImageTexture(0, P_tex_handle, 0, GL_TRUE, 0, GL_READ_ONLY, GL_R32F);


	marchingcubes_prog.set_uniform(ctx, "cubeGridDims", Voxel_size);
	marchingcubes_prog.set_uniform(ctx, "surfaceLevel", surfaceLevel);
	marchingcubes_prog.set_uniform(ctx, "vereticesGridDims", Vertices_size);
	marchingcubes_prog.set_uniform(ctx, "min_pos", min_pos);
	marchingcubes_prog.set_uniform(ctx, "max_pos", max_pos);

	//glNamedBufferData(vertices_buffer, sizeof(int) * V_length, results.data(), GL_DYNAMIC_READ);
	//glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, vertices_buffer);

	//glGenBuffers(1, &vertices_buffer);
	//glBindBuffer(GL_SHADER_STORAGE_BUFFER, vertices_buffer);//
	//glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(int)* results.size(), results.data(), GL_DYNAMIC_READ); //sizeof(data) only works for statically sized C/C++ arrays.
	//glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, vertices_buffer);


	//triangles
	glGenBuffers(1, &triangles_buffer);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, triangles_buffer);
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(GLuint)+sizeof(float)* results.size()*9 * 5, NULL, GL_DYNAMIC_COPY);
	//glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	GLuint zero = 0;
	glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(GLuint), &zero);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, TRIANGLES_SSB_BP, triangles_buffer);
	
	//tables
	int *flatTriTable = flattenTriTable();
	glGenBuffers(1, &tables_buffer);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, tables_buffer);
	glBufferData(GL_SHADER_STORAGE_BUFFER, (256 + 256 * 16) * sizeof(int), NULL, GL_DYNAMIC_DRAW);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, tables_buffer);
	glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(edgeTable), edgeTable);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, tables_buffer);
	glBufferSubData(GL_SHADER_STORAGE_BUFFER, sizeof(edgeTable), 256 * 16 * sizeof(int), flatTriTable);	
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	delete[] flatTriTable;
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, TRITABLES_SSB_BP, tables_buffer);
	
	



	
	marchingcubes_prog.enable(ctx);

	glDispatchCompute(num_groups[0], num_groups[1], num_groups[2]);

	// do something else
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	marchingcubes_prog.disable(ctx);

	// clear 3D image bindings
	glBindImageTexture(0, 0, 0, GL_TRUE, 0, GL_READ_ONLY, GL_R32F);

	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

	/*GLuint* results_ptr = static_cast<GLuint*>(glMapNamedBufferRange(
		vertices_buffer, 0, sizeof(int) * V_length, GL_MAP_READ_BIT));

	std::memcpy(results.data(), results_ptr, results.size() * sizeof(int));
	glUnmapNamedBuffer(vertices_buffer);*/
	
	glGetNamedBufferSubData(triangles_buffer, 0, sizeof(GLuint), &triCount);
	
	std::vector<vec3> tri(triCount * 3);
	glGetNamedBufferSubData(triangles_buffer, sizeof(GLuint), sizeof(vec3)*triCount*3, static_cast<void*>(tri.data()));

	//std::vector<float> V_float(results.begin(), results.end());

	/*V = V_float;

	V_1.clear();

	for (int i = 0; i < V.size(); i++) {
		if (V[i] != 0)
			V_1.push_back(i);
	}*/
	/*std::cout<<"vf:"<< triCount <<std::endl;
	for (int i = 0; i < triCount * 3;) {
		std::cout << "tri:" << tri[i]<<"  " << tri[i+1] << "  " << tri[i+2] << "  " << std::endl;
		i = i + 3;
	}*/
	triangles = tri;
	return true;


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


	triangles.clear();
	vec3 a1 = vec3(0,0,0);
	vec3 a2 = vec3(1, 0, 0);
	vec3 a3 = vec3(0, 1, 0);
	vec3 n1 = vec3(1,0,0);

	vec3 a4 = vec3(1, 0, 0);
	vec3 a5 = vec3(0, 0, 1);
	vec3 a6 = vec3(0, 1, 0);
	vec3 n2 = vec3(1, 1, 1);

	vec3 a7 = vec3(0, 0, 0);
	vec3 a8 = vec3(0, 1, 0);
	vec3 a9 = vec3(0, 0, 1);
	vec3 n3 = vec3(-1, 0, 0);

	vec3 a10 = vec3(0, 0, 0);
	vec3 a11 = vec3(0, 0, 1);
	vec3 a12 = vec3(1, 0, 0);
	vec3 n4 = vec3(0, -1, 0);

	triangles.push_back(a1);
	triangles.push_back(a2);
	triangles.push_back(a3);
	triangles.push_back(a4);
	triangles.push_back(a5);
	triangles.push_back(a6);
	triangles.push_back(a7);
	triangles.push_back(a8);
	triangles.push_back(a9); 
	triangles.push_back(a10);
	triangles.push_back(a11);
	triangles.push_back(a12);
	std::vector<vec3> N;
	N.push_back(n1);
	N.push_back(n1);
	N.push_back(n1);
	N.push_back(n2);
	N.push_back(n2);
	N.push_back(n2);
	N.push_back(n3);
	N.push_back(n3);
	N.push_back(n3);
	N.push_back(n4);
	N.push_back(n4);
	N.push_back(n4);

	



	if (triangles.size() == 0)
		return false;	
	rgb c(0.5, 0.5, 0.5);
	
	std::vector<rgb> C(triangles.size(),c);
		
	cgv::render::shader_program& prog = ctx.ref_default_shader_program();
	int pi = prog.get_position_index();
	int ci = prog.get_color_index();
	int ni = prog.get_normal_index();
	cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, triangles);
	cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
	cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ci, C);
	cgv::render::attribute_array_binding::enable_global_array(ctx, ci);
	cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ni, N);
	cgv::render::attribute_array_binding::enable_global_array(ctx, ni);
	
	prog.enable(ctx);
	glDrawArrays(GL_TRIANGLES, 0, (GLsizei)triangles.size());
	prog.disable(ctx);
	cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
	cgv::render::attribute_array_binding::disable_global_array(ctx, ci);
	cgv::render::attribute_array_binding::disable_global_array(ctx, ni);
	return true;
}








