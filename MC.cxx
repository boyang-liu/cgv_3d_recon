#include "MC.h"

#include <iostream>
#include <algorithm>
#include "Tables.h"


//#define NOISE_SSB_BP        0
//#define VERTICES_SSB_BP     1
#define CUBES_SSB_BP        2
#define TRITABLES_SSB_BP    3
//#define NORMALS_SSB_BP      4
//#define TRIANGLES_SSB_BP    5





bool MarchingCubes::get_signed_distance_func() {};

bool MarchingCubes::set_signed_weight(std::vector<int> Voxelid,  std::vector<float> Voxel, uvec3 Vox_size) {

	if (Voxelid.size() == 0 || Voxel.size() == 0)
		return false;
	int length_vertives =( Vox_size.x+1)*(Vox_size.y + 1)*(Vox_size.z + 1);
	Vertices.resize(length_vertives,0);

	
	int x,y,z;
	for (int i = 0; i < Voxelid.size(); i++)
	{
		
		z = int(floor(Voxelid[i] / (Vox_size.x*Vox_size.y)));
		y = int(floor((Voxelid[i] - (Vox_size.x*Vox_size.y)*z) / Vox_size.x));
		x = int(Voxelid[i] - (Vox_size.x*Vox_size.y)*z - Vox_size.x*y);
		if ( x > 0 && x < Vox_size.x && y > 0 && y < Vox_size.y && z > 0 && z < Vox_size.z )
			if ( Voxel[Voxelid[i] + 1] != 0 && Voxel[Voxelid[i] - 1] != 0
				&& Voxel[Voxelid[i] + Vox_size.x] != 0 && Voxel[Voxelid[i] - Vox_size.x] != 0 
				 && Voxel[Voxelid[i] + Vox_size.x*Vox_size.y] != 0 && Voxel[Voxelid[i] - Vox_size.x*Vox_size.y] != 0)
				Vertices[x+ y*( Vox_size.x+1) + z * (Vox_size.x + 1)* (Vox_size.y + 1)] = 1;

	}
	
	Vertices_size = ivec3(Vox_size.x+1, Vox_size.y+1, Vox_size.z+1);



	return false;
}


bool MarchingCubes::init_MC(cgv::render::context& ctx) {

	if (!marchingcubes_prog.build_program(ctx, "glsl/marchingcubes.glpr", true)) {
		std::cerr << "ERROR in building shader program " << std::endl;
		return false;
	}
	
	return true;
}


bool MarchingCubes::generate(cgv::render::context& ctx) {
	if (Vertices.size()==0)
		return false;
	//texture of vertices
	Vertices_tex.destruct(ctx);
	cgv::data::data_format Vtex_df(Vertices_size[0], Vertices_size[1], Vertices_size[2], cgv::type::info::TypeId::TI_FLT32, cgv::data::ComponentFormat::CF_R);
	cgv::data::const_data_view Vtex_dv(&Vtex_df, &Vertices.front());
	Vertices_tex.create(ctx, Vtex_dv, 0);
	Vertices_tex.generate_mipmaps(ctx);
	const int V_tex_handle = (const int&)Vertices_tex.handle - 1;
	glBindImageTexture(0, V_tex_handle, 0, GL_TRUE, 0, GL_READ_ONLY, GL_R32F);

	//tables
	int *flatTriTable = flattenTriTable();
	glGenBuffers(1, &tables_buffer);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, tables_buffer);
	glBufferData(GL_SHADER_STORAGE_BUFFER, (256 + 256 * 16) * sizeof(int), NULL, GL_DYNAMIC_DRAW);
	//glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(edgeTable), edgeTable);
	//glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	glBufferSubData(GL_SHADER_STORAGE_BUFFER, sizeof(edgeTable), 256 * 16 * sizeof(int), flatTriTable);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	delete[] flatTriTable;

	//cubes
	glGenBuffers(1, &cubes_buffer);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, cubes_buffer);
	glBufferData(GL_SHADER_STORAGE_BUFFER, cubeGrid.count * 13 * sizeof(int), NULL, GL_DYNAMIC_COPY);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);



	marchingcubes_prog.enable(ctx);
	/*marchingcubes_prog.set_uniform(ctx, "min_bbox", min_pos);
	marchingcubes_prog.set_uniform(ctx, "max_bbox", max_pos);
	marchingcubes_prog.set_uniform(ctx, "voxel_size", voxel_size);
	marchingcubes_prog.set_uniform(ctx, "resolution", num_groups);
	marchingcubes_prog.set_uniform(ctx, "cam_pos1", cam_pos[0]);
	marchingcubes_prog.set_uniform(ctx, "cam_pos2", cam_pos[1]);
	marchingcubes_prog.set_uniform(ctx, "cam_pos3", cam_pos[2]);*/
	glDispatchCompute(Vertices_size[0], Vertices_size[1], Vertices_size[2]);
	//int length = num_groups[0] * num_groups[1] * num_groups[2];
	// do something else
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	marchingcubes_prog.disable(ctx);

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



bool MarchingCubes::draw() {
	return false;
}










/*
if (Voxelid.size() == 0 || Voxel.size() == 0)
		return false;
	int length_vertives =( Vox_size.x+1)*(Vox_size.y)*(Vox_size.z);
	int length_voxel = Voxel.size();
	std::vector<float> vert;
	vert.resize(length_vertives,0);
	Vertices_size = ivec3(Vox_size.x+1, Vox_size.y + 1, Vox_size.z + 1);
	//to do



	std::vector<float> results(length_vertives, 0);

	glNamedBufferData(Vert_results_buffer, sizeof(int) * length_vertives, results.data(), GL_DYNAMIC_READ);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, Vert_results_buffer);

	glGenBuffers(1, &Voxel_buffer);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, Voxel_buffer);//
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(int)* length_voxel, Voxel.data(), GL_DYNAMIC_READ); //sizeof(data) only works for statically sized C/C++ arrays.

	glGenBuffers(1, &Voxelid_buffer);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, Voxelid_buffer);//
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(int)* Voxelid.size(), Voxelid.data(), GL_DYNAMIC_READ); //sizeof(data) only works for statically sized C/C++ arrays.


	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, Voxel_buffer);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, Voxelid_buffer);

	vertices_weight_prog.enable(ctx);

	glDispatchCompute(Voxelid.size(), 1, 1);

	// do something else
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	vertices_weight_prog.disable(ctx);


	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

	GLuint* results_ptr = static_cast<GLuint*>(glMapNamedBufferRange(
		Vert_results_buffer, 0, sizeof(float) * length_vertives, GL_MAP_READ_BIT));

	std::memcpy(results.data(), results_ptr, results.size() * sizeof(float));
	glUnmapNamedBuffer(Vert_results_buffer);

	std::vector<float> new_result(results.begin(), results.end());

	Vertices = new_result;




*/