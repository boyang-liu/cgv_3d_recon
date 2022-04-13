#include "rgbd_depth.h"
#include <cgv/base/base.h> // this should be first header to avoid warning
#include <omp.h>
#include <cgv_gl/gl/gl.h>
#include <cgv/gui/trigger.h>
#include <cgv/gui/dialog.h>
#include <cgv/gui/key_event.h>
#include <cgv/gui/file_dialog.h>
#include <cgv/utils/convert.h>
#include <cgv/utils/file.h>
#include <cgv/utils/dir.h>
#include <cgv/utils/statistics.h>
#include <cgv/type/standard_types.h>
#include <cgv/math/ftransform.h>
#include <cgv/math/svd.h>
#include <cgv/utils/file.h>
#include <cgv/utils/scan.h>
#include <cgv/utils/advanced_scan.h>
#include <cgv/utils/stopwatch.h>
#include <cgv/media/mesh/obj_reader.h>

using namespace std;
using namespace cgv::base;
using namespace cgv::signal;
using namespace cgv::type;
using namespace cgv::gui;
using namespace cgv::data;
using namespace cgv::render;
using namespace cgv::utils::file;
using namespace cgv::utils;
using namespace cgv::media::mesh;


bool rgbd_depth::write_rgbd_depth(const std::string& _file_name)
{
	string ext = to_lower(get_extension(_file_name));
	if (ext == "rd")
		return write_rd(_file_name);
	if (ext == "txt")
		return write_txt(_file_name);
	return false;
}
bool rgbd_depth::read_rgbd_depth(const std::string& _file_name)
{
	string ext = to_lower(get_extension(_file_name));
	if (ext == "rd")
		return read_rd(_file_name);
	if (ext == "txt")
		return read_txt(_file_name);

	return false;
}



bool rgbd_depth::read_rd(const std::string& file_name)
{
	string ext = to_lower(get_extension(file_name));
	if (!(ext == "rd"))
		return false;

	FILE* fp = fopen(file_name.c_str(), "rb");
	if (!fp)
		return false;
	bool success = true;

	int w;
	success = success && fread(&w, sizeof(int), 1, fp) == 1;
	int h;
	success = success && fread(&h, sizeof(int), 1, fp) == 1;
	width = w;
	height = h;


	success = success && fread(&camera_pos[0], sizeof(Pnt), 1, fp) == 1;

	success = success && fread(&camera_rotation[0], sizeof(Mat), 1, fp) == 1;

	success = success && fread(&camera_translation[0], sizeof(Pnt), 1, fp) == 1;

	int nr = w * h;
	Pixels.resize(nr);
	success = success && fread(&Pixels[0], sizeof(int), nr, fp) == nr;
	
	return fclose(fp) == 0 && success;


}
bool rgbd_depth::write_rd(const std::string& file_name)
{
	FILE* fp = fopen(file_name.c_str(), "wb");
	if (!fp)
		return false;
	bool success = true;
	int w = width;
	int h = height;
	int n = width * height;
	// write the header
	success = success && fwrite(&w, sizeof(int), 1, fp);
	success = success && fwrite(&h, sizeof(int), 1, fp);
	success = success && fwrite(&camera_pos, sizeof(Pnt), 1, fp);
	success = success && fwrite(&camera_rotation, sizeof(Mat), 1, fp);
	success = success && fwrite(&camera_translation, sizeof(Pnt), 1, fp);
	// write the content 
	if (get_nr_pixels() != 0) {
		success = success && fwrite(&Pixels[0], sizeof(int), n, fp) == n;	
	}



	return fclose(fp) == 0 && success;

}
bool rgbd_depth::read_txt(const std::string& file_name)
{
	string content;
	cgv::utils::stopwatch watch;
	if (!cgv::utils::file::read(file_name, content, true))
		return false;
	std::cout << "read data from disk "; watch.add_time();
	//clear();
	vector<line> lines;
	split_to_lines(content, lines);
	std::cout << "split data into " << lines.size() << " lines. ";	watch.add_time();

	bool do_parse = false;
	unsigned i;
	Pnt c_p;
	Mat c_r;
	Pnt c_t;
	int c_w;
	int c_h;

	double p_1;
	double p_2;
	double p_3;
	double p_4;
	double p_5;

	if (sscanf(lines[0].begin, "%d  ", &c_w) == 1) {

		width = c_w;
	}
	
	if (sscanf(lines[1].begin, "%d  ", &c_h) == 1) {

		height = c_h;
	}
	if (sscanf(lines[2].begin, "%f %f %f ", &c_p[0], &c_p[1], &c_p[2]) == 3) {


		camera_pos = c_p;
	}






	if (sscanf(lines[3].begin, "%f %f %f ", &c_r[0], &c_r[1], &c_r[2]) == 3) {
		camera_rotation[0] = c_r[0];
		camera_rotation[1] = c_r[1];
		camera_rotation[2] = c_r[2];
	}
	if (sscanf(lines[4].begin, "%f %f %f ", &c_r[3], &c_r[4], &c_r[5]) == 3) {
		camera_rotation[3] = c_r[3];
		camera_rotation[4] = c_r[4];
		camera_rotation[5] = c_r[5];
	}
	if (sscanf(lines[5].begin, "%f %f %f ", &c_r[6], &c_r[7], &c_r[8]) == 3) {
		camera_rotation[6] = c_r[6];
		camera_rotation[7] = c_r[7];
		camera_rotation[8] = c_r[8];
	}
	if (sscanf(lines[6].begin, "%f %f %f ", &c_t[0], &c_t[1], &c_t[2]) == 3) {


		camera_translation = c_t;
	}






	if (sscanf(lines[7].begin, "%lf  ", &p_1) == 1) {

		para.fx_d = p_1;
	}
	if (sscanf(lines[8].begin, "%lf  ", &p_2) == 1) {

		para.fy_d = p_2;
	}
	if (sscanf(lines[9].begin, "%lf  ", &p_3) == 1) {

		para.cx_d = p_3;
	}
	if (sscanf(lines[10].begin, "%lf  ", &p_4) == 1) {

		para.cy_d = p_4;
	}
	if (sscanf(lines[11].begin, "%lf  ", &p_5) == 1) {

		para.depth_scale = p_5;
	}



	for (i = 12; i < lines.size(); ++i) {
		if (lines[i].empty())
			continue;

		if (true) {
			int p;
			char tmp = lines[i].end[0];
			content[lines[i].end - content.c_str()] = 0;
			if (sscanf(lines[i].begin, "%d ", &p) == 1) {


				Pixels.push_back(p);
				
			}
			content[lines[i].end - content.c_str()] = tmp;
		}
		
	}
	watch.add_time();
	return true;

}
bool rgbd_depth::write_txt(const std::string& file_name)
{
	ofstream ofile;
	ofile.open(file_name.c_str());
	ofile << width << " ";
	ofile << "\n";	
	ofile << height << " ";
	ofile << "\n";

	ofile << camera_pos << " ";
	ofile << "\n";
	ofile << camera_rotation << " ";
	ofile << "\n";
	ofile << camera_translation << " ";
	ofile << "\n";

	ofile << para.fx_d << " ";
	ofile << "\n";
	ofile << para.fy_d << " ";
	ofile << "\n";
	ofile << para.cx_d << " ";
	ofile << "\n";
	ofile << para.cy_d << " ";
	ofile << "\n";
	ofile << para.depth_scale << " ";
	ofile << "\n";

	std::cout<<"Pixels.size():" << Pixels.size() << std::endl;
	for (int i = 0; i < Pixels.size(); i++) {

		ofile << Pixels[i] << " ";
		ofile << "\n";

	}
	ofile.close();
	
	return true;

}

bool rgbd_depth::V1_map_depth_to_point(int x, int y, int depth, float* point_ptr) const
{
	
	return true;
}





void rgbd_depth::clear_depth() {

	width = 0;
	height = 0;
	Pixels.clear();
	camera_pos = Pnt(0, 0, 0);
	camera_rotation.identity();
	camera_translation = Pnt(0, 0, 0);


}