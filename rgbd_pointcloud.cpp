#include "rgbd_pointcloud.h"
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






bool rgbd_pointcloud::write_pc(const std::string& _file_name)
{
	string ext = to_lower(get_extension(_file_name));
	if (ext == "lbypc")
		return write_lbypc(_file_name);
	if (ext == "txt")
		return write_txt(_file_name);
	return false;
}
bool rgbd_pointcloud::read_pc(const std::string& _file_name)
{
	string ext = to_lower(get_extension(_file_name));
	if (ext == "lbypc")
		return read_lbypc(_file_name);
	if (ext == "txt")
		return read_txt(_file_name);
	return false;
}

bool rgbd_pointcloud::write_lbypc(const std::string& file_name)
{
	FILE* fp = fopen(file_name.c_str(), "wb");
	if (!fp)
		return false;
	bool success = true;
	int n = Points.size();
	// write the header
	success = success && fwrite(&n, sizeof(int), 1, fp);
	// write the content 
	success = success && fwrite(&Points[0], sizeof(Pnt), n, fp) == n;
	success = success && fwrite(&Colors[0], sizeof(Rgba), n, fp) == n;




	return fclose(fp) == 0 && success;
}


bool rgbd_pointcloud::read_lbypc(const std::string& file_name)
{
	string ext = to_lower(get_extension(file_name));

	if (!(ext == "lbypc"))
		return false;

	FILE* fp = fopen(file_name.c_str(), "rb");
	if (!fp)
		return false;
	bool success = true;
	int nr;
	success = success && fread(&nr, sizeof(int), 1, fp) == 1;
	//success = success && fread(&flags, sizeof(cgv::type::uint32_type), 1, fp) == 1;
	Points.resize(nr);
	success = success && fread(&Points[0][0], sizeof(Pnt), nr, fp) == nr;
	Colors.resize(nr);
	success = success && fread(&Colors[0][0], sizeof(Rgba), nr, fp) == nr;

	return fclose(fp) == 0 && success;





}



bool rgbd_pointcloud::read_txt(const std::string& file_name)
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
	for (i = 0; i < lines.size(); ++i) {
		if (lines[i].empty())
			continue;

		if (true) {
			Pnt p;
			Rgba c;
			char tmp = lines[i].end[0];
			content[lines[i].end - content.c_str()] = 0;
			if (sscanf(lines[i].begin, "%f %f %f %d %d %d %d", &p[0], &p[1], &p[2], &c[0], &c[1], &c[2], &c[3]) == 7) {


				Points.push_back(p);
				Colors.push_back(c);
			}
			content[lines[i].end - content.c_str()] = tmp;
		}
		/*else {
			vector<token> numbers;
			tokenizer(lines[i]).bite_all(numbers);
			double values[7];
			unsigned n = min(7, (int)numbers.size());
			unsigned j;
			for (j = 0; j < n; ++j) {
				if (!is_double(numbers[j].begin, numbers[j].end, values[j]))
					break;
			}
			if (j >= 3 && j < 7)
			{
				Points.push_back(vec3(values[0],values[1], values[2]));
				Colors.push_back(rgba8(0, 0, 0,255));
			}
		}
		if ((Points.size() % 100000) == 0)
			cout << "read " << Points.size() << " points" << endl;*/
	}
	watch.add_time();
	return true;
}
bool rgbd_pointcloud::write_txt(const std::string& file_name)
{
	ofstream ofile;
	ofile.open(file_name.c_str());
	for (int i = 0; i < Points.size(); i++) {

		ofile << Points[i] << " ";
		ofile << Colors[i] << " ";
		ofile << "\n";
	}
	ofile.close();
	//success = success && fwrite(&flags, sizeof(cgv::type::uint32_type), 1, fp) == 1;
	// write the content 
	//for (int i = 0; i < n; i++)
	//{
		//fprintf(fp, "%f", "%f","%f", Points[i][0], Points[i][1], Points[i][2]);
		//fprintf(fp, "%f", "%f","%f", "%f", Colors[i][0], Colors[i][1], Colors[i][2], Colors[i][3]);
		//std::cout << "write" << std::endl;	
		//success = success && fwrite(&Points[i][0], sizeof(vec3), 1, fp);
		//success = success && fwrite(&Colors[i][0], sizeof(rgba8), 1, fp);
	//}
	return true;
}



size_t rgbd_pointcloud::add_point(const Pnt& p)
{
	Points.push_back(p);
	Colors.push_back(Rgba(0, 0, 0, 255));
	return Points.size();
}
size_t rgbd_pointcloud::add_point(const Pnt& p, const Rgba& c)
{
	Points.push_back(p);
	Colors.push_back(c);
	return Points.size();
}
void rgbd_pointcloud::resize(size_t nr_points) 
{
	Points.resize(nr_points);
	Colors.resize(nr_points);
	//labels.resize(nr_points);
	//renderColors.resize(nr_points);
}

void rgbd_pointcloud::clear()
{
	Points.clear();
	Colors.clear();
	labels.clear();
	renderColors.clear();
}
void rgbd_pointcloud::set_render_color() 
{
	
	renderColors = Colors;
	
	if (labels.size() > 0) {
		for (int i =0;i<labels.size();i++) 
		{
			renderColors[labels[i]]= (255, 0, 0, 255);
		}
	}
	return ;
}