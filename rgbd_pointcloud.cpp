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






bool rgbd_pointcloud::write(const std::string& _file_name)
{
	string ext = to_lower(get_extension(_file_name));
	if (ext == "lbypc")
		return write_lbypc(_file_name);
	return false;
}
bool rgbd_pointcloud::write_lbypc(const std::string& file_name)
{
	FILE* fp = fopen(file_name.c_str(), "wb");
	if (!fp)
		return false;
	std::cout << "write:" << Points[0] << std::endl;
	std::cout << "write:" << Colors[0] << std::endl;
	bool success = true; 
	int n = Points.size();
	// write the content 
	for (int i = 0; i < n; i++)
	{
		std::cout << "write" << std::endl;
		success = success && fwrite(&Points[0][0], sizeof(float), 3, fp);
		success = success && fwrite(&Colors[0][0], sizeof(float), 4, fp);
	}
	return fclose(fp) == 0 && success;
}


bool rgbd_pointcloud::read_lbypc(const std::string& file_name) 
{
	string ext = to_lower(get_extension(file_name));

	if (!(ext == "lbypc"))
		return false;

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
			vec3 p;
			rgba8 c;
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
size_t rgbd_pointcloud::add_point(const vec3& p) 
{
	Points.push_back(p);
	Colors.push_back(rgba8(0, 0, 0, 255));
	return Points.size();
}
size_t rgbd_pointcloud::add_point(const vec3& p, const rgba8& c) 
{
	Points.push_back(p);
	Colors.push_back(c);
	return Points.size();
}
int rgbd_pointcloud::get_nr_Points()
{
	return Points.size();

}


