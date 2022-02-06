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


class point_cloud_obj_loader : public obj_reader, public point_cloud_types
{
protected:
	std::vector<Pnt>& P;
	std::vector<Nml>& N;
	std::vector<Clr>& C;
public:
	///
	point_cloud_obj_loader(std::vector<Pnt>& _P, std::vector<Nml>& _N, std::vector<Clr>& _C) : P(_P), N(_N), C(_C) {}
	/// overide this function to process a vertex
	void process_vertex(const v3d_type& p)
	{
		P.push_back(rgbd_pointcloud::Pnt(p));
	}
	/// overide this function to process a normal
	void process_normal(const v3d_type& n)
	{
		N.push_back(rgbd_pointcloud::Nml(n));
	}
	/// overide this function to process a color (this called for vc prefixes which is is not in the standard but for example used in pobj-files)
	void process_color(const color_type& c)
	{
		C.push_back(c);
	}

};



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
	if (ext == "obj" || ext == "pobj")
		return read_obj(_file_name);

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
	success = success && fwrite(&cam_pos, sizeof(Pnt), 1, fp) ;
	success = success && fwrite(&cam_rotation, sizeof(Mat), 1, fp);
	success = success && fwrite(&cam_translation, sizeof(Pnt), 1, fp);
	// write the content 
	if (get_nr_Points() != 0) {
	success = success && fwrite(&Points[0], sizeof(Pnt), n, fp) == n;
	success = success && fwrite(&Colors[0], sizeof(Rgba), n, fp) == n;
	}



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

	success = success && fread(&cam_pos[0], sizeof(Pnt), 1, fp) == 1;

	success = success && fread(&cam_rotation[0], sizeof(Mat), 1, fp) == 1;

	success = success && fread(&cam_translation[0], sizeof(Pnt), 1, fp) == 1;

	


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
	Pnt c_p;
	Mat c_r;
	Pnt c_t;
	if (lines[0].empty())
	if (sscanf(lines[0].begin, "%f %f %f ", &c_p[0], &c_p[1], &c_p[2]) == 3) {


		cam_pos = c_p;
	}

	if (sscanf(lines[1].begin, "%f %f %f ", &c_r[0], &c_r[1], &c_r[2]) == 3) {
		cam_rotation[0] = c_r[0];
		cam_rotation[1] = c_r[1];
		cam_rotation[2] = c_r[2];
	}
	if (sscanf(lines[2].begin, "%f %f %f ", &c_r[3], &c_r[4], &c_r[5]) == 3) {
		cam_rotation[3] = c_r[3];
		cam_rotation[4] = c_r[4];
		cam_rotation[5] = c_r[5];
	}
	if (sscanf(lines[3].begin, "%f %f %f ", &c_r[6], &c_r[7], &c_r[8]) == 3) {
		cam_rotation[6] = c_r[6];
		cam_rotation[7] = c_r[7];
		cam_rotation[8] = c_r[8];
	}
	if (sscanf(lines[4].begin, "%f %f %f ", &c_t[0], &c_t[1], &c_t[2]) == 3) {


		cam_translation = c_t;
	}


	for (i = 5; i < lines.size(); ++i) {
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

bool rgbd_pointcloud::read_obj(const std::string& _file_name)
{
	vector<Clr> myColor;
	point_cloud_obj_loader pc_obj(Points, Normals, myColor);
	if (!exists(_file_name))
		return false;
	clear();
	if (!pc_obj.read_obj(_file_name))
		return false;
	Colors.clear();
	if (myColor.size() == 0)
		Colors.resize(Points.size(),Rgba(255,255,255,255));
	else
	{
	for (int i = 0; i < myColor.size(); i++)
		Colors.emplace_back(Rgba(myColor[i][0], myColor[i][1], myColor[i][2], 255));
	}
	return true;
}






bool rgbd_pointcloud::write_txt(const std::string& file_name)
{
	ofstream ofile;
	ofile.open(file_name.c_str());
	ofile << cam_pos << " ";
	ofile << "\n";
	ofile << cam_rotation << " ";
	ofile << "\n";
	ofile << cam_translation << " ";
	ofile << "\n";
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

bool rgbd_pointcloud::write_pts(const std::string& file_name) {

	ofstream ofile;
	ofile.open(file_name.c_str());
	
	for (int i = 0; i < Points.size(); i++) {

		ofile << Points[i] << " ";
		
		ofile << "\n";

	}
	ofile.close();

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
			renderColors[labels[i]]= Rgba(255, 0, 0, 255);
		}
	}
	
	return ;
}
void rgbd_pointcloud::merge_labels(std::vector<int>& a) {
	sort(a.begin(), a.end());
	sort(labels.begin(), labels.end());
	std::vector<int> result;
	std::set_union(std::begin(a), std::end(a), 
	std::begin(labels), std::end(labels), 
	std::back_inserter(result));
	labels = result;
}
void rgbd_pointcloud::delete_labels(std::vector<int>& a) {
	if (labels.size() == 0)
		return;
	int current = 0;
	std::vector<int> newlabels;
	sort(a.begin(), a.end());
	for(int i=0; i<labels.size();i++)
	{
		newlabels.push_back(lab(i));
		for (int j = 0; j < a.size(); j++) {
			if (lab(i) == a[j])
			{
				newlabels.pop_back();
				break;
			}
			
		}
		
	}
	labels = newlabels;
}


void rgbd_pointcloud::do_transformation(Mat& rotation_mat,Pnt translation_vec) {

	
	if(get_nr_Points()==0)
	{
		std::cout<<"no point cloud"<<std::endl;
		return;
	}
	for (int i = 0; i < get_nr_Points(); i++)
	{
		Points[i] = rotation_mat * Points[i] + translation_vec;
	}

}
void rgbd_pointcloud::do_transformation(Mat& rotation_mat) {


	if (get_nr_Points() == 0)
	{
		std::cout << "no point cloud" << std::endl;
		return;
	}
	for (int i = 0; i < get_nr_Points(); i++)
	{
		Points[i] = rotation_mat * Points[i] ;
	}

}
void rgbd_pointcloud::do_transformation( Pnt translation_vec) {


	if (get_nr_Points() == 0)
	{
		std::cout << "no point cloud" << std::endl;
		return;
	}
	for (int i = 0; i < get_nr_Points(); i++)
	{
		Points[i] = Points[i]+ translation_vec;
	}

}

void  rgbd_pointcloud::append(const rgbd_pointcloud& pc) {
	if (pc.get_nr_Points() == 0)
		return;
	Cnt old_n = (Cnt)Points.size();
	Cnt n = (Cnt)Points.size() + pc.get_nr_Points();
	Points.resize(n);
	std::copy(pc.Points.begin(), pc.Points.end(), Points.begin() + old_n);
	Colors.resize(n);
	std::copy(pc.Colors.begin(), pc.Colors.end(), Colors.begin() + old_n);

	if (pc.labels.size() == 0)
		return;
	Cnt old_l = (Cnt)labels.size();
	Cnt l = (Cnt)labels.size() + pc.labels.size();

	for (int i = old_l; i < l; i++)
		labels[i] = pc.lab(i) + old_l;
	return;
}
void rgbd_pointcloud::create_normals(){
	has_nmls = true;
	Normals.resize(Points.size());
}
void rgbd_pointcloud::delete_labeled_points() {
	for (int i = 1; i <= labels.size(); i++) {
		Points.erase(Points.begin() + labels[labels.size() - i]);
		Colors.erase(Colors.begin() + labels[labels.size() - i]);
	}
	labels.clear();
	set_render_color();
}