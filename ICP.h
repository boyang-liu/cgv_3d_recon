#pragma once
#include <vector>
#include "rgbd_pointcloud.h"
#include "ann_tree.h"
#include <random>
#include <ctime>
#include <cgv/math/svd.h> 
#include "lib_begin.h"
#include <memory>

//namespace cgv {

	//namespace rgbd_pointcloud {

		class  ICP  : public point_cloud_types
		{
		public:
			enum Sampling_Type {
				RANDOM_SAMPLING = 0,
				NORMAL_SPACE_SAMPLING = 1,
			} S_type;

			const rgbd_pointcloud* sourceCloud;
			const rgbd_pointcloud* targetCloud;
			int maxIterations;
			int numRandomSamples;
			float eps;
			rgbd_pointcloud* crspd_source;
			rgbd_pointcloud* crspd_target;

			ICP();
			~ICP();

			void build_ann_tree();
			void clear();
			void set_source_cloud(const rgbd_pointcloud& inputCloud);
			void set_target_cloud(const rgbd_pointcloud& inputCloud, std::shared_ptr<ann_tree> precomputed_tree = nullptr);
			void set_iterations(int Iter);
			void set_num_random(int NR);
			void set_eps(float e);

			void reg_icp(Mat& rotation_m, Dir& translation_v);
			void get_center_point(const rgbd_pointcloud& input, Pnt& mid_point);
			float error(Pnt& ps, Pnt& pd, Mat& r, Dir& t);
			void get_crspd(Mat& rotation_m, Dir& translation_v, rgbd_pointcloud& pc1, rgbd_pointcloud& pc2);
			void print_rotation(float* rotationMatrix);
			void print_translation(float* translation);

			bool correspondences_filter(const rgbd_pointcloud& source, const rgbd_pointcloud& target, Pnt& source_p, Pnt& target_p);
			float dis_pts(const Pnt& source_p, const Pnt& target_p);

		private:
			std::shared_ptr<ann_tree> tree;
		};
	//}
//}
#include <cgv/config/lib_end.h>
