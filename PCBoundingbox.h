#pragma once
//#include <cgv/media/axis_aligned_box.h>
class PCBoundingbox
{

	typedef cgv::math::fvec<float, 3> vec3;
	
public:
	PCBoundingbox() {
		pos1 = (0, 0, 0);
		pos2 = (0, 0, 0);
	}
	vec3 pos1;
	vec3 pos2;
	vec3 boxcenter;
	float height;
	float length;
	float width;
	float step;
	vec3 getpos1() { return boxcenter - vec3(boxcenter[0] - length / 2, boxcenter[1] - length / 2, boxcenter[2] - height / 2); }
	vec3 getpos2() { return boxcenter - vec3(boxcenter[0] + length / 2, boxcenter[1] + length / 2, boxcenter[2] + height / 2); }
	/*void initvoxelindex() {
		voxelindex.resize(length / step);
		for (int i = 0; i < voxelindex.size(); i++) {
			voxelindex[i].resize(width / step);
			for (int j = 0; j < voxelindex.size(); i++)
			{
				voxelindex[j].resize(height / step);
			}
		}
	
	}*/
protected:
	std::vector<std::vector<std::vector<int>>> voxelindex;
};

