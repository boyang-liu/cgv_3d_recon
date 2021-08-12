#pragma once

#include <tuple>

template <typename POINT>
class chunks : public cgv::render::render_types {
	std::vector<uint32_t> non_empty_chunks;

	cgv::render::render_types::box3 point_cloud_bounding_box_p;
	size_t num_chunks_p = 0;
	int32_t dim_chunks[4] = {0,0,0,0};
	float chunk_extents_p = 1.0;

public:
	//make all chunk classes friends
	template <typename>
	friend class chunks;

	std::vector<uint32_t> start_indices;
	std::vector<uint32_t> chunk_sizes;
	std::vector<cgv::render::render_types::box3> bounding_boxes; // 
	std::vector<POINT> points; // all points 

	// exisiting chunks
	size_t num_chunks() const {
		return num_chunks_p;
	}
	// number of points stored by this data structure
	size_t num_points() const {
		return points.size();
	}

	size_t num_filled_chunks() const {
		return non_empty_chunks.size();
	}
	
	uint32_t num_points_in_chunk(uint32_t chunk_id) {
		return chunk_sizes[chunk_id];
	}

	// returns the index of the first point belonging to the chunk refered by chunk_id
	uint32_t start_index(uint32_t chunk_id) {
		return start_indices[chunk_id];
	}

	float chunk_cube_size() const {
		return chunk_extents_p;
	}

	box3 bounding_box_of(int x, int y, int z) {
		vec3 chunk_cube = vec3(chunk_cube_size());
		return box3(point_cloud_bounding_box_p.get_min_pnt()+vec3(x,y,z)* chunk_cube, point_cloud_bounding_box_p.get_min_pnt() + vec3(x+1, y+1, z+1) * chunk_cube);
	}

	// chunkwise copies points from src_points over to the array stored at target_points, returns a list of indices to the first point of each chunk in target points 
	// template parameter is a struct with a position() method which returns a vec3
	void create_chunks_from(const POINT* src_points, const size_t num_points, float chunk_cube_extents) {
		const POINT* src_points_end = src_points + num_points;
		chunk_extents_p = chunk_cube_extents;

		//find limits
		for (const POINT* pnt = src_points; pnt < src_points_end; ++pnt) {
			point_cloud_bounding_box_p.add_point(pnt->position());
		}

		vec3 chunks_dims = (point_cloud_bounding_box_p.get_extent() / chunk_cube_extents) + 1; // add one for safety, ceil does not work? we have overflow sometimes 
		chunks_dims.ceil();

		dim_chunks[0] = (int32_t)chunks_dims.x();
		dim_chunks[1] = (int32_t)chunks_dims.y();
		dim_chunks[2] = (int32_t)chunks_dims.z();
		dim_chunks[3] = dim_chunks[0] * dim_chunks[1];
		num_chunks_p = dim_chunks[0] * dim_chunks[1] * dim_chunks[2];

		std::vector<std::vector<POINT>> chunk_data;
		chunk_data.resize(num_chunks_p);
		//copy points
		for (const POINT* p = src_points; p < src_points_end; ++p) {
			chunk_data[chunk_index_from_point(p->position())].push_back(*p);
		}
		//reshape data so everything is contained in start_indices_p, chunk_size_p and point_data_p
		size_t current_point = 0;
		points.resize(num_points);
		{
			int x = 0, y = 0, z = 0;
			
			int i = 0;
			for (int z = 0; z < dim_chunks[2]; ++z) {
				for (int y = 0; y < dim_chunks[1]; ++y) {
					for (int x = 0; x < dim_chunks[0]; ++x) {
						auto& chunk = chunk_data[i];
						start_indices.push_back(current_point);
						chunk_sizes.push_back(chunk.size());
						bounding_boxes.push_back(bounding_box_of(x, y, z));
						if (chunk.size() > 0) {
							memcpy(&points[current_point], chunk.data(), chunk.size() * sizeof(POINT));
							non_empty_chunks.push_back(i);
						}
						current_point += chunk.size();
						++i;
					}
				}
			}
		}
	}

	uint32_t chunk_index(const int x, const int y, const int z) const {
		assert(x < dim_chunks[0]&& x >= 0);
		assert(y < dim_chunks[1]&& y >= 0);
		assert(z < dim_chunks[2]&& z >= 0);
		return unchecked_chunk_index(x,y,z);
	}

	uint32_t unchecked_chunk_index(const int& x, const int& y, const int& z) const {
		return z * dim_chunks[3] + y * dim_chunks[0] + x;
	}

	uint32_t chunk_index_from_point(const vec3& pnt) const {
		dvec3 tmp = dvec3(pnt-point_cloud_bounding_box_p.get_min_pnt()) / chunk_cube_size();
		tmp.floor();
		return chunk_index(tmp.x(), tmp.y(), tmp.z());
	}

	chunks() = default;
	chunks(const std::vector<POINT>&src, float chunk_cube_size) {
		create_chunks_from(src.data(), src.size(), chunk_cube_size);
	}

	//point format conversion move operation
	template<typename P>
	void move_from_chunks(chunks<P>&& other_chunks) {
		this->non_empty_chunks.swap(other_chunks.non_empty_chunks);
		this->point_cloud_bounding_box_p = other_chunks.point_cloud_bounding_box_p;
		this->num_chunks_p = other_chunks.num_chunks_p;
		for (int i = 0; i < 4; ++i) {
			this->dim_chunks[i] = other_chunks.dim_chunks[i];
		}
		this->chunk_extents_p = other_chunks.chunk_extents_p;

		this->start_indices.swap(other_chunks.start_indices);
		this->chunk_sizes = other_chunks.chunk_sizes;
		this->bounding_boxes.swap(other_chunks.bounding_boxes);
		
		points.resize(other_chunks.points.size());
		for (int i = 0; i < other_chunks.points.size();++i) {
			this->points[i] = (POINT)other_chunks.points[i];
		}
		other_chunks.points.clear();
	}

	const std::vector<uint32_t>& get_filled_chunks_ids() {
		return non_empty_chunks;
	}

	// update after scaling, does not work 
	void update_bbox(mat4 model_transform) {

		std::cout << "bbox min point before: " << point_cloud_bounding_box_p.ref_min_pnt() << std::endl;
		std::cout << "bbox max point before: " << point_cloud_bounding_box_p.ref_max_pnt() << std::endl;

		vec4 tmp;
		tmp = model_transform * point_cloud_bounding_box_p.ref_min_pnt().lift();
		point_cloud_bounding_box_p.ref_min_pnt() = vec3(tmp.x(), tmp.y(), tmp.z());
		std::cout << "bbox min point: " << point_cloud_bounding_box_p.ref_min_pnt() << std::endl;

		tmp = model_transform * point_cloud_bounding_box_p.ref_max_pnt().lift();
		point_cloud_bounding_box_p.ref_max_pnt() = vec3(tmp.x(), tmp.y(), tmp.z());
		std::cout << "bbox max point: " << point_cloud_bounding_box_p.ref_max_pnt() << std::endl;
	}

	// writes the ids of the chunks inside the frustum to the vector in chunk_ids
	void intersect_chunks(const vec4* planes,std::vector<uint32_t>& chunk_ids) {
		for (uint32_t cid : non_empty_chunks) {
			bool accept = true;
			//check planes
			for (int j = 0; j < 6; ++j) {
				bool rejected_by_plane = true;
				const vec4& plane = planes[j];
				//check box corners
				for (int k = 0; k < 8; ++k) {
					vec4 corner = bounding_boxes[cid].get_corner(k).lift();
					if (dot(corner, plane) > 0.0) {
						rejected_by_plane = false;
						break;
					}
				}
				if (rejected_by_plane) {
					accept = false;
					break;
				}
			}
			if (accept) {
				chunk_ids.push_back(cid);
			}
		}
	}

	std::array<int,3> xyz_indices_from_point(const vec3& pnt) {
		vec3 tmp = (pnt - point_cloud_bounding_box_p.get_min_pnt()) / chunk_cube_size();
		tmp.floor();
		return std::array<int, 3> {(int)tmp.x(), (int)tmp.y(), (int)tmp.z()};
	}

	// writes the ids of the chunks intersecting with the sphere to the vector in chunk_ids
	void intersect_sphere(const vec3& sphere_pos,const float radius, std::vector<uint32_t>& chunk_ids) {
		float sqradius = radius * radius;
		auto base = xyz_indices_from_point(sphere_pos);

		auto min_xyz = xyz_indices_from_point(sphere_pos-radius);
		auto max_xyz = xyz_indices_from_point(sphere_pos+radius);

		// enforce boundaries on min_xyz
		for (auto& i : min_xyz) {
			i = i < 0 ? 0 : i;
		}
		for (int i = 0; i < 3;++i) {
			max_xyz[i] = max_xyz[i] < dim_chunks[i] ? max_xyz[i] : dim_chunks[i] - 1; // to my understanding, we have to assign to max_xyz[i]? 
		}

		for (int z = min_xyz[2]; z <= max_xyz[2]; ++z) {
			for (int y = min_xyz[1]; y <= max_xyz[1]; ++y) {
				for (int x = min_xyz[0]; x <= max_xyz[0]; ++x) {
					auto ix = unchecked_chunk_index(x, y, z);
					chunk_ids.push_back(ix);
				}
			}
		}
	}

	//selects every chunk bellow and intersecting with the clipping plane
	void intersect_clip_plane(const vec3& plane_normal, const float plane_distance, std::vector<uint32_t>& chunk_ids) {
		for (uint32_t cid : non_empty_chunks) {
			// Convert AABB to center-extents representation
			vec3 c = bounding_boxes[cid].get_center();
			vec3 e = bounding_boxes[cid].get_max_pnt() - c; // Compute positive extents

			// Compute the projection interval radius of b onto L(t) = b.c + t * p.n
			float proj_radius = dot(abs(plane_normal), e);

			// Compute distance of box center from plane
			float s = dot(plane_normal, c) - plane_distance;

			// Intersection occurs when distance s falls within [-inf,+r] interval
			if (s <= proj_radius) {
				chunk_ids.push_back(cid);
			}
		}
	}

	//selects chunks intersecting with the plane
	void intersect_plane(const vec3& plane_normal, const float plane_distance, std::vector<uint32_t>& chunk_ids) {
		for (uint32_t cid : non_empty_chunks) {

			// Convert AABB to center-extents representation
			vec3 c = bounding_boxes[cid].get_center();
			vec3 e = bounding_boxes[cid].get_max_pnt() - c; // Compute positive extents

			// Compute the projection interval radius of b onto L(t) = b.c + t * p.n
			float proj_radius = dot(abs(plane_normal), e);

			// Compute distance of box center from plane
			float s = dot(plane_normal, c) - plane_distance;

			// Intersection occurs when distance s falls within [-r,+r] interval
			if (abs(s) <= proj_radius) {
				chunk_ids.push_back(cid);
			}
		}
	}

	// writes the ids of the chunks intersecting with the cube to the vector in chunk_ids
	void intersect_cube(const vec3& box_pos, const vec3 max_ext, const vec3 min_ext, std::vector<uint32_t>& chunk_ids) {
		//float sqradius = radius * radius;
		auto base = xyz_indices_from_point(box_pos);

		auto min_xyz = xyz_indices_from_point(min_ext);
		auto max_xyz = xyz_indices_from_point(max_ext);

		// enforce boundaries on min_xyz
		for (auto& i : min_xyz) {
			i = i < 0 ? 0 : i;
		}
		for (int i = 0; i < 3; ++i) {
			max_xyz[i] = max_xyz[i] < dim_chunks[i] ? max_xyz[i] : dim_chunks[i] - 1; // to my understanding, we have to assign to max_xyz[i]? 
		}

		for (int z = min_xyz[2]; z <= max_xyz[2]; ++z) {
			for (int y = min_xyz[1]; y <= max_xyz[1]; ++y) {
				for (int x = min_xyz[0]; x <= max_xyz[0]; ++x) {
					auto ix = unchecked_chunk_index(x, y, z);
					chunk_ids.push_back(ix);
				}
			}
		}
	}
};