#include "epa.hpp"


bool same_direction_b(const vec3& direction, const vec3& ao)
{
	return ti::dot(direction, ao) > 0;
}

std::tuple<std::vector<vec3>, std::vector<scalar> , size_t> get_face_normals(
	const std::vector<vec3>& polytope,
	const std::vector<size_t>& faces)
{
	std::vector<vec3> normals;
	std::vector<scalar> distances;
	size_t min_triangle = 0;
	scalar  min_distance = INFINITY;

	for (size_t i = 0; i < faces.size(); i += 3) {
		vec3 a = polytope[faces[i    ]];
		vec3 b = polytope[faces[i + 1]];
		vec3 c = polytope[faces[i + 2]];

		vec3 normal = ti::normalize(ti::cross(b - a, c - a));
		scalar distance = ti::dot(normal, a);

		if (distance < 0) {
			normal   *= -1;
			distance *= -1;
		}

		normals.push_back(normal);
		distances.push_back(distance);
		if (distance < min_distance) {
			min_triangle = i / 3;
			min_distance = distance;
		}
	}

	return { normals, distances, min_triangle };
}




void add_if_unique_edge(
	std::vector<std::pair<size_t, size_t>>& edges,
	const std::vector<size_t>& faces,
	size_t a,
	size_t b)
{
	auto reverse = std::find(                       //      0--<--3
		edges.begin(),                              //     / \ B /   A: 2-0
		edges.end(),                                //    / A \ /    B: 0-2
		std::make_pair(faces[b], faces[a])          //   1-->--2
	);
 
	if (reverse != edges.end()) {
		edges.erase(reverse);
	}
 
	else {
		edges.emplace_back(faces[a], faces[b]);
	}
}


EPACollisionResponse epa(
    Simplex &simplex,
    const Collider &colliderA,
    const Collider &colliderB)
{

    std::vector<vec3> polytope;//(simplex.begin(), simplex.end());
	for (int i = 0; i < simplex.size(); i++){
		vec3 point = simplex[i];
		polytope.push_back(point);
	}

    std::vector<size_t> faces = {
        0, 1, 2,
        0, 3, 1,
        0, 2, 3,
        1, 3, 2};

    // list: vec4(normal, distance), index: min distance
    auto [normals, distances, min_face] = get_face_normals(polytope, faces);

    vec3  min_normal;
	scalar min_distance = INFINITY;
	
	while (min_distance == INFINITY) {
		min_normal   = normals[min_face];
		min_distance = distances[min_face];
 
		vec3 support = support_function(colliderA, colliderB, min_normal);
		scalar sDistance = dot(min_normal, support);
 
		if (ti::abs(sDistance - min_distance) > 0.001) {
			min_distance = INFINITY;

            std::vector<std::pair<size_t, size_t>> uniqueEdges;

			for (size_t i = 0; i < normals.size(); i++) {
				if (same_direction_b(normals[i], support)) {
					size_t f = i * 3;

					add_if_unique_edge(uniqueEdges, faces, f,     f + 1);
					add_if_unique_edge(uniqueEdges, faces, f + 1, f + 2);
					add_if_unique_edge(uniqueEdges, faces, f + 2, f    );

					faces[f + 2] = faces.back(); faces.pop_back();
					faces[f + 1] = faces.back(); faces.pop_back();
					faces[f    ] = faces.back(); faces.pop_back();

					normals[i] = normals.back(); // pop-erase
					normals.pop_back();

					i--;
				}
			}

			std::vector<size_t> new_faces;
			for (auto [edgeIndex1, edgeIndex2] : uniqueEdges) {
				new_faces.push_back(edgeIndex1);
				new_faces.push_back(edgeIndex2);
				new_faces.push_back(polytope.size());
			}
			 
			polytope.push_back(support);

			auto [new_normals, new_distances, new_min_face] = get_face_normals(polytope, new_faces);

			scalar old_min_distance = INFINITY;
			for (size_t i = 0; i < normals.size(); i++) {
				if (new_distances[i] < old_min_distance) {
					old_min_distance = new_distances[i];
					min_face = i;
				}
			}
 
			if (new_distances[new_min_face] < old_min_distance) {
				min_face = new_min_face + normals.size();
			}
 
			faces  .insert(faces  .end(), new_faces  .begin(), new_faces  .end());
			normals.insert(normals.end(), new_normals.begin(), new_normals.end());

        }
    }

	EPACollisionResponse response;
 
	response.normal = min_normal;
	response.penetration_depth = min_distance + 0.001;
	response.collision = true;
 
	return response;
}





