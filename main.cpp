#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include <fstream>
#include <iostream>
#include <list>

#include "stb_image.h"

#include <random>
static std::default_random_engine engine(10); // random seed=10
static std::uniform_real_distribution<double>uniform(0,1);

static inline double sqr(double x) { return x * x; }

class Vector {
public:
	explicit Vector(double x = 0, double y = 0, double z = 0) {
		coord[0] = x;
		coord[1] = y;
		coord[2] = z;
	}
	double& operator[](int i) { return coord[i]; }
	double operator[](int i) const { return coord[i]; }

	Vector& operator+=(const Vector& v) {
		coord[0] += v[0];
		coord[1] += v[1];
		coord[2] += v[2];
		return *this;
	}

	double norm2() const {
		return sqr(coord[0]) + sqr(coord[1]) + sqr(coord[2]);
	}

	void normalize() {
		double norm = sqrt(norm2());
		if (norm > 0) {
			coord[0] /= norm;
			coord[1] /= norm;
			coord[2] /= norm;
		}
	}

	double coord[3];
};

Vector operator+(const Vector& a, const Vector& b) {
	return Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}
Vector operator-(const Vector& a, const Vector& b) {
	return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
Vector operator-(const Vector& a) {
	return Vector(-a[0], -a[1], -a[2]);
}
Vector operator*(const Vector& a, double b) {
	return Vector(a[0]*b, a[1]*b, a[2]*b);
}
Vector operator*(double a, const Vector& b) {
	return Vector(a*b[0], a*b[1], a*b[2]);
}
Vector operator*(const Vector& a, const Vector& b) {
	return Vector(a[0]*b[0], a[1]*b[1], a[2]*b[2]);
}

Vector operator/(const Vector& b, double a) {
	return Vector(b[0]/a, b[1]/a, b[2]/a);
}

double dot(const Vector& a, const Vector& b) {
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vector cross(const Vector& a, const Vector& b) {
	return Vector(a[1]*b[2]- a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]);
}

int getMaxDimension(Vector v) {
	if (v[0] > v[1] && v[0] > v[2]) {
		return 0;
	}
	if (v[1] > v[2] && v[1] > v[0]) {
		return 1;
	}
	return 2;
}

class Ray {
public:
	explicit Ray(const Vector& origin, const Vector& direction) : O(origin), u(direction) {};
	Vector O;
	Vector u;
};


class Geometry {
  public:
    Geometry(const Vector& Albedo, const bool Mirror = false, const bool isTransparent = false) : Albedo(Albedo), Mirror(Mirror), isTransparent(isTransparent) {};
	virtual double intersect(const Ray& r, Vector& P, Vector& N) const = 0;

	Vector Albedo;
	bool Mirror;
	bool isTransparent;
};

class TriangleIndices {
public:
    TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1, bool added = false) : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk), ni(ni), nj(nj), nk(nk), group(group) {
    };
    int vtxi, vtxj, vtxk; // indices within the vertex coordinates array
    int uvi, uvj, uvk;  // indices within the uv coordinates array
    int ni, nj, nk;  // indices within the normals array
    int group;       // face group
};

class BoundingBox {
	public:
		BoundingBox() {}
		BoundingBox(Vector& m, Vector& M): m(m), M(M) {}

		// Checks if a ray intersect with the bounding box
		bool intersect(const Ray& r) {
			double tmin[3],tmax[3];
			Vector Om = m - r.O;
			Vector OM = M - r.O;
			for (int i = 0; i < 3; i++) {
				double t1 = Om[i]/r.u[i];
				double t2 = OM[i]/r.u[i];
				tmin[i] = std::min(t1, t2);
				tmax[i] = std::max(t1, t2);
			}

			double exit = std::min(tmax[0],std::min(tmax[1], tmax[2]));
			return std::max(tmin[0],std::max(tmin[1], tmin[2])) < exit && exit > 0;
		}

	Vector m;
	Vector M;
};

class BVH {
	public:
		BVH() {}
		BVH(BoundingBox& bbox): bbox(bbox) {}

	BVH *left, *right;
	BoundingBox bbox;
	int start, end;

};


class TriangleMesh : public Geometry {
public:
  ~TriangleMesh() {}
    explicit TriangleMesh(const Vector& Albedo, const bool Mirror = false, const bool isTransparent = false): Geometry(Albedo, Mirror, isTransparent) {};

	// Compute the bounding box for a given set of vertices
	BoundingBox compute_bbox(int start, int end) {
		Vector m,M;
		for (int j=0;j<3;j++) {
			double min = vertices[indices[start].vtxi][j],max = vertices[indices[start].vtxi][j];
			for (int i=start; i<end; i++) {
				if (vertices[indices[i].vtxi][j] < min) min = vertices[indices[i].vtxi][j];
				if (vertices[indices[i].vtxi][j] > max) max = vertices[indices[i].vtxi][j];

				if (vertices[indices[i].vtxj][j] < min) min = vertices[indices[i].vtxj][j];
				if (vertices[indices[i].vtxj][j] > max) max = vertices[indices[i].vtxj][j];

				if (vertices[indices[i].vtxk][j] < min) min = vertices[indices[i].vtxk][j];
				if (vertices[indices[i].vtxk][j] > max) max = vertices[indices[i].vtxk][j];
			}
			m[j] = min; M[j] = max;
		}
		return BoundingBox(m,M);
	}

	// Build mesh BVH
	void build_bvh(BVH* bvh, int start, int end) {
		bvh->start = start;
		bvh->end = end;
		bvh->left = NULL;
		bvh->right = NULL;
		bvh->bbox = compute_bbox(start, end);
		if (end - start <= 4) return;

		Vector mM = bvh->bbox.M - bvh->bbox.m;
		int maxDim = getMaxDimension(mM);
		int pivot = start;
		double middle = mM[maxDim]/2 + bvh->bbox.m[maxDim];

		for (int i = start; i < end; i++) {
			double barycent = (vertices[indices[i].vtxi][maxDim] + vertices[indices[i].vtxj][maxDim] + vertices[indices[i].vtxk][maxDim])/3;
			if (barycent <= middle) { // if triangles are on the left part
				std::swap(indices[i], indices[pivot]);
				pivot++;
			}
		}
		if (pivot == start || pivot == end) return;

		bvh->left = new BVH();
		bvh->right = new BVH();
		build_bvh(bvh->left, start, pivot);
		build_bvh(bvh->right, pivot, end);
	}

	// Computes intersection with the triangles if the mesh
	double intersect(const Ray& r, Vector& P, Vector& bestN) const override {
		bool has_intersection = false;
		double min_dist = std::numeric_limits<double>::max();

		std::vector<BVH*> bvhs;
		bvhs.push_back(meshBVH);

		while (!bvhs.empty()) {
			const BVH* cur = bvhs.back();
			bvhs.pop_back();
			if (cur->left) {
				if (cur->left->bbox.intersect(r)) {
					bvhs.push_back(cur->left);
				}
				if (cur->right->bbox.intersect(r)) {
					bvhs.push_back(cur->right);
				}
			} else {
				for (int i = cur->start; i < cur->end; i++) {
					const Vector A = vertices[indices[i].vtxi];
					const Vector B = vertices[indices[i].vtxj];
					const Vector C = vertices[indices[i].vtxk];
					const Vector e1 = B - A;
					const Vector e2 = C - A;
					Vector AO = r.O - A;
					Vector AOcrossU = cross(AO, r.u);
					Vector N = cross(e1, e2);

					double invDet = 1./dot(r.u, N);

					double beta = -dot(e2,AOcrossU) * invDet;
					if (beta<0) continue;
					if (beta>1) continue;

					double gamma = dot(e1,AOcrossU) * invDet;
					if (gamma<0) continue;
					if (gamma>1) continue;

					double alpha = 1 - beta - gamma;
					if (alpha<0) continue;

					double t = -dot(AO, N) * invDet;
					if (t < 0) continue;
					if (t > min_dist) continue;

					min_dist = t;
					P = r.O + t * r.u;
					N.normalize();
					bestN = normals[indices[i].ni] * alpha + normals[indices[i].nj] * beta + normals[indices[i].nk] * gamma;
					has_intersection = true;
				}
			}
		}
		return has_intersection ? min_dist : -1;
	};

	void scale(double a, const Vector & b) {
		for (int i=0; i<vertices.size();i++) {
			vertices[i] = vertices[i]*a +b;
		}
	}

	// Computes the bounding box of the mesh
	void compute_bbox() {
		Vector m,M;
		for (int j=0;j<3;j++) {
			double min = vertices[0][j],max = vertices[0][j];
			for (int i=1;i<vertices.size();i++) {
				if (vertices[i][j]<min) min = vertices[i][j];
				if (vertices[i][j]>max) max = vertices[i][j];
			}
			m[j] = min; M[j] = max;
		}
		bbox = BoundingBox(m,M);
	}

	// Reads mesh object from file
    void readOBJ(const char* obj) {

        char matfile[255];
        char grp[255];

        FILE* f;
        f = fopen(obj, "r");
        int curGroup = -1;
        while (!feof(f)) {
            char line[255];
            if (!fgets(line, 255, f)) break;

            std::string linetrim(line);
            linetrim.erase(linetrim.find_last_not_of(" \r\t") + 1);
            strcpy(line, linetrim.c_str());

            if (line[0] == 'u' && line[1] == 's') {
                sscanf(line, "usemtl %[^\n]\n", grp);
                curGroup++;
            }

            if (line[0] == 'v' && line[1] == ' ') {
                Vector vec;

                Vector col;
                if (sscanf(line, "v %lf %lf %lf %lf %lf %lf\n", &vec[0], &vec[1], &vec[2], &col[0], &col[1], &col[2]) == 6) {
                    col[0] = std::min(1., std::max(0., col[0]));
                    col[1] = std::min(1., std::max(0., col[1]));
                    col[2] = std::min(1., std::max(0., col[2]));

                    vertices.push_back(vec);
                    vertexcolors.push_back(col);

                } else {
                    sscanf(line, "v %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
                    vertices.push_back(vec);
                }
            }
            if (line[0] == 'v' && line[1] == 'n') {
                Vector vec;
                sscanf(line, "vn %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
                normals.push_back(vec);
            }
            if (line[0] == 'v' && line[1] == 't') {
                Vector vec;
                sscanf(line, "vt %lf %lf\n", &vec[0], &vec[1]);
                uvs.push_back(vec);
            }
            if (line[0] == 'f') {
                TriangleIndices t;
                int i0, i1, i2, i3;
                int j0, j1, j2, j3;
                int k0, k1, k2, k3;
                int nn;
                t.group = curGroup;

                char* consumedline = line + 1;
                int offset;

                nn = sscanf(consumedline, "%u/%u/%u %u/%u/%u %u/%u/%u%n", &i0, &j0, &k0, &i1, &j1, &k1, &i2, &j2, &k2, &offset);
                if (nn == 9) {
                    if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                    if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                    if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                    if (j0 < 0) t.uvi = uvs.size() + j0; else   t.uvi = j0 - 1;
                    if (j1 < 0) t.uvj = uvs.size() + j1; else   t.uvj = j1 - 1;
                    if (j2 < 0) t.uvk = uvs.size() + j2; else   t.uvk = j2 - 1;
                    if (k0 < 0) t.ni = normals.size() + k0; else    t.ni = k0 - 1;
                    if (k1 < 0) t.nj = normals.size() + k1; else    t.nj = k1 - 1;
                    if (k2 < 0) t.nk = normals.size() + k2; else    t.nk = k2 - 1;
                    indices.push_back(t);
                } else {
                    nn = sscanf(consumedline, "%u/%u %u/%u %u/%u%n", &i0, &j0, &i1, &j1, &i2, &j2, &offset);
                    if (nn == 6) {
                        if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                        if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                        if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                        if (j0 < 0) t.uvi = uvs.size() + j0; else   t.uvi = j0 - 1;
                        if (j1 < 0) t.uvj = uvs.size() + j1; else   t.uvj = j1 - 1;
                        if (j2 < 0) t.uvk = uvs.size() + j2; else   t.uvk = j2 - 1;
                        indices.push_back(t);
                    } else {
                        nn = sscanf(consumedline, "%u %u %u%n", &i0, &i1, &i2, &offset);
                        if (nn == 3) {
                            if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                            if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                            if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                            indices.push_back(t);
                        } else {
                            nn = sscanf(consumedline, "%u//%u %u//%u %u//%u%n", &i0, &k0, &i1, &k1, &i2, &k2, &offset);
                            if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                            if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                            if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                            if (k0 < 0) t.ni = normals.size() + k0; else    t.ni = k0 - 1;
                            if (k1 < 0) t.nj = normals.size() + k1; else    t.nj = k1 - 1;
                            if (k2 < 0) t.nk = normals.size() + k2; else    t.nk = k2 - 1;
                            indices.push_back(t);
                        }
                    }
                }

                consumedline = consumedline + offset;

                while (true) {
                    if (consumedline[0] == '\n') break;
                    if (consumedline[0] == '\0') break;
                    nn = sscanf(consumedline, "%u/%u/%u%n", &i3, &j3, &k3, &offset);
                    TriangleIndices t2;
                    t2.group = curGroup;
                    if (nn == 3) {
                        if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                        if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                        if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                        if (j0 < 0) t2.uvi = uvs.size() + j0; else  t2.uvi = j0 - 1;
                        if (j2 < 0) t2.uvj = uvs.size() + j2; else  t2.uvj = j2 - 1;
                        if (j3 < 0) t2.uvk = uvs.size() + j3; else  t2.uvk = j3 - 1;
                        if (k0 < 0) t2.ni = normals.size() + k0; else   t2.ni = k0 - 1;
                        if (k2 < 0) t2.nj = normals.size() + k2; else   t2.nj = k2 - 1;
                        if (k3 < 0) t2.nk = normals.size() + k3; else   t2.nk = k3 - 1;
                        indices.push_back(t2);
                        consumedline = consumedline + offset;
                        i2 = i3;
                        j2 = j3;
                        k2 = k3;
                    } else {
                        nn = sscanf(consumedline, "%u/%u%n", &i3, &j3, &offset);
                        if (nn == 2) {
                            if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                            if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                            if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                            if (j0 < 0) t2.uvi = uvs.size() + j0; else  t2.uvi = j0 - 1;
                            if (j2 < 0) t2.uvj = uvs.size() + j2; else  t2.uvj = j2 - 1;
                            if (j3 < 0) t2.uvk = uvs.size() + j3; else  t2.uvk = j3 - 1;
                            consumedline = consumedline + offset;
                            i2 = i3;
                            j2 = j3;
                            indices.push_back(t2);
                        } else {
                            nn = sscanf(consumedline, "%u//%u%n", &i3, &k3, &offset);
                            if (nn == 2) {
                                if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                                if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                                if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                                if (k0 < 0) t2.ni = normals.size() + k0; else   t2.ni = k0 - 1;
                                if (k2 < 0) t2.nj = normals.size() + k2; else   t2.nj = k2 - 1;
                                if (k3 < 0) t2.nk = normals.size() + k3; else   t2.nk = k3 - 1;
                                consumedline = consumedline + offset;
                                i2 = i3;
                                k2 = k3;
                                indices.push_back(t2);
                            } else {
                                nn = sscanf(consumedline, "%u%n", &i3, &offset);
                                if (nn == 1) {
                                    if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                                    if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                                    if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                                    consumedline = consumedline + offset;
                                    i2 = i3;
                                    indices.push_back(t2);
                                } else {
                                    consumedline = consumedline + 1;
                                }
                            }
                        }
                    }
                }

            }

        }
        fclose(f);

    }

    std::vector<TriangleIndices> indices;
    std::vector<Vector> vertices;
    std::vector<Vector> normals;
    std::vector<Vector> uvs;
    std::vector<Vector> vertexcolors;
	BoundingBox bbox;
	BVH *meshBVH;

};

class Sphere : public Geometry {
public:
	explicit Sphere(const Vector& C, const double R, const Vector& Albedo, const bool Mirror = false, const bool isTransparent = false, const float refraction_index = 1) :
	Geometry(Albedo, Mirror, isTransparent), C(C), R(R), refraction_index(refraction_index) {};

	// Determines if a ray is intersected by an object and determine the position of the intersection
	double intersect(const Ray& r, Vector& P, Vector& N) const override {
		// solve ax^2 + bx + c = 0
		double b = 2 * dot(r.u, r.O - C);
		double c = (r.O-C).norm2() - R * R;
		double delta = b * b - 4 * c;
		if (delta < 0) {
			return -1;
		};
		double t1 = (-b - sqrt(delta)) / 2.;
		double t2 = (-b + sqrt(delta)) / 2.;
		if (t2 < 0) {
			return -1; // sphere is behind
		}
		double t;
		if (t1>0) {
			t = t1;
		} else {
			t = t2;
		}
		P = r.O + t * r.u;
		N = P - C;
		N.normalize();
		return t;
	}

	Vector C;
	double R;
	double refraction_index;
};

class Scene {
public:
	Scene() {}
	void addSphere(const Sphere *s) { objects.push_back(s);}
	void addMesh(const TriangleMesh *s) { objects.push_back(s);}

	// Determines the first object that intersect the ray
	double first_intersect(const Ray& r, Vector& P, Vector& N, Vector& Albedo, bool& Mirror, bool& isTransparent, int& index, double& R) {
		bool hasIntersection = false;
		Vector local_P, local_N;
		double min_dist = std::numeric_limits<double>::max();
		for (int i = 0; i < (int)(objects.size()); i++) {
			double dist = objects[i]->intersect(r, local_P, local_N);
			if (dist != -1 && dist < min_dist) {
				hasIntersection = true;
				min_dist = dist;
				Albedo = objects[i]->Albedo;
                Mirror = objects[i]->Mirror;
				index = i;
                isTransparent = objects[i]->isTransparent;
				P = local_P;
				N = local_N;
			}
		}
		if (hasIntersection) {
			return min_dist;
		}
		return -1;
	}

	// Computes direction vector of a randomly reflected ray on a surface
    Vector random_cos(Vector& N) {
      double u1 = uniform(engine);
      double u2 = uniform(engine);
      double s = sqrt(1 - u2);
      double x = cos(2. * M_PI * u1) * s;
      double y = sin(2. * M_PI * u1) * s;
      double z = sqrt(u2);
      Vector T;
		if(std::abs(N[0]) <= std::abs(N[1]) && std::abs(N[0]) <= std::abs(N[2])) {
			T = Vector(0,N[2],-N[1]);
		}
		if (std::abs(N[1]) <= std::abs(N[0]) && std::abs(N[1]) <= std::abs(N[2])) {
			T = Vector(-N[2], 0,N[0]);
		}
		if(std::abs(N[2]) <= std::abs(N[1]) && std::abs(N[2]) <= std::abs(N[0])) {
			T = Vector(-N[1],N[0],0);
		}
		T.normalize();
		Vector T2 = cross(T,N);
		return x * T + y * T2 + z * N;
    }

	// Computes the color to display for a pixel
	Vector getColor(const Ray& r, int nb_rebound, bool isIndirect = false) {
		Vector P, N;
		Vector Albedo;
        bool Mirror, isTransparent;
		double R;
		int index;
		if (nb_rebound <= 0) {
			return Vector(0, 0, 0);
		}
		double min_dist = first_intersect(r, P, N, Albedo, Mirror, isTransparent, index, R);
		if (min_dist != -1) {
			if (isIndirect && index == 0) {
				return Vector(0,0, 0);
			}
			if (index == 0) {
				// TODO: add a sphere attribute light = true/false
				return Vector(I, I, I)/ sqr(2*M_PI * R);
			}
			if (Mirror) {
                Vector R = r.u - 2 * dot(r.u, N)*N;
                Ray mirrorRay(P + 0.0001*N, R);
				return getColor(mirrorRay, nb_rebound - 1);
			}
			if (isTransparent) {
				double n1 = 1.;
				// TODO : affect refraction index to the sphere
				double n2 = 1.2;
				Vector Ntransp = N;
				if (dot(r.u, N) > 0) {
					std::swap(n1,n2);
					Ntransp = -N;
				}
				double k0 = sqr(n1 - n2) / sqr(n1 + n2);
				double alphaR = k0 + (1 - k0) * std::pow(1 - std::abs(dot(Ntransp, r.u)), 5.);
				double alphaT =  1 - alphaR;

				double d = 1 - sqr(n1/n2)*(1 - sqr(dot(r.u, Ntransp)));
				if (d<0) {
					return Vector(0, 0, 0);
				}

				double u = uniform(engine);
				Vector color;
				if (u<alphaT) {
					// Refraction
					Vector Tt = (n1/n2)*(r.u - dot(r.u, Ntransp)*Ntransp);
					Vector Tn = - sqrt(d) * Ntransp;
					Vector T = Tt + Tn;
					Ray transpRay(P - 0.0001*Ntransp, T);
					color = getColor(transpRay, nb_rebound-1);
				} else {
					// Reflection
					Vector R = r.u - 2 * dot(r.u, Ntransp)*Ntransp;
					Ray mirrorRay(P + 0.0001*Ntransp, R);
					color = getColor(mirrorRay, nb_rebound - 1);
				}

				return color;
			}
			Vector PL = L - P;
			PL.normalize();

			Vector PLinv = -PL;
			Vector Nprime = random_cos(PLinv); // ==shadowN ?
			Vector x = Nprime * dynamic_cast<const Sphere*>(objects[0])->R + dynamic_cast<const Sphere*>(objects[0])->C;
			Vector Px = x - P;
			double Px_norm2 = Px.norm2();
			Px.normalize();

			Ray shadowRay(P+0.0001*N,Px);
			Vector directLight, shadowColor, shadowN, shadowP;
			// If it is NOT a shadow, directlight is not 0
			if (first_intersect(shadowRay, shadowP, shadowN, shadowColor, Mirror, isTransparent, index, R) > sqrt(Px_norm2) - 0.1) {
				double px = std::max(1e-12, dot(-PL, Nprime))/(M_PI * dynamic_cast<const Sphere*>(objects[0])->R * dynamic_cast<const Sphere*>(objects[0])->R);
				directLight = (I * std::max(0., dot(N, Px)) * std::max(0., dot(-Px, Nprime)) * Albedo) / (Px_norm2 * M_PI * px * sqr(2 * M_PI * dynamic_cast<const Sphere*>(objects[0])->R));
			}

            // Indirect lighting
            Vector randomVector = random_cos(N);
			Ray indirectRay(P + 0.0001*N, randomVector);
			Vector indirectLight = getColor(indirectRay, nb_rebound - 1, true);

			return directLight + Albedo * indirectLight;
		}
		return Vector(0, 0, 0);
	}

	std::vector<const Geometry*> objects;

	Vector L;
	double I;
};


int main() {
	int W = 128;
	int H = 128;
	int num_simu = 2;
	int num_rebonds = 3;
	double alpha = 60 * 3.14159265358979323846 / 180;
	double focusd = 65;

	Scene scene;
	scene.L = Vector(20,30,-30);
	scene.I = 2e10;
	scene.addSphere(new Sphere( scene.L, 5, Vector(0,0,0), false, false)); // light
	scene.addSphere(new Sphere(Vector(0,1050,0), 1000, Vector(0.5,0.5,0.7)));
	scene.addSphere(new Sphere(Vector(0,0,1100), 1000, Vector(0.5,0.5,0.7)));
	scene.addSphere(new Sphere(Vector(0,-1010,0), 1000, Vector(0.5,0.7,0.5)));
	scene.addSphere(new Sphere(Vector(0,0,-1100), 1000, Vector(0.7,0.5,0.5)));
	scene.addSphere(new Sphere(Vector(1050,0,0), 1000, Vector(0.5,0.5,0.7)));
	scene.addSphere(new Sphere(Vector(-1050,0,0), 1000, Vector(0.5,0.5,0.7)));
	// scene.addSphere(new Sphere(Vector(-25,0,10),10, Vector(0.5,0.5,0.5), false, false, 1.5));
	// scene.addSphere(new Sphere(Vector(0,0,10),10, Vector(0.5,0.5,0.5), true, false, 1.5));
	// scene.addSphere(new Sphere(Vector(25,0,10),10, Vector(0.5,0.5,0.5), false, true, 1.5));
	// scene.addSphere(Sphere(Vector(0,-5,-20),5, Vector(0.5,0.0,0.0), false, false, 1.5));

	TriangleMesh *catMesh = new TriangleMesh(Vector(0.5,0.5,0.5), false, false);
	catMesh->readOBJ("Mesh/Models_F0202A090/cat.obj");
	catMesh->scale(0.75, Vector(0,-10,0));
	catMesh->meshBVH = new BVH();
	catMesh->build_bvh(catMesh->meshBVH,0, catMesh->indices.size());
	scene.addMesh(catMesh);

	Vector O(0,10,-55);

	std::vector<unsigned char> image(W*H * 3, 0);
#pragma omp parallel for
	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W; j++) {
			Vector color(0,0,0);
			for (int k = 1; k < num_simu; k++) {

				// anti-aliasing
				double u1 = uniform(engine);
				double u2 = uniform(engine);
				double rr = sqrt(-2 * log(u1));
				double n1 = rr * cos(2.0 * M_PI * u2) * 0.5 ;
				double n2 = rr * sin(2.0 * M_PI * u2) * 0.5;

				Vector u(j - W/2 + n1, -i + H/2 + n2, W/(2*tan(alpha/2)));
				u.normalize();

				// Depth of field
				double n1b = rr * cos(2.0 * M_PI * u2) * 0.5 ;
				double n2b = rr * sin(2.0 * M_PI * u2) * 0.5;

				Vector dest = O + focusd * u;
				Vector Oprime = O + Vector(n1b, n2b, 0);
				Vector uprime = dest - Oprime;
				uprime.normalize();
				Ray r(Oprime,uprime);

				color += scene.getColor(r, num_rebonds);
			}
			color = color / num_simu;

			// Assign the RGB values to the image
			// Apply gamma correction
			// Cap values over 255
			image[(i*W + j) * 3 + 0] = std::min(std::pow(color[0],1/2.2),255.);  // RED
			image[(i*W + j) * 3 + 1] = std::min(std::pow(color[1],1/2.2),255.);  // GREEN
			image[(i*W + j) * 3 + 2] = std::min(std::pow(color[2],1/2.2),255.);  // BLUE
		}
	}
	stbi_write_png("image.png", W, H, 3, &image[0], 0);

	return 0;
}