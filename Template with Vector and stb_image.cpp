#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include <iostream>

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

class Ray {
public:
	explicit Ray(const Vector& origin, const Vector& direction) : O(origin), u(direction) {};
	Vector O;
	Vector u;
};

class Sphere {
public:
	explicit Sphere(const Vector& C, const double R, const Vector& Albedo, const bool Mirror = false, const bool isTransparent = false, const float refraction_index = 1) :
	C(C), R(R), Albedo(Albedo), Mirror(Mirror), isTransparent(isTransparent), refraction_index(refraction_index) {};

	// Determines if a ray is intersected by an object and determine the position of the intersection
	double intersect(const Ray& r, Vector& P, Vector& N) {
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
	Vector Albedo;
    bool Mirror;
	bool isTransparent;
	double refraction_index;
};

class Scene {
public:
	Scene() {}
	void addSphere(const Sphere& s) { objects.push_back(s);}

	// Determines the first object that intersect the ray
	double first_intersect(const Ray& r, Vector& P, Vector& N, Vector& Albedo, bool& Mirror, bool& isTransparent, int& index, double& R) {
		bool hasIntersection = false;
		Vector local_P, local_N;
		double min_dist = std::numeric_limits<double>::max();
		for (int i = 0; i < (int)(objects.size()); i++) {
			double dist = objects[i].intersect(r, local_P, local_N);
			if (dist != -1 && dist < min_dist) {
				hasIntersection = true;
				min_dist = dist;
				Albedo = objects[i].Albedo;
                Mirror = objects[i].Mirror;
				index = i;
				R = objects[i].R;
                isTransparent = objects[i].isTransparent;
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
			Vector x = Nprime * objects[0].R + objects[0].C;
			Vector Px = x - P;
			double Px_norm2 = Px.norm2();
			Px.normalize();

			Ray shadowRay(P+0.0001*N,Px);
			Vector directLight, shadowColor, shadowN, shadowP;
			// If it is NOT a shadow, directlight is not 0
			if (first_intersect(shadowRay, shadowP, shadowN, shadowColor, Mirror, isTransparent, index, R) > sqrt(Px_norm2) - 0.1) {
				double px = std::max(1e-12, dot(-PL, Nprime))/(M_PI * objects[0].R * objects[0].R);
				directLight = (I * std::max(0., dot(N, Px)) * std::max(0., dot(-Px, Nprime)) * Albedo) / (Px_norm2 * M_PI * px * sqr(2 * M_PI * objects[0].R));
			}

            // Indirect lighting
            Vector randomVector = random_cos(N);
			Ray indirectRay(P + 0.0001*N, randomVector);
			Vector indirectLight = getColor(indirectRay, nb_rebound - 1, true);

			return directLight + Albedo * indirectLight;
		}
		return Vector(0, 0, 0);
	}

	std::vector<Sphere> objects;
	Vector L;
	double I;
};


int main() {
	int W = 1024;
	int H = 1024;
	int num_simu = 128;
	double alpha = 60 * 3.14159265358979323846 / 180;
	double focusd = 65;

	Scene scene;
	scene.L = Vector(20,30,20);
	scene.I = 2e10;
	scene.addSphere(Sphere( scene.L, 5, Vector(0,0,0), false, false)); // light
	scene.addSphere(Sphere(Vector(0,1050,0), 1000, Vector(0.5,0.5,0.7)));
	scene.addSphere(Sphere(Vector(0,0,1100), 1000, Vector(0.5,0.5,0.7)));
	scene.addSphere(Sphere(Vector(0,-1010,0), 1000, Vector(0.5,0.7,0.5)));
	scene.addSphere(Sphere(Vector(0,0,-1100), 1000, Vector(0.7,0.5,0.5)));
	scene.addSphere(Sphere(Vector(1050,0,0), 1000, Vector(0.5,0.5,0.7)));
	scene.addSphere(Sphere(Vector(-1050,0,0), 1000, Vector(0.5,0.5,0.7)));
	scene.addSphere(Sphere(Vector(-25,0,10),10, Vector(0.5,0.5,0.5), false, false, 1.5));
	scene.addSphere(Sphere(Vector(0,0,10),10, Vector(0.5,0.5,0.5), true, false, 1.5));
	scene.addSphere(Sphere(Vector(25,0,10),10, Vector(0.5,0.5,0.5), false, true, 1.5));
	// scene.addSphere(Sphere(Vector(0,-5,-20),5, Vector(0.5,0.0,0.0), false, false, 1.5));
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

				color += scene.getColor(r, 10);
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
	stbi_write_png("image_3spheres.png", W, H, 3, &image[0], 0);

	return 0;
}