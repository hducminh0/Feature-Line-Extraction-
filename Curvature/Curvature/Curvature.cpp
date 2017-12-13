#include <iostream>
#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vcg/complex/complex.h>
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export_off.h>
#include <wrap/io_trimesh/import_ply.h>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/curvature.h>
#include <vcg/simplex/face/pos.h>

class MyEdge;
class MyFace;
class MyVertex;
struct MyUsedTypes : public vcg::UsedTypes<     vcg::Use<MyVertex>   ::AsVertexType,
	vcg::Use<MyEdge>     ::AsEdgeType,
	vcg::Use<MyFace>     ::AsFaceType> {};

class MyVertex : public vcg::Vertex<MyUsedTypes, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::VFAdj, vcg::vertex::CurvatureDirf, vcg::vertex::Curvaturef, vcg::vertex::BitFlags  > {};
class MyFace : public vcg::Face< MyUsedTypes, vcg::face::FFAdj, vcg::face::VFAdj, vcg::face::VertexRef, vcg::face::BitFlags > {};
class MyEdge : public vcg::Edge<MyUsedTypes> {};
class MyMesh : public vcg::tri::TriMesh< std::vector<MyVertex>, std::vector<MyFace>, std::vector<MyEdge>  > {};



vector<MyVertex> getNeighbors(MyVertex inputVertex) {
	vector<MyVertex> retVertices;
	vcg::face::VFIterator<MyFace> vfi(&inputVertex);
	for (; !vfi.End(); ++vfi)
	{
		MyFace* f = vfi.F();
		//cout << f->V(0)->P().X() << "|" << f->V(0)->P().Y() << "|" << f->V(0)->P().Z()<<endl;
		//cout << f->V(1)->P().X() << "|" << f->V(1)->P().Y() << "|" << f->V(1)->P().Z() << endl;
		//cout << f->V(2)->P().X() << "|" << f->V(2)->P().Y() << "|" << f->V(2)->P().Z() << endl;
		retVertices.push_back(*(f->V(1)));
	}
	return retVertices;
}

// find the circumcircle
MyVertex circumcircle(MyVertex A, MyVertex B, MyVertex C) {
	float a11, a12, a13, a21, a22, a23, a31, a32, a33, b1, b2, b3, x, y, z, d, dx, dy, dz;
	MyVertex O;

	// phuong trinh mat phang trung truc cua AB
	a11 = B.P().X() - A.P().X();
	a12 = B.P().Y() - A.P().Y();
	a13 = B.P().Z() - A.P().Z();
	b1 = -(a11*(-(A.P().X() + B.P().X()) / 2) + a12 * (-(A.P().Y() + B.P().Y()) / 2) + a13 * (-(A.P().Z() + B.P().Z()) / 2));

	// phuong trinh mat phang trung truc cua AC
	a21 = C.P().X() - A.P().X();
	a22 = C.P().Y() - A.P().Y();
	a23 = C.P().Z() - A.P().Z();
	b2 = -(a21*(-(A.P().X() + C.P().X()) / 2) + a22 * (-(A.P().Y() + C.P().Y()) / 2) + a23 * (-(A.P().Z() + C.P().Z()) / 2));

	// equation of ABC surface
	a31 = a12 * a23 - a22 * a13;
	a32 = a13 * a21 - a23 * a11;
	a33 = a11 * a22 - a21 * a12;
	b3 = -(a31 * (-A.P().X()) + a32 * (-A.P().Y()) + a33 * (-A.P().Z()));

	// solving system of equation with 3 variable
	d = a11 * a22*a33 + a12 * a23*a31 + a21 * a32*a13 - a13 * a22*a31 - a12 * a21*a33 - a11 * a32*a23;
	dx = b1 * a22*a33 + a12 * a23*b3 + b2 * a32*a13 - a13 * a22*b3 - a12 * b2*a33 - a23 * a32*b1;
	dy = a11 * b2*a33 + b1 * a23*a31 + a21 * b3*a13 - a13 * b2*a31 - b1 * a21*a33 - a23 * b3*a11;
	dz = a11 * a22*b3 + a12 * b2*a31 + a21 * a32*b1 - b1 * a22*a31 - a12 * a21*b3 - b2 * a32*a11;
	if (d == 0)
	{
		if ((dx == 0) && (dy == 0) && (dz == 0))
			printf("he vo so nghiem");
		else printf("he vo nghiem");
	}
	else
	{
		O.P().X() = dx / d;
		O.P().Y() = dy / d;
		O.P().Z() = dz / d;
	}
	return O;
}


MyVertex find_tangent_vector(MyVertex V, MyVertex Vi, MyVertex Vj, MyVertex O) {
	// V, Vi, Vj: three vertices in a neighborhood, V is the current point
	// O: origin of the circumcircle goes through V, Vi, Vj

	MyVertex V_Vi, V_Vj, V_O, V_temp, T;
	// V_Vi, V_Vj, V_O: vectors start from V and end at Vi, Vj, O respectively 
	// V_temp: result vector of V_Vi X V_Vj
	// T: tangent vector	

	float length = 0;    //length of the tangent vector for normaliztion

	V_Vi.P() = Vi.P() - V.P();
	V_Vj.P() = Vj.P() - V.P();
	V_O.P() = O.P() - V.P();

	// V_Vi X V_Vj
	V_temp.P().X() = V_Vi.P().Y() * V_Vj.P().Z() - V_Vi.P().Z() * V_Vj.P().Y();
	V_temp.P().Y() = V_Vi.P().Z() * V_Vj.P().X() - V_Vi.P().X() * V_Vj.P().Z();
	V_temp.P().Z() = V_Vi.P().X() * V_Vj.P().Y() - V_Vi.P().Y() * V_Vj.P().X();

	// V_temp X V_O
	T.P().X() = V_O.P().Y() * V_temp.P().Z() - V_O.P().Z() * V_temp.P().Y();
	T.P().Y() = V_O.P().Z() * V_temp.P().X() - V_O.P().X() * V_temp.P().Z();
	T.P().Z() = V_O.P().X() * V_temp.P().Y() - V_O.P().Y() * V_temp.P().X();

	// calculate the length of the vector
	length = sqrt(pow(T.P().X(), 2) + pow(T.P().Y(), 2) + pow(T.P().Z(), 2));

	//normalize the tangent vector
	T.P() = T.P() / length;
	//T.P().X() = T.P().X() / length;
	//T.P().Y() = T.P().Y() / length;
	//T.P().Z() = T.P().Z() / length;
	return T;
}


int main(int /*argc*/, char **/*argv*/)
{
	MyMesh m;
	//vcg::tri::io::ImporterOFF<MyMesh>::Open(m, "D:\\Laurana50k.off");
	vcg::tri::io::ImporterOFF<MyMesh>::Open(m, "F:\\MLP\\USTH\\ICTLab\\VCG\\TestObj\\Tetrahedron.off");

	//vcg::tri::Torus(m, 30, 10);
	//vcg::tri::Tetrahedron(m);

	vcg::tri::UpdateTopology<MyMesh>::FaceFace(m);
	vcg::tri::UpdateTopology<MyMesh>::VertexFace(m);
	vcg::tri::UpdateCurvature<MyMesh>::MeanAndGaussian(m);

	MyMesh::VertexIterator iterator;
	int count = 0;
	MyVertex O, T, V, Vi, Vj; 
	// O: origin of the circumcircle 
    // T: tangent vector
	// V: current vertex
	// Vi, Vj: neighbors of V
	for (iterator = m.vert.begin(); iterator != m.vert.end(); ++iterator)
	{
		V.P() = iterator->P();
		//cout << "Point:" << iterator->P().X() << " " << iterator->P().Y() << " " << iterator->P().Z() << "Has neighbors" << endl;

		vector<MyVertex> listNeighbors = getNeighbors(*iterator);
		Vi = listNeighbors.at(0);
		Vj = listNeighbors.at(1);
		// for (int i = 0; i < listNeighbors.size(); i++) {
		// 			MyVertex v = listNeighbors.at(i);
		// 			cout << i << ": " << v.P().X() << " " << v.P().Y() << " " << v.P().Z() << "--------" << endl;
		// 		}

		O = circumcircle(V, Vi, Vj);
		cout << O.P().X() << ", " << O.P().Y() << ", " << O.P().Z() << endl;

		T = find_tangent_vector(V, Vi, Vj, O); 
		cout << T.P().X() << ", " << T.P().Y() << ", " << T.P().Z() << endl;

		count++;
	}

	system("pause");
	return 0;
}