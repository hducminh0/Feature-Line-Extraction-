#include <iostream>
#include<vcg/complex/complex.h>
#include<wrap/io_trimesh/import.h>
//#include<wrap/io_trimesh/import_off.h>
#include<wrap/io_trimesh/export_off.h>
#include<wrap/io_trimesh/import_ply.h>
#include<vcg/complex/algorithms/update/topology.h>
#include<vcg/complex/algorithms/update/normal.h>
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

vector<MyVertex> getNeighbors(MyVertex v) {
	vcg::face::VFIterator<MyFace> vfi(&v); //initialize the iterator the first face
	for (; !vfi.End(); ++vfi)
	{
		MyFace* f = vfi.F();
		for (int k = 0; k<(*f).VN(); k++)
		{
			MyVertex* v = (*f).V(k);
			cout << v->P().X() << "," << v->P().Y() << "," << v->P().Z() << "," << endl;
		}
	}
	return vector<MyVertex>();
}

void OneRingNeighborhood(MyFace * f)
{
	MyVertex * v = f->V(0);
	MyFace* start = f;
	vcg::face::Pos<MyFace> p(f, 0, v);// constructor that takes face, edge and vertex

	int count = 0;
	do
	{
		p.FlipF();
		p.FlipE();

		cout << "flip" << count++ << endl;
		vcg::face::Pos<MyFace> f = p;
		f.FlipV();
		cout << f.v->P().X() << " " << f.v->P().Y() << " " << f.v->P().Z() << endl;
		cout << p.v->P().X() << " " << p.v->P().Y() << " " << p.v->P().Z() << endl;
		cout << v->K1() << "; " << v->K2() << endl;

	} while (p.f != start);
}

int main(int /*argc*/, char **/*argv*/)
{
	MyMesh m;
	vcg::tri::io::ImporterOFF<MyMesh>::Open(m, "C:\\Users\\XPS 15-9550\\Downloads\\Laurana50k.off");
	//vcg::tri::Torus(m, 30, 10);

	vcg::tri::UpdateTopology<MyMesh>::FaceFace(m);
	vcg::tri::UpdateTopology<MyMesh>::VertexFace(m);


	MyMesh::VertexIterator vi;
	int count = 0;
	vcg::tri::UpdateCurvature<MyMesh>::MeanAndGaussian(m);
	for (vi = m.vert.begin(); vi != m.vert.end(); ++vi)
	{
		cout << vi->P().X() << " " << vi->P().Y() << " " << vi->P().Z() << "--------" << endl;
		//getNeighbors(*vi);
		if(vi == m.vert.end())
			break;
		MyFace * f = vi->VFp();
		MyVertex * v = f->V(0);
		OneRingNeighborhood(f);
		//vcg::face::Pos<MyFace> p(start, 0, vi);// constructor that takes face, edge and vertex

		//cout << vcg::tri::UpdateCurvature<MyMesh>::ComputeSingleVertexCurvature(v) << endl;
		//MyMesh::EdgePointer ep = vi->VEp();
		//cout <<"1:"<< ep->V(0)->P().X() << " " << ep->V(0)->P().Y() << " " << ep->V(0)->P().Z()<<endl;
		//cout << "2:" << ep->V(1)->P().X() << " " << ep->V(0)->P().Y() << " " << ep->V(0)->P().Z() << endl;
		//break;
		count++;
	}
	// Two different techniques for computing Discrete Gaussian and Mean Curvature
	// they require the presence of the vertex::Curvature component
	//vcg::tri::UpdateCurvature<MyMesh>::PerVertex(m);

	// Two different techniques for computing Principal Curvature Directions 
	// they require the presence of the vertex::CurvatureDir component
	vcg::tri::UpdateCurvature<MyMesh>::PrincipalDirections(m);
	//vcg::tri::UpdateCurvature<MyMesh>::PrincipalDirectionsNormalCycle(m);
	printf("Input mesh  vn:%i fn:%i\n", m.VN(), m.FN());

	system("pause");
	return 0;
}