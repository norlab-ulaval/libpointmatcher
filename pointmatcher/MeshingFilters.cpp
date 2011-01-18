// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab

#include "Core.h"

#include <math.h>

// Eigenvalues
#include <Eigen/Eigen>

// CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/centroid.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/Surface_mesh_simplification/HalfedgeGraph_Polyhedron_3.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_stop_predicate.h>

/**************************************************************************
* Mesh generation
**************************************************************************/

/** Meshing filter */

// Compute triangle centroid
template<typename T>
typename MetricSpaceAligner<T>::Vector3 MetricSpaceAligner<T>::MeshingFilter::computeCentroid(const Matrix3 matrixIn) const
{
	typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
	typedef K::Point_3 Point;
	typedef CGAL::Triangle_3<K> Triangle;

	// Create triangle	
	Triangle tri(Point(matrixIn(0, 0), matrixIn(1, 0), matrixIn(2, 0)),
				   Point(matrixIn(0, 1), matrixIn(1, 1), matrixIn(2, 1)),
				   Point(matrixIn(0, 2), matrixIn(1, 2), matrixIn(2, 2)));
	Point pointC = centroid(tri);

	Vector3 vc;
	vc(0) = pointC.x();
	vc(1) = pointC.y();
	vc(2) = pointC.z();

	return vc;
}

// Compute normal of plane in 3D
template<typename T>
typename MetricSpaceAligner<T>::Vector3 MetricSpaceAligner<T>::MeshingFilter::computeNormal(Matrix3 matrixIn) const
{
	Vector3 v1 = matrixIn.col(1) - matrixIn.col(0);
	Vector3 v2 = matrixIn.col(2) - matrixIn.col(0);
	Vector3 vn = v1.cross(v2);

	return vn.normalized();
}


/** ITMLocalMeshingFilter
  Generate local (i.e. sensor-centric) irregular triangular mesh (ITM) */
 
// Constructor
template<typename T>
MetricSpaceAligner<T>::ITMLocalMeshingFilter::ITMLocalMeshingFilter() {}

// Conversions
template<typename T>
typename MetricSpaceAligner<T>::Matrix MetricSpaceAligner<T>::ITMLocalMeshingFilter::cart2Spheric(const Matrix matrixIn) const
{
	// Generate matrix
	Matrix matrixOut(3, matrixIn.cols());

	// Compute r
	matrixOut.row(0) = matrixIn.colwise().norm();

	for (int i = 0; i < matrixIn.cols(); i++)
	{	
		// Compute theta
		matrixOut(1, i) = asin(matrixIn(2, i) / matrixOut(0, i));
						//acos(matrixIn(2, i) / matrixOut(0, i));

		// Compute phi
		matrixOut(2, i) = atan2(matrixIn(1, i), matrixIn(0, i));
						//acos(matrixIn(0, i) / (matrixOut(0, i)*cos(matrixOut(1, i))));
	}

	return matrixOut;
}

// 2D Delaunay triangulation
template<typename T>
typename MetricSpaceAligner<T>::Matrix MetricSpaceAligner<T>::ITMLocalMeshingFilter::delaunay2D(const Matrix matrixIn) const
{
	typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
	typedef CGAL::Triangulation_vertex_base_with_info_2<int, K> Vb;
	typedef CGAL::Triangulation_face_base_2<K> Fb;
	typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
	typedef CGAL::Delaunay_triangulation_2<K, Tds> DelaunayTri;

	typedef DelaunayTri::Finite_faces_iterator Finite_faces_iterator;
	typedef DelaunayTri::Vertex_handle Vertex_handle;
	typedef DelaunayTri::Point Point;

	// Compute triangulation
	DelaunayTri dt;
  	for (int i = 0; i < matrixIn.cols(); i++)
    {
		// Add point
		Point p = Point(matrixIn(0, i), matrixIn(1, i));
		dt.push_back(Point(p));
		// Add index
		Vertex_handle vh = dt.push_back(p);
		vh->info() = i;
	}

	// Iterate through faces
	Matrix matrixOut(3, dt.number_of_faces());
	int itCol = 0;
	DelaunayTri::Finite_faces_iterator it;
	for (it = dt.finite_faces_begin(); it != dt.finite_faces_end(); it++)
	{
		for (int k = 0; k < matrixOut.rows(); k++)
		{
			// Add index of triangle vertex to list
			matrixOut(k, itCol) = it->vertex(k)->info();
		}
		itCol++;
	}

	return matrixOut;
}

// Generate ITM
template<typename T>
void MetricSpaceAligner<T>::ITMLocalMeshingFilter::generateTriMesh(const Matrix matrixFeatures, const Matrix matrixIndices, 
	Matrix & matrixNewFeatures, Matrix & matrixNewDescriptors) const
{
	// Initialization
	Matrix3 vp;
	Vector3 vc;
	Vector3 vn;

	for (int i = 0; i < matrixIndices.cols(); i++)
	{
		// Get triangle
		vp.col(0) = matrixFeatures.col(int(matrixIndices(0, i)));
		vp.col(1) = matrixFeatures.col(int(matrixIndices(1, i)));
		vp.col(2) = matrixFeatures.col(int(matrixIndices(2, i)));

		// Compute triangle centroid
		vc = computeCentroid(vp);

		// Compute normal at centroid
		vn = computeNormal(vp);

		// Generate data structure
		matrixNewFeatures.col(i) = vc;
		matrixNewDescriptors.block(0, i, 3, 1) = vn;
		matrixNewDescriptors.block(3, i, 3, 1) = vp.col(0);
		matrixNewDescriptors.block(6, i, 3, 1) = vp.col(1);
		matrixNewDescriptors.block(9, i, 3, 1) = vp.col(2);
	}
	matrixNewDescriptors.block(12, 0, 3, matrixNewDescriptors.cols()) = matrixIndices;
}

// Prefilter
template<typename T>
typename MetricSpaceAligner<T>::DataPoints MetricSpaceAligner<T>::ITMLocalMeshingFilter::preFilter(
	const DataPoints& input, bool& iterate) const
{
	// Get input features
	Matrix mFeatures = input.features.block(0, 0, 3, input.features.cols());

	// Convert into spherical coordinates
	Matrix mSpheriCoord = cart2Spheric(mFeatures);

	// 2D Delaunay triangulation
	Matrix mTriIndexList = delaunay2D(mSpheriCoord.block(1, 0, 2, mSpheriCoord.cols()));

	// Generate ITM
	Matrix mNewFeatures(3, mTriIndexList.cols());
	Matrix mNewDescriptors(15, mTriIndexList.cols());
	generateTriMesh(mFeatures, mTriIndexList, mNewFeatures, mNewDescriptors);

	return DataPoints(mNewFeatures, input.featureLabels, mNewDescriptors, input.descriptorLabels);
}

template struct MetricSpaceAligner<float>::ITMLocalMeshingFilter;
template struct MetricSpaceAligner<double>::ITMLocalMeshingFilter;


/** ITMGlobalMeshingFilter
  Generate global irregular triangular mesh (ITM) */


/** MarchingCubeMeshingFilter
  Generate global surface mesh by Marching Cubes */


/**************************************************************************
* Mesh processing
**************************************************************************/

/** ArtifactsRemovalMeshingFilter */

// Constructor
template<typename T>
MetricSpaceAligner<T>::ArtifactsRemovalMeshingFilter::ArtifactsRemovalMeshingFilter(
	const T thresh1, const T thresh2, const T thresh3):
	thresh1(thresh1), thresh2(thresh2), thresh3(thresh3){}

// Prefilter 
template<typename T>
typename MetricSpaceAligner<T>::DataPoints MetricSpaceAligner<T>::ArtifactsRemovalMeshingFilter::preFilter(
	const DataPoints& input, bool &iterate) const
{
	assert((input.descriptors.rows() >= 15) && (input.features.cols() == input.descriptors.cols()));

	// Initialization
	int nbCols, nbTriangles, j;
	typename DataPoints::Features featIn;
	typename DataPoints::Descriptors descIn;
	typename DataPoints::Features featOut;
	typename DataPoints::Descriptors descOut;
	
	featIn = input.features;
	descIn = input.descriptors;
	int nbFeatRows = featIn.rows();
	int nbDescRows = descIn.rows();
	featOut = featIn;
	descOut = descIn;

	/* Filter 1
	*	for every triangle in the mesh, apply threshold on the ratio
	*	between its closest and farthest vertices relative to the sensor origin
	*	-> remove shadow triangles
	*/
	if (thresh1 > 0)
	{
	  	nbCols = descIn.cols();

		// Compute distances
		Matrix mDist(3, nbCols);
		mDist.row(0) = descIn.block(3, 0, 3, nbCols).colwise().norm();
		mDist.row(1) = descIn.block(6, 0, 3, nbCols).colwise().norm();
		mDist.row(2) = descIn.block(9, 0, 3, nbCols).colwise().norm();

		Matrix mRatio(1, nbCols);
		mRatio = mDist.colwise().maxCoeff();
		mRatio = mRatio.cwise() / mDist.colwise().minCoeff();
		nbTriangles = (mRatio.cwise() < thresh1).count();

		// Filtering
		featOut = typename DataPoints::Features(nbFeatRows, nbTriangles);
		descOut = typename DataPoints::Descriptors(nbDescRows, nbTriangles);

		j = 0;
		for (int i = 0; i < nbCols; i++)
		{
			if (mRatio(i) < thresh1)
			{
				featOut.col(j) = featIn.col(i);
				descOut.col(j) = descIn.col(i);
				j++;
			}
		}

		featIn = featOut;
		descIn = descOut;
	}

	/* Filter 2
	*	for every triangle in the mesh, apply threshold on triangle's perimeter
	*	-> remove frontier triangles
	*/
	
	if (thresh2 > 0 && j > 0)
	{
		nbCols = descIn.cols();

		// Compute perimeters
		Matrix mSideL(3, nbCols);
		mSideL.row(0) = (descIn.block(3, 0, 3, nbCols) - descIn.block(6, 0, 3, nbCols)).colwise().norm();
		mSideL.row(1) = (descIn.block(3, 0, 3, nbCols) - descIn.block(9, 0, 3, nbCols)).colwise().norm();
		mSideL.row(2) = (descIn.block(6, 0, 3, nbCols) - descIn.block(9, 0, 3, nbCols)).colwise().norm();
		
		Matrix mPerim(1, nbCols);
		mPerim = mSideL.colwise().sum();
		nbTriangles = (mPerim.cwise() < thresh2).count();

		// Filtering
		featOut = typename DataPoints::Features(nbFeatRows, nbTriangles);
		descOut = typename DataPoints::Descriptors(nbDescRows, nbTriangles);

		j = 0;
		for (int i = 0; i < nbCols; i++)
		{
			if (mPerim(i) < thresh2)
			{
				featOut.col(j) = featIn.col(i);
				descOut.col(j) = descIn.col(i);
				j++;
			}
		}

		featIn = featOut;
		descIn = descOut;
	}

	/* Filter 3
	*	for every triangle in the mesh, apply threshold on the triangles
	*	with small incident angle relative to the sensor's line of sight
	* 	-> remove remaining shadow triangles that escaped Filter 1
	*/
	if (thresh3 > 0 && j > 0)
	{
		nbCols = descIn.cols();

		// Compute incident angle
		Matrix mUnitVec(3, nbCols);
		Matrix mVecNorm = featIn.colwise().norm();
		mUnitVec.row(0) = featIn.row(0).cwise() / mVecNorm;
		mUnitVec.row(1) = featIn.row(1).cwise() / mVecNorm;
		mUnitVec.row(2) = featIn.row(2).cwise() / mVecNorm;

		Matrix mIncAngle(1, nbCols);
		Matrix mUnitNormal = descIn.block(0, 0, 3, nbCols);
		Matrix mComp = (mUnitNormal.cwise() * mUnitVec).cwise().abs();
		mIncAngle = mComp.colwise().sum();
		nbTriangles = (mIncAngle.cwise() > thresh3).count();

		// Filtering
		featOut = typename DataPoints::Features(nbFeatRows, nbTriangles);
		descOut = typename DataPoints::Descriptors(nbDescRows, nbTriangles);

		j = 0;
		for (int i = 0; i < nbCols; i++)
		{
			if (mIncAngle(i) > thresh3)
			{
				featOut.col(j) = featIn.col(i);
				descOut.col(j) = descIn.col(i);
				j++;
			}
		}

		featIn = featOut;
		descIn = descOut;
	}

	return DataPoints(featOut, input.featureLabels, descOut, input.descriptorLabels);
}

template struct MetricSpaceAligner<float>::ArtifactsRemovalMeshingFilter;
template struct MetricSpaceAligner<double>::ArtifactsRemovalMeshingFilter;


/** SimplifyMeshingFilter */

// Modifier creating a surface mesh
typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_3 Point;
typedef CGAL::Polyhedron_3<K, CGAL::Polyhedron_items_with_id_3> SurfMesh;
typedef SurfMesh::HalfedgeDS HalfedgeDS;
typedef CGAL::Polyhedron_incremental_builder_3<HalfedgeDS> B;

template <typename TT>
class Build_SM : public CGAL::Modifier_base<HalfedgeDS>
{
	typedef typename Eigen::Matrix<TT, Eigen::Dynamic, Eigen::Dynamic> Matrix;

private:
	int nbV, nbF;
	Matrix mSMPoints;

public:
	Build_SM(Matrix matrixIn)
	{
		mSMPoints = matrixIn;
		nbF = mSMPoints.cols();
		nbV = 3*nbF;//int(mSMPoints.block(12, 0, 3, nbF).maxCoeff() + 1); // upper bound on # vertices
	};
	void operator() (HalfedgeDS & hds)
	{	
		B ib(hds, true);
		ib.begin_surface(nbV, nbF);

		int* arrIndRemap = new int[nbV];
		for (int j = 0; j < nbV; j++)
		{
			arrIndRemap[j] = -1;
		}

		int newIndex, oldIndex, triIndex;
		triIndex = 0;
		for (int i = 0; i < nbF; i++)
		{
			Point p1(mSMPoints(3, i), mSMPoints(4, i), mSMPoints(5, i));
			Point p2(mSMPoints(6, i), mSMPoints(7, i), mSMPoints(8, i));
			Point p3(mSMPoints(9, i), mSMPoints(10, i), mSMPoints(11, i));
			ib.add_vertex(p1);
			ib.add_vertex(p2);
			ib.add_vertex(p3);

			// Rearrange indices
			ib.begin_facet();
			for (int k = 12; k < 15; k++)
			{
				oldIndex = int(mSMPoints(k, i));

				if (arrIndRemap[oldIndex] == -1)
				{
					newIndex = triIndex;
					arrIndRemap[oldIndex] = newIndex;
					triIndex++;
				}
				else
				{
					newIndex = arrIndRemap[oldIndex];
				}

				if (i < 2)
					std::cout << "Remapping: " << oldIndex << " -> " << newIndex << "\n";
				ib.add_vertex_to_facet(newIndex);
				ib.vertex(newIndex)->id() = newIndex;
			}
			ib.end_facet();
		}
		ib.end_surface();
		delete [] arrIndRemap;

		std::cout << triIndex << " " << nbV << "\n";
	}
};
	
// Constructor
template<typename T>
MetricSpaceAligner<T>::SimplifyMeshingFilter::SimplifyMeshingFilter(
	const int edgeCount) : edgeCount(edgeCount) {}

// Prefilter 
template<typename T>
typename MetricSpaceAligner<T>::DataPoints MetricSpaceAligner<T>::SimplifyMeshingFilter::preFilter(
    const DataPoints& input, bool &iterate) const
{
	assert((input.descriptors.rows() >= 15) && (input.features.cols() == input.descriptors.cols()));

	typedef CGAL::Simple_cartesian<double> K;
	typedef K::Point_3 Point;
	typedef CGAL::Polyhedron_3<K, CGAL::Polyhedron_items_with_id_3> SurfMesh;
	typedef SurfMesh::Facet_iterator FacetIt;
	typedef SurfMesh::Halfedge::Halfedge_handle HalfedgeIt;
	typedef SurfMesh::HalfedgeDS HalfedgeDS;
	typedef CGAL::Polyhedron_incremental_builder_3<HalfedgeDS> B;
	namespace SMS = CGAL::Surface_mesh_simplification;

	typename DataPoints::Features featIn;
	typename DataPoints::Descriptors descIn;
	typename DataPoints::Features featOut;
	typename DataPoints::Descriptors descOut;

	featIn = input.features;
	descIn = input.descriptors;

	// Create surface mesh
	SurfMesh sm;
	Build_SM<T> triMesh(descIn);
	sm.delegate(triMesh);

std::cout << sm.size_of_facets() << " Done1\n";

	// Simplify polyhedral mesh
	SMS::Count_stop_predicate<SurfMesh> stop(edgeCount); // stop predicate: stop when number of edges < edgeCount
	int nbEdgesRem;

    std::cout << sm.is_pure_triangle() << " triangle surface mesh.\n";
	std::cout << sm.size_of_facets() << " triangles.\n";


//	nbEdgesRem = SMS::edge_collapse(sm, stop, CGAL::edge_index_map(boost::get(CGAL::edge_external_index, sm)));

std::cout << edgeCount << " " << nbEdgesRem << " Done2\n";

	// Get result
	FacetIt itF; // iterate through facets
	HalfedgeIt itH;
	int j = 0;
	int nbFacets = sm.size_of_facets();
	featOut = typename DataPoints::Features(input.features.rows(), nbFacets);
	descOut = typename DataPoints::Descriptors(input.descriptors.rows(), nbFacets);
	
	Matrix3 vp;
	Vector3 vc;
	Vector3 vn;

	for (itF = sm.facets_begin(); itF != sm.facets_end(); itF++)
	{
		// Access vertices
		itH = itF->halfedge();
		Point p1 = itH->vertex()->point();
		int id1 = itH->vertex()->id();
		Point p2 = itH->next()->vertex()->point();
		int id2 = itH->next()->vertex()->id();
		Point p3 = itH->prev()->vertex()->point();
		int id3 = itH->prev()->vertex()->id();
		vp << p1.x(), p2.x(), p3.x(),
			  p1.y(), p2.y(), p3.y(),
			  p1.z(), p2.z(), p3.z();

		// Compute triangle centroid
		vc = computeCentroid(vp);

		// Compute normal at centroid
		vn = computeNormal(vp);

		featOut.col(j) = vc; 
		descOut.block(0, j, 3, 1) = vn;
		descOut.block(3, j, 3, 1) = vp.col(0);
		descOut.block(6, j, 3, 1) = vp.col(1);
		descOut.block(9, j, 3, 1) = vp.col(2);
		descOut.block(12, j, 3, 1) << id1, id2, id3;
		j++;
	}

std::cout << vp << "\n" << j << " Done3\n";

	return DataPoints(featOut, input.featureLabels, descOut, input.descriptorLabels);
}


template struct MetricSpaceAligner<float>::SimplifyMeshingFilter;
template struct MetricSpaceAligner<double>::SimplifyMeshingFilter;

