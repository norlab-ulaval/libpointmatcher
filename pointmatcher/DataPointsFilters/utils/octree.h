// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2018,
François Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
You can contact the authors at <f dot pomerleau at gmail dot com> and
<stephane at magnenat dot net>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#pragma once

#include <cstdlib> 
#include <vector>
#include "PointMatcher.h"

/*!
 * \class octree.h
 * \brief Octree class for DataPoints spatial representation
 *
 * \author Mathieu Labussiere (<mathieu dot labu at gmail dot com>)
 * \date 24/05/2018
 * \version 0.1
 *
 * Octree implementation for decomposing point cloud. 
 * The current implementation use the data structure PointMatcher<T>::DataPoints. 
 * It ensures that each node has either 8 or 0 childs. 
 *
 * Can create an octree with the 2 following crieterions:
 *	- max number of data by node
 *	- max size of a node (or stop when only one or zero data element is available)
 *
 * After construction, we can apply a process by creating a Visitor, implementing the following method:
 *	```cpp
 *	template<typename T>
 *	struct Visitor {
 *		bool operator()(Octree<T>& oc);
 *	};
 *	```
 *
 * Remark:
 *	- Current implementation only store the indexes of the points from the pointcloud.
 *	- Data element are exclusively contained in leaves node.
 *	- Some leaves node contains no data (to ensure 8 or 0 childs).
 *
 */
template < typename T >
class Octree 
{
public:
	using PM = PointMatcher<T>;
	using DP = typename PM::DataPoints; /**/
	using Id = typename DP::Index; /**/
	
	using Data = typename DP::Index; /**/
	using DataContainer = std::vector<Data>;

private:
	struct XYZ 
	{
		T x;
		T y;
		T z;
		
		XYZ(T x_=T(0.0), T y_=T(0.0), T z_=T(0.0)) : x{x_}, y{y_}, z{z_} {}
	
		void operator+=(const XYZ& v){ x+=v.x; y+=v.y; z+=v.z; }
		void operator-=(const XYZ& v){ x-=v.x; y-=v.y; z-=v.z; }
		void operator*=(const XYZ& v){ x*=v.x; y*=v.y; z*=v.z; }
		void operator*=(const T& s){ x*=s; y*=s; z*=s; }
		void operator/=(const T& s){ x/=s; y/=s; z/=s; }
		
		XYZ operator+(const XYZ& v2){return XYZ{x+v2.x,y+v2.y,z+v2.z};}
		XYZ operator-(const XYZ& v2){return XYZ{x-v2.x,y-v2.y,z-v2.z};}
		XYZ operator*(const XYZ& v2){return XYZ{x*v2.x,y*v2.y,z*v2.z};}
		XYZ operator*(const T& s)   {return XYZ{s*x,s*y,s*z};}
		XYZ operator/(const XYZ& v2){return XYZ{x/v2.x,y/v2.y,z/v2.z};}
		XYZ operator/(const T& s)   {return XYZ{s/x,s/y,s/z};}
	};
	
	struct BoundingBox 
	{
			XYZ center;
			T 	radius;
	};
	
	Octree* parent;
	Octree* octants[8];
	
	/******************************************************
	 *	Octants id are assigned as their position 
	 *   on a hamming cube (+ greater than center, - lower than center)
	 *
	 *	  	0	1	2	3	4	5	6	7
	 * 	x:	-	+	-	+	-	+	-	+
	 * 	z:	-	-	+	+	-	-	+	+
	 * 	y:	-	-	-	-	+	+	+	+
	 *
	 *****************************************************/
	
	BoundingBox bb;
	
	DataContainer data;	
	
	std::size_t depth;
	bool parallel_build;
	
public:
	Octree();
	Octree(const Octree<T>& o); //Deep-copy
	Octree(Octree<T>&&o);
	
	virtual ~Octree();
	
	Octree<T>& operator=(const Octree<T>&o); //Deep-copy
	Octree<T>& operator=(Octree<T>&& o);
	
	inline bool isLeaf() const;
	inline bool isRoot() const;
	inline bool isEmpty()const;
	
	inline void setParallel(bool enable);
	
	inline std::size_t idx(const XYZ& xyz) const;
	inline std::size_t idx(T x, T y, T z) const;
	
	inline std::size_t getDepth() const;
	DataContainer * getData();
	Octree<T>* operator[](std::size_t idx);
	
	// Build tree from DataPoints with a specified number of points by node
	bool build(const DP& pts, std::size_t maxDataByNode);
	bool build_par(const DP& pts, DataContainer&& datas, BoundingBox && bb, std::size_t maxDataByNode);
	bool build(const DP& pts, DataContainer&& datas, BoundingBox && bb, std::size_t maxDataByNode);
	
	// Build tree from DataPoints with a specified max size by node
	bool build(const DP& pts, T maxSizeByNode);
	bool build_par(const DP& pts, DataContainer&& datas, BoundingBox && bb, T maxSizeByNode);
	bool build(const DP& pts, DataContainer&& datas, BoundingBox && bb, T maxSizeByNode);
	
	template < typename Callback >
	bool visit(Callback& cb);
};
