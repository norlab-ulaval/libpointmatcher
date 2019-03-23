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
#include "octree.h"

#include <iterator>
#include <future>
#include <ciso646>

template<typename T, std::size_t dim>
Octree_<T,dim>::Octree_(): parent{nullptr}, depth{0}
{
	for(size_t i = 0; i < nbCells; ++i)
	{
		cells[i] = nullptr;
	}
}

template<typename T, std::size_t dim>
Octree_<T,dim>::Octree_(const Octree_<T,dim>& o): 
	bb{o.bb.center, o.bb.radius}, depth{o.depth}
{
	if(!o.parent)
	{
		parent = nullptr;
	}
	
	if(o.isLeaf())
	{
		for(size_t i = 0; i < nbCells; ++i)
		{
			cells[i] = nullptr;
		}
		
		data.insert(data.end(), o.data.begin(), o.data.end());
	}
	else
	{
		//Create each child recursively
		for(size_t i = 0; i < nbCells;++i)
		{
			cells[i] = new Octree_<T,dim>(*(o.cells[i]));
			cells[i]->parent = this;
		}
	}
}

template<typename T, std::size_t dim>
Octree_<T,dim>::Octree_(Octree_<T,dim>&& o):
	parent{nullptr}, bb{o.bb.center, o.bb.radius}, depth{o.depth}
{
	//only allow move of root node
	assert(o.isRoot());
	
	if(o.isLeaf())
	{
		data.insert(data.end(),
			std::make_move_iterator(o.data.begin()),
			std::make_move_iterator(o.data.end()));
	}
	
	//copy child ptr
	for(size_t i = 0; i < nbCells; ++i)
	{
		cells[i] = o.cells[i];
		o.cells[i] = nullptr;
	}
}

template<typename T, std::size_t dim>
Octree_<T,dim>::~Octree_()
{
	//delete recursively children
	if(!isLeaf())
	{
		for(size_t i = 0; i < nbCells; ++i)
		{
			delete cells[i];
		}
	}
}

template<typename T, std::size_t dim>
Octree_<T,dim>& Octree_<T,dim>::operator=(const Octree_<T,dim>& o)
{
	if(!o.parent)
	{
		parent = nullptr;
	}
	
	depth = o.depth;
	bb.center = o.bb.center;
	bb.radius = o.bb.radius;
	
	if(o.isLeaf())
	{
		for(size_t i = 0; i < nbCells; ++i)
		{
			cells[i] = nullptr;
		}
		
  		data.insert(data.end(), o.data.begin(), o.data.end());
	}
	else
	{
		//Create each child recursively
  		for(size_t i = 0; i < nbCells; ++i)
  		{
  			cells[i] = new Octree_<T,dim>(*(o.cells[i]));
  			cells[i]->parent = this;
  		}
	}
	return *this;
}

template<typename T, std::size_t dim>
Octree_<T,dim>& Octree_<T,dim>::operator=(Octree_<T,dim>&& o)
{
	//only allow move of root node
	assert(o.isRoot());
	
	parent = nullptr;
	bb.center = o.bb.center;
	bb.radius = o.bb.radius;
	
	depth = o.depth;
	
	if(o.isLeaf())
	{
		data.insert(data.end(),
			std::make_move_iterator(o.data.begin()), 
			std::make_move_iterator(o.data.end()));
	}
	
	//copy child ptrs
	for(size_t i = 0; i < nbCells; ++i)
	{
		cells[i] = o.cells[i];
		o.cells[i] = nullptr;
	}
	
	return *this;
}

template<typename T, std::size_t dim>
bool Octree_<T,dim>::isLeaf() const
{
	return cells[0] == nullptr;
}

template<typename T, std::size_t dim>
bool Octree_<T,dim>::isRoot() const
{
	return parent == nullptr;
}

template<typename T, std::size_t dim>
bool Octree_<T,dim>::isEmpty() const
{
	return data.size() == 0;
}

template<typename T, std::size_t dim>
size_t Octree_<T,dim>::childIdx(const Point& pt) const
{
	size_t id = 0;
	
	for(size_t i = 0; i < dim; ++i)
	{
		id |= ((pt(i) > bb.center(i)) << i);
	}
	
	return id;
}

template<typename T, std::size_t dim>
size_t Octree_<T,dim>::childIdx(const DP& pts, const Data d) const
{
	return childIdx(pts.features.col(d).head(dim));
}

template<typename T, std::size_t dim>
size_t Octree_<T,dim>::getDepth() const
{
	return depth;
}

template<typename T, std::size_t dim>
T Octree_<T,dim>::getRadius() const
{
	return bb.radius;
}

template<typename T, std::size_t dim>
typename Octree_<T,dim>::Point Octree_<T,dim>::getCenter() const
{
	return bb.center;
}

template<typename T, std::size_t dim>
typename Octree_<T,dim>::DataContainer* Octree_<T,dim>::getData()
{
	return &data;
}

template<typename T, std::size_t dim>
Octree_<T,dim>* Octree_<T,dim>::operator[](size_t idx)
{
	assert(idx < nbCells);
	return cells[idx];
}

template<typename T, std::size_t dim>
typename Octree_<T,dim>::DataContainer Octree_<T,dim>::toData(const std::vector<Id>& ids)
{
	return DataContainer{ids.begin(), ids.end()};
}

// Build tree from DataPoints with a specified number of points by node
template<typename T, std::size_t dim>
void Octree_<T,dim>::build(const DP& pts, size_t maxDataByNode, T maxSizeByNode, bool parallelBuild)
{
	typedef typename PM::Vector Vector;
	
	BoundingBox box;
	
	Vector minValues = pts.features.rowwise().minCoeff();
	Vector maxValues = pts.features.rowwise().maxCoeff();
	
	Point min = minValues.head(dim);
	Point max = maxValues.head(dim);
	
	Point radii = max - min;
	box.center = min + radii * 0.5;
	
	box.radius = radii(0);
	for(size_t i = 1; i < dim; ++i)
	{
		if(box.radius < radii(i))
		{
			box.radius = radii(i);
		}
	}
	
	box.radius *= 0.5;
	
	const size_t nbpts = pts.getNbPoints();
	std::vector<Id> indexes;
	indexes.reserve(nbpts);
		
	for(size_t i = 0; i < nbpts; ++i)
	{
		indexes.emplace_back(Id(i));
	}
	DataContainer ptsData = toData(indexes);
	
	this->build(pts, std::move(ptsData), std::move(box), maxDataByNode, maxSizeByNode, parallelBuild);
}

//Offset lookup table
template<typename T, std::size_t dim>
struct OctreeHelper;

template<typename T>
struct OctreeHelper<T,3>
{
	static const typename Octree_<T,3>::Point offsetTable[Octree_<T,3>::nbCells];
};

template<typename T>
const typename Octree_<T,3>::Point OctreeHelper<T,3>::offsetTable[Octree_<T,3>::nbCells] = 
		{
			{-0.5, -0.5, -0.5},
			{+0.5, -0.5, -0.5},
			{-0.5, +0.5, -0.5},
			{+0.5, +0.5, -0.5},
			{-0.5, -0.5, +0.5},
			{+0.5, -0.5, +0.5},
			{-0.5, +0.5, +0.5},
			{+0.5, +0.5, +0.5}
		};

template<typename T>
struct OctreeHelper<T,2>
{
	static const typename Octree_<T,2>::Point offsetTable[Octree_<T,2>::nbCells];
};

template<typename T>
const typename Octree_<T,2>::Point OctreeHelper<T,2>::offsetTable[Octree_<T,2>::nbCells] = 
		{
			{-0.5, -0.5},
			{+0.5, -0.5},
			{-0.5, +0.5},
			{+0.5, +0.5}
		};

template<typename T, std::size_t dim>
void Octree_<T,dim>::build(const DP& pts, DataContainer&& ptsData, BoundingBox && bb,
	size_t maxDataByNode, T maxSizeByNode, bool parallelBuild)
{
	this->bb.center = bb.center;
	this->bb.radius = bb.radius;

	//Check stop condition
	if((bb.radius * 2.0 <= maxSizeByNode) or (ptsData.size() <= maxDataByNode))
	{
		data.insert(data.end(), 
			std::make_move_iterator(ptsData.begin()),
			make_move_iterator(ptsData.end()));
		return;
	}
	
	const std::size_t nbData = ptsData.size();
	
	DataContainer splittedPtsData[nbCells];
	for(size_t i = 0; i < nbCells; ++i)
	{
		splittedPtsData[i].reserve(nbData);
	}
		
	for(auto&& d : ptsData)
	{
		(splittedPtsData[childIdx(pts, d)]).emplace_back(d);
	}
	
	for(size_t i = 0; i < nbCells; ++i)
	{
		splittedPtsData[i].shrink_to_fit();
	}
	
	BoundingBox boxes[nbCells];
	const T half_radius = this->bb.radius * 0.5;
	for(size_t i = 0; i < nbCells; ++i)
	{
		const Point offset = OctreeHelper<T,dim>::offsetTable[i] * this->bb.radius;
		boxes[i].radius = half_radius;
		boxes[i].center = this->bb.center + offset;
	}
	
	//For each child build recursively
	std::vector<std::future<void>> futures;
	for(size_t i = 0; i < nbCells; ++i)
	{		
		auto compute = [maxDataByNode, maxSizeByNode, i, &pts, &splittedPtsData, &boxes, this](){
				this->cells[i] = new Octree_<T,dim>();
				this->cells[i]->depth = this->depth + 1;
				this->cells[i]->parent = this;
				
				//next call is not parallelizable
				this->cells[i]->build(pts, std::move(splittedPtsData[i]), std::move(boxes[i]), maxDataByNode, maxSizeByNode, false);
			};
		
		if(parallelBuild)
		{
			futures.push_back(std::async(std::launch::async, compute));
		}
		else
		{
			compute();
		}
	}
	
	for(auto& f : futures)
	{
		f.wait();
	}
}

//------------------------------------------------------------------------------
template<typename T, std::size_t dim>
template<typename Callback>
void Octree_<T,dim>::visit(Callback& cb)
{
	cb(*this);
	
	if(!isLeaf())
	{
		// Recursively traverse my children
		for(size_t i = 0; i < nbCells; ++i)
		{
			cells[i]->visit(cb);
		}
	}
}
