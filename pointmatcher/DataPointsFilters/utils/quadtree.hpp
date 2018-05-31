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
#include "quadtree.h"

#include <iterator>

template< typename T >
Quadtree<T>::Quadtree()
	: parent{nullptr}, 
		cells{nullptr,nullptr,nullptr,nullptr},
		depth{0}
{
}

template< typename T >
Quadtree<T>::Quadtree(const Quadtree<T>& o)
	: bb{o.bb.center, o.bb.radius}, depth{o.depth}
{
	if (!o.parent) parent = nullptr;	
	if(o.isLeaf()) //Leaf case
	{
		//nullify childs
		for(size_t i=0; i<4; ++i)
			cells[i]= nullptr;
		//Copy data
		data.insert(data.end(), o.data.begin(), o.data.end());
	}
	else //Node case
	{
		//Create each child recursively
  	for(size_t i=0; i<4;++i)
  	{
  		cells[i] = new Quadtree<T>(*(o.cells[i]));	
			//Assign parent  	
  		cells[i]->parent = this;
  	}
  	//no data in node to copy 	
	}
}

template< typename T >
Quadtree<T>::Quadtree(Quadtree<T>&& o)
	: parent{nullptr}, bb{o.bb.center, o.bb.radius}, depth{o.depth}
{
	//only allow move of root node
	assert(o.isRoot());
	
	if(o.isLeaf()) //Leaf case
	{
		//Copy data
		data.insert(data.end(), 
			std::make_move_iterator(o.data.begin()), 
			std::make_move_iterator(o.data.end()));
	}
	
	//copy child ptr
	for(size_t i=0; i<4; ++i)
	{
		cells[i] = o.cells[i];
		//Nullify ptrs
		o.cells[i]=nullptr;
	}
}

template< typename T >
Quadtree<T>::~Quadtree()
{	
	//delete recursively childs
	if(!isLeaf())
		for(size_t i=0; i<4; ++i)
			delete cells[i];
}

template< typename T >	
Quadtree<T>& Quadtree<T>::operator=(const Quadtree<T>&o)
{
	if (!o.parent) parent = nullptr;
	depth=o.depth;
	
	if(o.isLeaf()) //Leaf case
	{
		//nullify childs
		for(size_t i=0; i<4; ++i)
			cells[i]= nullptr;
		//Copy data
		data.insert(data.end(), o.data.begin(), o.data.end());
	}
	else //Node case
	{
		//Create each child recursively
		for(size_t i=0; i<4; ++i)
		{
			cells[i] = new Quadtree<T>(*(o.cells[i]));	
			//Assign parent  	
			cells[i]->parent = this;
		}
		//no data in node to copy 	
	}
	return *this;
}

template< typename T >	
Quadtree<T>& Quadtree<T>::operator=(Quadtree<T>&&o)
{
	//only allow move of root node
	assert(o.isRoot());
	
	parent = nullptr;
	bb.center = o.bb.center;
	bb.radius = o.bb.radius;
	
	depth = o.depth;
	
	if(o.isLeaf()) //Leaf case
	{
		//Copy data
		data.insert(data.end(), 
			std::make_move_iterator(o.data.begin()), 
			std::make_move_iterator(o.data.end()));
	}
	
	//copy childs ptrs
	for(size_t i=0; i<4; ++i)
	{
		cells[i] = o.cells[i];
		//Nullify ptrs
		o.cells[i]=nullptr;
	}
	
	return *this;
}

template< typename T >
bool Quadtree<T>::isLeaf() const
{
	return (cells[0]==nullptr);
}
template< typename T >
bool Quadtree<T>::isRoot() const
{
	return (parent==nullptr);
}
template< typename T >
bool Quadtree<T>::isEmpty() const
{
	return (data.size() == 0);
}
template< typename T >
size_t Quadtree<T>::idx(const XY& xy) const
{
	size_t id = 0;
	id|= ((xy.x > bb.center.x) << 0);
	id|= ((xy.y > bb.center.y) << 1);
	return id;
}
template< typename T >
size_t Quadtree<T>::idx(T x, T y) const
{
	size_t id = 0;
	id|= ((x > bb.center.x) << 0);
	id|= ((y > bb.center.y) << 1);
	return id;
}
template< typename T >
size_t Quadtree<T>::getDepth() const
{
	return depth;
}
template< typename T >
typename Quadtree<T>::DataContainer * Quadtree<T>::getData()
{
	return &data;
}
template< typename T >
Quadtree<T>* Quadtree<T>::operator[](size_t idx)
{
	assert(idx<4);
	return cells[idx];
}


#define TO_DATA(pts_, ids_) ([](const DP& pts, const std::vector<Id>& ids) -> DataContainer \
		{ return DataContainer{ids.begin(), ids.end()}; })(pts_, ids_)
						
#define TO_XY(pts, d) pts.features(0,d),pts.features(1,d)

// Build tree from DataPoints with a specified number of points by node
template< typename T >
bool Quadtree<T>::build(const DP& pts, size_t maxDataByNode, bool parallel_build)
{
	typedef typename PM::Vector Vector;
	
	//Build bounding box
	BoundingBox box;
	
	Vector minValues = pts.features.rowwise().minCoeff();
	XY min{minValues(0), minValues(1)};
	Vector maxValues = pts.features.rowwise().maxCoeff();
	XY max{maxValues(0), maxValues(1)};
	
	XY radii = max - min;
	box.center = min + radii * 0.5;
	box.radius = radii.x;
	if (box.radius < radii.y) box.radius = radii.y;
	
	//Transform pts in data
	const size_t nbpts = pts.getNbPoints();
	std::vector<Id> indexes;
	indexes.reserve(nbpts);
		
	for(size_t i=0; i<nbpts; ++i)
		indexes.emplace_back(Id(i));
	
	//FIXME: should be a generic conversion from DP to DataContainer 
	DataContainer datas = TO_DATA(pts, indexes);
	
	//build
	bool ret = true;
	if(parallel_build) 
		ret = this->build_par(pts, std::move(datas), std::move(box), maxDataByNode);
	else 
		ret = this->build(pts, std::move(datas), std::move(box), maxDataByNode);

	return ret;
}

template< typename T >
bool Quadtree<T>::build(const DP& pts, DataContainer&& datas, BoundingBox && bb, size_t maxDataByNode)
{
	static XY offsetTable[4] =
		{
			XY{-0.5, -0.5},
			XY{+0.5, -0.5},
			XY{-0.5, +0.5},
			XY{+0.5, +0.5}
		};
	//Check maxData count
	if(datas.size() <= maxDataByNode)
	{	
		//insert data
		data.insert(data.end(), 
			std::make_move_iterator(datas.begin()), make_move_iterator(datas.end()));
		return (isLeaf());
	}
	
	//Assign bounding box
	this->bb.center = bb.center;
	this->bb.radius = bb.radius;
	
	//Split datas
	const std::size_t nbData = datas.size();
	
	DataContainer sDatas[4];
	for(size_t i=0; i<4; ++i)
		sDatas[i].reserve(nbData);
		
	for(auto&& d : datas)
	{
		//FIXME: Should be a generic conversion from DataPoint considering Data to XY
		(sDatas[idx( TO_XY(pts,d) )]).emplace_back(d);
	}
	
	for(size_t i=0; i<4; ++i)
		sDatas[i].shrink_to_fit();
	
	//Compute new bounding boxes
	BoundingBox boxes[4];
	const T half_radius = this->bb.radius * 0.5;
	for(size_t i=0; i<4; ++i)
	{
		const XY offset = offsetTable[i] * this->bb.radius;
		boxes[i].radius = half_radius;
		boxes[i].center = this->bb.center + offset;
	}
	
	//For each child build recursively
	bool ret = true;
	for(size_t i=0; i<4; ++i)
	{
		cells[i] = new Quadtree<T>();
		cells[i]->depth = this->depth+1;
		ret = ret and cells[i]->build(pts, std::move(sDatas[i]), std::move(boxes[i]), maxDataByNode);		
		//Assign parent
		cells[i]->parent = this;
	}

	return (!isLeaf() and ret);
}

#include <thread>
template< typename T >
bool Quadtree<T>::build_par(const DP& pts, DataContainer&& datas, BoundingBox && bb, size_t maxDataByNode)
{
	static XY offsetTable[4] =
		{
			XY{-0.5, -0.5},
			XY{+0.5, -0.5},
			XY{-0.5, +0.5},
			XY{+0.5, +0.5}
		};
	
	//Check maxData count
	if(datas.size() <= maxDataByNode)
	{			
		//insert data
		data.insert(data.end(), 
			std::make_move_iterator(datas.begin()), make_move_iterator(datas.end()));
		return (isLeaf());
	}
	
	//Assign bounding box
	this->bb.center = bb.center;
	this->bb.radius = bb.radius;
	
	//Split datas
	const std::size_t nbData = datas.size();
	
	DataContainer sDatas[4];
	for(size_t i=0; i<4; ++i)
		sDatas[i].reserve(nbData);
		
	for(auto&& d : datas)
	{
		//FIXME: Should be a generic conversion from DataPoint considering Data to XY
		(sDatas[idx( TO_XY(pts,d) )]).emplace_back(d);
	}
	
	for(size_t i=0; i<4; ++i)
		sDatas[i].shrink_to_fit();
	
	//Compute new bounding boxes
	BoundingBox boxes[4];
	const T half_radius = this->bb.radius * 0.5;
	for(size_t i=0; i<4; ++i)
	{
		const XY offset = offsetTable[i] * this->bb.radius;
		boxes[i].radius = half_radius;
		boxes[i].center = this->bb.center + offset;
	}
	
	//Parallely build each child recursively	
	std::vector<std::thread> threads;
	bool ret = true;
	for(size_t i=0; i<4; ++i)
	{
		threads.push_back( std::thread( [maxDataByNode, i, &pts, &sDatas, &boxes, this](){
			this->cells[i] = new Quadtree<T>();
			//Assign depth
			this->cells[i]->depth = this->depth+1;	
			//Assign parent
			this->cells[i]->parent = this;
			this->cells[i]->build(pts, std::move(sDatas[i]), std::move(boxes[i]), maxDataByNode);	
		}));
	}
	
	// Wait for all thread to finish
	for (std::thread & t : threads)
		if (t.joinable()) t.join();
		
	return (!isLeaf() and ret);
}

//------------------------------------------------------------------------------
template< typename T >
bool Quadtree<T>::build(const DP& pts, T maxSizeByNode, bool parallel_build)
{
	typedef typename PM::Vector Vector;
	
	//Build bounding box
	BoundingBox box;
	
	Vector minValues = pts.features.rowwise().minCoeff();
	XY min{minValues(0), minValues(1)};
	Vector maxValues = pts.features.rowwise().maxCoeff();
	XY max{maxValues(0), maxValues(1)};
	
	XY radii = max - min;
	box.center = min + radii * 0.5;
	box.radius = radii.x;
	if (box.radius < radii.y) box.radius = radii.y;
	
	//Transform pts in data
	const size_t nbpts = pts.getNbPoints();
	std::vector<Id> indexes;
	indexes.reserve(nbpts);
	
	for(size_t i=0; i<nbpts; ++i)
		indexes.emplace_back(Id(i));
	
	//FIXME: should be a generic conversion from DP to DataContainer 
	DataContainer datas = TO_DATA(pts, indexes);
	
	//build
	bool ret = true;
	if(parallel_build) 
		ret = this->build_par(pts, std::move(datas), std::move(box), maxSizeByNode);
	else 
		ret = this->build(pts, std::move(datas), std::move(box), maxSizeByNode);

	return ret;
}

template< typename T >
bool Quadtree<T>::build(const DP& pts, DataContainer&& datas, BoundingBox && bb, T maxSizeByNode)
{
	static XY offsetTable[4] =
		{
			XY{-0.5, -0.5},
			XY{+0.5, -0.5},
			XY{-0.5, +0.5},
			XY{+0.5, +0.5}
		};
	//Check maxData count
	if((bb.radius*2.0 <= maxSizeByNode) or (datas.size() <= 1))
	{
		//insert data
		data.insert(data.end(), 
			std::make_move_iterator(datas.begin()), make_move_iterator(datas.end()));
		return (isLeaf());
	}
	
	//Assign bounding box
	this->bb.center = bb.center;
	this->bb.radius = bb.radius;
	
	//Split datas
	const std::size_t nbData = datas.size();
	
	DataContainer sDatas[4];
	for(size_t i=0; i<4; ++i)
		sDatas[i].reserve(nbData);
		
	for(auto&& d : datas)
	{
		//FIXME: Should be a generic conversion from DataPoint considering Data to XY
		(sDatas[idx( TO_XY(pts,d) )]).emplace_back(d);
	}
	
	for(size_t i=0; i<4; ++i)
		sDatas[i].shrink_to_fit();
	
	//Compute new bounding boxes
	BoundingBox boxes[4];
	const T half_radius = this->bb.radius * 0.5;
	for(size_t i=0; i<4; ++i)
	{
		const XY offset = offsetTable[i] * this->bb.radius;
		boxes[i].radius = half_radius;
		boxes[i].center = this->bb.center + offset;
	}
	
	//For each child build recursively
	bool ret = true;
	for(size_t i=0; i<4; ++i)
	{
		cells[i] = new Quadtree<T>();
		cells[i]->depth = this->depth+1;	
		ret = ret and cells[i]->build(pts, std::move(sDatas[i]), std::move(boxes[i]), maxSizeByNode);		
		//Assign parent
		cells[i]->parent = this;
	}

	return (!isLeaf() and ret);
}

template< typename T >
bool Quadtree<T>::build_par(const DP& pts, DataContainer&& datas, BoundingBox && bb, T maxSizeByNode)
{
	static XY offsetTable[4] =
		{
			XY{-0.5, -0.5},
			XY{+0.5, -0.5},
			XY{-0.5, +0.5},
			XY{+0.5, +0.5}
		};
	
	//Check maxData count
	if((bb.radius*2.0 <= maxSizeByNode) or (datas.size() <= 1))
	{			
		//insert data
		data.insert(data.end(), 
			std::make_move_iterator(datas.begin()), make_move_iterator(datas.end()));
		return (isLeaf());
	}
	
	//Assign bounding box
	this->bb.center = bb.center;
	this->bb.radius = bb.radius;
	
	//Split datas
	const std::size_t nbData = datas.size();
	
	DataContainer sDatas[4];
	for(size_t i=0; i<4; ++i)
		sDatas[i].reserve(nbData);
		
	for(auto&& d : datas)
	{
		//FIXME: Should be a generic conversion from DataPoint considering Data to XY
		(sDatas[idx( TO_XY(pts,d) )]).emplace_back(d);
	}
	
	for(size_t i=0; i<4; ++i)
		sDatas[i].shrink_to_fit();
	
	//Compute new bounding boxes
	BoundingBox boxes[4];
	const T half_radius = this->bb.radius * 0.5;
	for(size_t i=0; i<4; ++i)
	{
		const XY offset = offsetTable[i] * this->bb.radius;
		boxes[i].radius = half_radius;
		boxes[i].center = this->bb.center + offset;
	}
	
	//For each child build recursively	
	std::vector<std::thread> threads;
	bool ret = true;
	for(size_t i=0; i<4; ++i)
	{
		threads.push_back( std::thread( [maxSizeByNode, i, &pts, &sDatas, &boxes, this](){
			this->cells[i] = new Quadtree<T>();
			//Assign depth
			this->cells[i]->depth = this->depth+1;	
			//Assign parent
			this->cells[i]->parent = this;
			this->cells[i]->build(pts, std::move(sDatas[i]), std::move(boxes[i]), maxSizeByNode);	
		}));
	}
	
	// Wait for all thread to finish
	for (std::thread & t : threads)
		if (t.joinable()) t.join();
		
	return (!isLeaf() and ret);
}

//------------------------------------------------------------------------------
template< typename T >
template< typename Callback >
bool Quadtree<T>::visit(Callback& cb)
{
	// Call the callback for this node (if the callback returns false, then
	// stop traversing.
	if (!cb(*this)) return false;

	// If I'm a node, recursively traverse my children
	if (!isLeaf())
		for (size_t i=0; i<4; ++i)
			if (!cells[i]->visit(cb)) return false;

	return true;
}

template class Quadtree<float>;
template class Quadtree<double>;
