// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2012,
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

#include "pointmatcher/PointMatcher.h"

#include <Eigen/Dense>    // various vector operations
#include <Eigen/Geometry> // Eigen::Affine3f (convenience transformations)
#include <cmath>          // std::cos, std::sin
#include <xmmintrin.h>    // _mm_malloc
#include <iostream>       // printing messages
#include <limits>         // std::numeric_limits<float>::quiet_NaN()

/// A simple struct for housing four floats, could be replaced with Eigen::Vector4f.
struct alignas(16u) float4 {
	float x, y, z, w;
};

/// Convenience method for making a float4 from four floats.
inline float4 make_float4(float x, float y, float z, float w) {
	return {x, y, z, w};
}

/// Convenience method for making a float4 from a single float.
inline float4 make_float4(float v) {
	return {v, v, v, v};
}


/// Simulated range image width.
static constexpr unsigned Width = 640u;

/// Simulated range image height.
static constexpr unsigned Height = 480u;

/**
 * \brief Generates a Width * Height 'sin-cos' point cloud.
 *
 * It is worth acknowledging that this method does not simulate an actual range
 * image, the X and Y positions will range from 0.0 to 2 pi.  This is done for
 * simplicity / clarity of generating the data, but the real goal of this
 * tutorial is to demonstrate how you can supply externally managed data to
 * libpointmatcher.
 *
 * That is, the important aspect is generating Width * Height data points, not
 * what the actual 3D positions generated are.
 *
 * \param positions
 *     The already-allocated array of positions to generate data for.
 */
void generatePointCloud(float4 *positions) {
	static constexpr float TwoPi = 6.283185307179586476925286766559005768394338798750211641949f;
	// simple 'linspace' range from 0 to 2pi spread across each dimension
	static constexpr float start = 0.0;
	static constexpr float stop  = TwoPi;
	static constexpr float dx = (stop - start) / static_cast<float>(Width  - 1);
	static constexpr float dy = (stop - start) / static_cast<float>(Height - 1);

	// populate all of the positions
	for (unsigned idx = 0; idx < Width * Height; ++idx) {
		unsigned col = idx % Width;
		unsigned row = idx / Width;

		float x = static_cast<float>(col) * dx;
		float y = static_cast<float>(row) * dy;

		/* the last coordinate 'w' is the homogeneous coordinate.  in this
		 * scenario we should make sure to set it to 1.0f so that if / when
		 * homogenization occurs, the positions will not change.
		 *
		 * adding 10.0f to 'z' to make the "view point" of (0, 0, 0) valid, as
		 * well as typically range data will not have negative measurements. */
		positions[idx] = make_float4(
			x, y, std::cos(x) * std::sin(y) + 10.0f, 1.0f
		);
	}
}

/**
 * \brief Computes the surface normals for the specified positions.
 *
 * This method performs a very naive approximation of the surface normals, doing
 * a cross product between the north, south, east, and west neighbors.  This
 * will NEVER produce normals as nice as the NormalFilter supplied by
 * libpointmatcher.  However, this method is easy to understand, and (due to its
 * naivety) much faster to compute.
 *
 * As with the \ref generatePointCloud method, this method is not to be taken as
 * an authoritative way of computing normals.  It is one of many options in this
 * domain, and exists only to demonstrate how you can coordinate externally
 * computed normals with libpointmatcher.
 *
 * \remark
 *     Though you could be more clever with the borders in this method, the
 *     implementation here deliberately sets the borders to be NaN to show how
 *     the NaN Filter for libpointmatcher behaves -- all positions are valid (no
 *     NaN), but the normals need to get filtered.  Libpointmatcher will handle
 *     this (and all other descriptors) correctly for you.
 *
 * \param positions
 *     The already computed positions of the input point cloud.
 *
 * \param normals
 *     Where to store the computed normals for each point.
 */
void computeNormals(const float4 *positions, float4 *normals) {
	for (unsigned idx = 0; idx < Width * Height; ++idx) {
		unsigned x = idx % Width;
		unsigned y = idx / Width;

		// set the boundaries to NaN
		if (x == 0 || x == Width - 1 || y == 0 || y == Height - 1) {
			normals[idx] = make_float4(std::numeric_limits<float>::quiet_NaN());
			continue;
		}

		// at this point, we can be sure that north / south / east / west are
		// valid indices
		unsigned north_idx = (y - 1) * Width + (x + 0);
		unsigned south_idx = (y + 1) * Width + (x + 0);
		unsigned west_idx  = (y + 0) * Width + (x - 1);
		unsigned east_idx  = (y + 0) * Width + (x + 1);

		// read in the positions
		const float4 &north_4 = positions[north_idx];
		const float4 &south_4 = positions[south_idx];
		const float4 &west_4  = positions[west_idx];
		const float4 &east_4  = positions[east_idx];

		// using Eigen::Vector3f for convenience of vector operations
		Eigen::Vector3f west(west_4.x, west_4.y, west_4.z);
		Eigen::Vector3f north(north_4.x, north_4.y, north_4.z);
		Eigen::Vector3f south(south_4.x, south_4.y, south_4.z);
		Eigen::Vector3f east(east_4.x, east_4.y, east_4.z);

		/* compute the normal, the cross product is setup this way because we
		 * have a "view point" at (0, 0, 0):
		 *
		 *                 N
		 *                 ^
		 *                 |
		 *             W --+--> E
		 *                 |
		 *                 S
		 */
		Eigen::Vector3f v1 = east  - west;
		Eigen::Vector3f v2 = north - south;
		Eigen::Vector3f normal = v1.cross(v2).normalized();

		/* assign the computed normal, we ignore the 'w' component later on so
		 * setting it to 0.0f here does not have any real meaning.  some normal
		 * estimation methods you may find (e.g. in the point cloud library)
		 * may treat 'w' as 'curvature' which can have meaning. */
		normals[idx] = make_float4(normal.x(), normal.y(), normal.z(), 0.0f);
	}
}

/**
 * \brief Performs an arbitrary transformation of the reference positions and
 *        stores this in the reading positions.
 *
 * \param referencePositions
 *     The input positions to transform.
 *
 * \param readingPositions
 *     Where to store the transformed positions.
 *
 * \param xform
 *     The transformation to be applied.  Assumed to consist of only a rotation
 *     and / or translation (that is, no scaling).  This assumption is not
 *     checked, but violating it invalidates the way this method treats the
 *     homogeneous coordinate of the positions.
 */
void transformReference(const float4 *referencePositions, float4 *readingPositions,
                        const Eigen::Affine3f &xform) {
	for (unsigned idx = 0; idx < Width * Height; ++idx) {
		const float4 &ref_4 = referencePositions[idx];
		Eigen::Vector3f ref(ref_4.x, ref_4.y, ref_4.z);
		Eigen::Vector3f read = xform * ref;
		readingPositions[idx] = make_float4(read.x(), read.y(), read.z(), 1.0f);
	}
}

/**
  *
  */
int main(void)
{
	/* allocate memory for the positions and normals, in this tutorial we are
	 * managing everything manually, but it is important to understand that when
	 * we use Eigen::Map later, if you do not allocate aligned memory, then you
	 * need to make sure you map using the Eigen::DontAlign flag. */
	// data for the reference frame
	float4 *reference_positions = (float4 *)_mm_malloc(
		Width * Height * sizeof(float4), 16u
	);
	float4 *reference_normals = (float4 *)_mm_malloc(
		Width * Height * sizeof(float4), 16u
	);
	// data for the reading frame
	float4 *reading_positions = (float4 *)_mm_malloc(
		Width * Height * sizeof(float4), 16u
	);
	float4 *reading_normals = (float4 *)_mm_malloc(
		Width * Height * sizeof(float4), 16u
	);

	/* initialize the problem:
	 * 1. Generate the reference positions.
	 * 2. Compute the reference normals.
	 * 3. Apply an arbitrary transformation to get a reading frame.
	 * 4. Compute the reading frame normals. */
	generatePointCloud(reference_positions);
	computeNormals(reference_positions, reference_normals);
	// there is nothing meaningful about this specific transformation, it is
	// just to get a different frame to run ICP with
	Eigen::Affine3f xform = Eigen::Affine3f::Identity();
	xform.translation() = Eigen::Vector3f::Constant(10.0f);
	xform.linear() = Eigen::AngleAxisf(
		0.5f * M_PI, Eigen::Vector3f::UnitZ()
	).toRotationMatrix();
	transformReference(reference_positions, reading_positions, xform);
	computeNormals(reading_positions, reading_normals);

	/* At last we finally have the data available for a reference and reading
	 * frame.  In the context of your own application, this would be where you
	 * actually start working -- you already have the positions and computed the
	 * normals, now you just need a way of interfacing with libpointmatcher.
	 *
	 * We utilize the Eigen::Map for these purposes, but it is important to
	 * understand that you need to have a firm understanding of how your data
	 * is laid out, and what the expectations of libpointmatcher are.
	 *
	 *     http://libpointmatcher.readthedocs.io/en/latest/Pointclouds/#features-matrix
	 *
	 * Libpoint matcher expects data laid out in rows:
	 *
	 *     +-                       -+
	 *     | x_0  ...  x_i  ...  x_n |
	 *     | y_0  ...  y_i  ...  y_n |
	 *     | z_0  ...  z_i  ...  z_n |
	 *     | w_0  ...  w_i  ...  w_n |
	 *     +-                       -+
	 *
	 * Above, we allocated data as an "array of structs", laid out as
	 *
	 *     +-                  -+
	 *     | x_0  y_0  z_0  w_0 |
	 *     |  .    .    .    .  |
	 *     | x_i  y_i  z_i  w_i |
	 *     |  .    .    .    .  |
	 *     | x_n  y_n  z_n  w_n |
	 *     +-                  -+
	 *
	 * So our data should be thought of as row major storage.  As it turns out,
	 * though, this is perfectly acceptable *ONLY* because when we do a map with
	 * Eigen, Eigen by default assumes column major storage.  So when we map
	 * using 4 rows and N columns, this all works out in the end.
	 *
	 * Similarly, with the normals, we employ the same mapping scheme.  In this
	 * example code, we are also using `float4` for the normals for
	 * demonstrative purposes.  In your own application, you may or may not need
	 * the extra `w` component.  If not, you can simply use a `float3` or
	 * `Eigen::Vector3f` etc.  In the example below, since we have this `w`
	 * component, we need to take one extra step and effectively just ignore `w`
	 * as far as libpointmatcher is concerned.  This is why we use `topRows(3)`:
	 * libpointmatcher expects "normals" to be laid out as <nx, ny, nz>.  Since
	 * our mapped type will have row 0 as x, row 1 as y, row 2 as z, and row 3
	 * as w, we simply want the top three rows.
	 */
	using PM = PointMatcher<float>;
	using DP = typename PM::DataPoints;
	using Matrix = typename PM::Matrix;

	/***************************************************************************
	 * create the reference frame                                              *
	 **************************************************************************/
	DP reference;
	// assign the positions
	reference.features = Eigen::Map<Matrix>(
		reinterpret_cast<float *>(reference_positions), 4, Width * Height
	);
	reference.featureLabels.push_back({"X", 1});
	reference.featureLabels.push_back({"Y", 1});
	reference.featureLabels.push_back({"Z", 1});
	reference.featureLabels.push_back({"W", 1});

	// assign the normals descriptor
	reference.allocateDescriptor("normals", 3);
	Matrix reference_normals_mat = Eigen::Map<Matrix>(
		reinterpret_cast<float *>(reference_normals), 4, Width * Height
	);
	reference.addDescriptor("normals", reference_normals_mat.topRows(3));

	/***************************************************************************
	 * create the reading frame                                                *
	 **************************************************************************/
	DP reading;
	// assign the positions
	reading.features = Eigen::Map<Matrix>(
		reinterpret_cast<float *>(reading_positions), 4, Width * Height
	);
	reading.featureLabels.push_back({"X", 1});
	reading.featureLabels.push_back({"Y", 1});
	reading.featureLabels.push_back({"Z", 1});
	reading.featureLabels.push_back({"W", 1});

	// assign the normals descriptor
	reading.allocateDescriptor("normals", 3);
	Matrix reading_normals_mat = Eigen::Map<Matrix>(
		reinterpret_cast<float *>(reading_normals), 4, Width * Height
	);
	reference.addDescriptor("normals", reading_normals_mat.topRows(3));

	/***************************************************************************
	 * perform the registration                                                *
	 **************************************************************************/
	PM::ICP icp;
	icp.setDefault();// see icp_customized for custom matcher / error minimizer

	// add a NaN data filter
	PointMatcherSupport::Parametrizable::Parameters params; // reused below
	PM::DataPointsFilter *nanFilter = PM::get().DataPointsFilterRegistrar.create(
		"RemoveNaNDataPointsFilter", params
	);
	params.clear();
	icp.readingDataPointsFilters.push_back(nanFilter);
	icp.referenceDataPointsFilters.push_back(nanFilter);

	// compute the transformation to express reading in reference
	PM::TransformationParameters T = icp(reading, reference);

	// transform data to express it in reference
	DP data_out(reading);
	icp.transformations.apply(data_out, T);

	// save files to see the results
	reference.save("test_ref.vtk");
	reading.save("test_data_in.vtk");
	data_out.save("test_data_out.vtk");

	std::cout << "Transform Computed: " << std::endl << T << std::endl;

	std::cout << "freeing frame data" << std::endl;
    // free the externally managed data
    _mm_free(reference_positions);
    _mm_free(reference_normals);
    _mm_free(reading_positions);
    _mm_free(reading_normals);
    std::cout << "done freeing data" << std::endl;
    // segfault after this ...

    return 0;
}
