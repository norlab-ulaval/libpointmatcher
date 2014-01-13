# Datapoint Filters

In this section, the various filters which can be applied to the reading and reference point clouds before performing the ICP process.

As a reminder, *datapoint filters* can have several purposes:

* Removing noisy points which render the alignment of point clouds difficult.
* Removing redundant points so as to speed up alignment 
* Adding descriptive information to the points such as a surface normal vector, or the direction from the point to the sensor.

Note that *datapoint filters* differ from *outlier filters* which appear further down the ICP chain and have a different purpose.

[Libpointmatcher](https://github.com/ethz-asl/libpointmatcher) provides developers with a number of datapoint filters which process an input point cloud into an intermediate point cloud used in the alignment procedure.  Filters function as independent modules that can and often are combined into chains.  Sequential chains of datapoint filters can thus be adapted to the alignment problem at hand.

## Filter Index
### Sub-sampling
1. [Bounding Box Filter](#boundingboxhead)

2. [Maximum Density Filter](#maxdensityhead)

3. [Maximum and Minimum Distance Filters](#maxmindistancehead)

4. [Maximum Point Count Filter](#maxpointcounthead)

5. [Maximum Quantile on Axis Filter](#maxquantilehead)

6. [Random Sampling Filter](#randomsamplinghead)

7. [Remove NaN Filter](#removenanhead)

8. [Shadow Point Filter](#shadowpointhead)

### Descriptor Augmenting 
1. [Observation Direction Filter](#obsdirectionhead)

2. [Surface Normal Filter](#surfacenormalhead)

3. [Orient Normals Filter](#orientnormalshead)

4. [Sampling Surface Normal Filter](#samplingnormhead)

5. [Simple Sensor Noise Filter](#sensornoisehead)


## An Example Point Cloud View of an Appartment
![alt text](images/floor_plan.png "Floor plan of the apartment")

The following examples are drawn from the apartment dataset available for [download](http://projects.asl.ethz.ch/datasets/doku.php?id=laserregistration:apartment:home) from the ASL at ETH Zurich.  A top-down view of the point cloud is depicted below, with the colors showing vertical elevation.  The ceiling has been removed from the point cloud such that the floor (in blue) and the walls (in red) are clearly visible.  Note that the coordinate origin is placed in the kitchen at the starting point of data capture, which is in the top-left of the top-down view and floor plan.

![alt text](images/appt_top.png "Top down view of point cloud from appartment dataset")

## Bounding Box Filter <a name="boundingboxhead"></a>
### Description
Points can be excluded from a rectangular bounding region by using this filter.  The box dimensions are specified by defining the maximum and minimum coordinate values in the x,y,z directions. 

|Parameter  |Description  |Default value    |Allowable range|
|---------  |:---------:  |----------------:|--------------:|
|xMin       |Minimum value on the x-axis defining one side of the box | -1.0 | -inf to inf|
|xMax       |Maximum value on the x-axis defining one side of the box | 1.0 | -inf to inf|
|yMin       |Minimum value on the y-axis defining one side of the box | -1.0 | -inf to inf|
|yMax       |Maximum value on the y-axis defining one side of the box | 1.0 | -inf to inf|
|zMin       |Minimum value on the z-axis defining one side of the box | -1.0 | -inf to inf|
|zMax       |Maximum value on the z-axis defining one side of the box | 1.0 | -inf to inf|
|removeInside   |if set to 1, points contained within the box are removed, else points outside are removed  |1   | 0 or 1|

### Example
In the following example, a box filter of the following dimensions was applied to the input point cloud.

|Parameter | Value|
|---------|:-------|
|xMin | -1.0 |
|xMax | 1.0 |
|yMin | -1.0 |
|yMax | 1.0 |
|zMin | -1.0 |
|zMax | 2.0 |
|removeInside | 0 |

Note that only points **outside** the bounding box are removed by the filter by setting the property *removeInside* to 0.  Because the point cloud center is located in the kitchen in the top-left, a square region of 2m x 2m is selected by this filter.  In the following image, the output of the filter is overlaid in white.

![box filter output](images/box_filt.png "Top down view of the appartment point cloud with a box filter applied.  The input is shown in color and the output of the filter is overlaid in white")

## Maximum Density Filter <a name="maxdensityhead"></a>
### Description
A number of filters are used to reduce the number of points in a cloud by randomly sub-sampling or randomly rejecting a set of points.  Points in regions of high density often contain redundant information, and the ICP algorithm could be performed more efficiently with a smaller number of points.  This filter is thus used to homogenize the density of a point cloud by rejection a sub-sample of points in high-density regions.

Points are only considered for rejection if they exceed a density threshold, otherwise they are preserved.  The single parameter of this filter sets the maximum density that should be obtained in the output point cloud.  Points are randomly rejected such that this maximum density is obtained as closely as possible.  

|Parameter  |Description  |Default value    |Allowable range|
|---------  |:---------:  |----------------:|:--------------|
|maxDensity |The desired maximum density of points in *points/m^3* | 10 | min: 0.0000001, max: inf|   

### Example
In the following example we observe the effect of the maximum density filter on the apartment point cloud.  Sub-sampling occurs mostly in high density regions, which colored in red in both images.  The result is an image with lower density overall with the low density regions in blue being preserved. 

![max density before](images/max_dens_before.png "Before applying filtering the points are colored by density with high density regions in red and low density regions in blue")

**Figure A:** Before applying filtering the points are colored by density with high density regions in red and low density regions in blue

![max density after](images/max_dens_after.png "After applying maximum density filter with target density set to 1 point/m^3")

**Figure B:** After applying maximum density filter with target density set to 1 point/m^3

## Maximum and Minimum Distance Filters <a name="maxmindistancehead"></a>
### Description
These filters remove points which lie beyond a threshold distance from the coordinate center.  For the minimum distance filter, points are kept if their distance from the center **greater than** the threshold while for the maximum distance filter if it is **smaller than** the threshold.   The distance threshold can be defined on the x,y, and z axes or can be a radial distance from the center.

|Parameter  |Description  |Default value    |Allowable range|
|---------  |:---------:  |----------------:|:--------------|
|dim        | Dimension over which the distance is thresholded.  If -1, then the threshold is a radial distance from the center | -1 | 0: x, 1: y, 2: z, -1: radial|
|maxDistance or minDistance |Distance threshold (in m) beyond which points are rejected/selected | 1.0 | min: -inf, max: inf|

### Example (Maximum Distance)
In the following example, a maximum distance threshold of 1m is applied radially by setting the dimension parameter to -1.  As shown on the image below, points which lie within a sphere of radius 1m centered at the origin are selected by the filter and are displayed in white.  All other points are rejected by the filter.  Were a maximum distance filter to be replaced by an equivalent minimum distance filter, only points outside the sphere would be selected. 

![max distance after](images/max_dis.png "After applying maximum distance filter with a distance threshold of 1m and the dimension parameter set to radial")

**Figure:** Maximum distance filter with a distance threshold of 1m and the dimension parameter set to radial

## Maximum Point Count Filter <a name="maxpointcounthead"></a>
### Description
The size of the point cloud is reduced by randomly rejecting points if the total number of points exceeds a threshold.

|Parameter  |Description  |Default value    |Allowable range|
|---------  |:---------:|----------------:|:--------------|
|prob        | Probability of keeping a point, 1/decimation factor | 0.75 | min: 0 max: 1 |
|maxCount |number of points beyond which subsampling occurs | 1000 | min: 0, max: 2147483647|

### Example
For the following example we apply a maximum count of 1000, noting that there are over 400,000 points in the input point cloud and a sampling probability of 0.1.  We observe in the output point cloud a point set representing approximately 10% the original number of points selected evenly from the input cloud. 

![max count after](images/max_num.png "After applying maximum point count filter with a maximum count of 1000 and a sampling probability of 0.1")

**Figure:** After applying maximum point count filter with a maximum count of 1000 and a sampling probability of 0.1.  Input point cloud contains 404,395 points, filtered point cloud contains 40,200 points.

## Maximum Quantile on Axis Filter <a name="maxquanthead"></a>
### Description
Points are filtered according to where they lie on a distribution of their positions along a given axis.  The entire distance range is divided into quantiles which lie between 0 and 1.  One can specify the distance quantile above which points are rejected by the filter.

|Parameter  |Description  |Default value    |Allowable range|
|---------  |:---------:|----------------:|:--------------|
|dim        | Dimension over which the distance (from the center) is thresholded | 0 | x:0 y:1 z:2 |
|ratio |Quantile threshold.  Points whose distance exceed this threshold are rejected by the filter | 0.5 | min: 0.0000001, max: 0.9999999 | 

### Example
In the following example, maximum quantile filtering is performed over the x-axis with a quantile threshold of 0.5.  Therefore, points which have an x-value which exceeds the 50% quantile are rejected.  The output of the filter is displayed in white and overlaid with the input point cloud in the image below.  A sampling region centered at the origin and extending in both directions of the x-axis is clearly visible.
  
![max quant after](images/max_quant.png "After applying maximum quantile on axis filter in the x-direction with a maximum quantile of 0.5")

**Figure:** Maximum quantile on axis filter in the x-direction with a maximum quantile of 0.5

## Random Sampling Filter <a name="randsamplinghead"></a>

### Description
This filter behaves similarly to the [Maximum Point Count Filter](#maxpointcounthead) but does not enforce a maximum point constraint.  Instead points are kept by the filter with a fixed probability.

|Parameter  |Description  |Default value    |Allowable range|
|---------  |:---------:|----------------:|:--------------|
|prob        | Probability that a point is kept (1/decimation factor) | 0.75 | min: 0, max: 1 |

### Example
In the following sample, points are kept with a probability of 0.1.  Therefore the total number of points in the output point cloud is approximately 10 times less than the number of points in the input point cloud and the density is decreased overall.

![rand after](images/appt_rand.png "After applying the random sampling filter with a probability of 0.1")

**Figure:** After applying the random sampling filter with a probability of 0.1

## Remove NaN Filter <a name="removenanhead"></a>

### Description
Due to errors in the capture process point clouds may contain points with invalid coordinates.  This filter can be applied to remove points which contain a NaN coordinate, thus producing a "clean" dataset.

##Shadow Point Filter <a name="shadowpointhead"></a>

### Description

Shadow points are noisy points usually located at point cloud edge discontinuities.

*IMPORTANT:* The surface normal descriptors are required in the input point cloud. 


## Observation Direction Filter <a name="obsdirectionhead"></a>

### Description
As opposed to the previous filters, the following does not yield a sub-sample of points but rather augments the input point cloud with additional information.  In particular, this filter adds a descriptor vector to each point representing its direction to the sensor used for capturing the point cloud.  Remark that adding a direction vector is useful for locally captured point clouds in which the position of the sensor is fixed.  In contrast global point clouds which are formed from several local point clouds do not have a fixed sensor position.

The returned direction vector is a vector connecting the point and the sensor, whose positions can be specified in the filter parameters.

|Parameter  |Description  |Default value    |Allowable range|
|---------  |:---------:|----------------:|:--------------|
|x       | x-coordinate of the sensor position | 0.0 | min: -inf, max: inf |
|y       | y-coordinate of the sensor position | 0.0 | min: -inf, max: inf |
|z       | z-coordinate of the sensor position | 0.0 | min: -inf, max: inf |

### Example
**Remark:** The following example uses a local point cloud representing only a portion of the apartment as opposed to previous examples which used a global point cloud representing the entire apartment.  The filter is used to extract direction informations and a small subset of these directions is shown in the following image.  The arrows point towards the position of the sensor.  The input point cloud is color coded according to the z-elevation of the points (red represents the ceiling and blue the floor).

![dir after](images/appt_dir.png "Applying the observation direction filter to a local point cloud")

**Figure:** Applying the observation direction filter to a local point cloud.  A small subset of point observation directions are displayed.

## Surface Normal Filter <a name="surfacenormalhead"></a>

### Description
The surface normal to each point is estimated by finding a number of neighboring points and taking the eigen-vector corresponding to the smallest eigen-value of all neighboring points.  For more theoretical background on the estimation of normal vectors for a point cloud the reader is invited to visit [the following][1].

Remark that that given a surface, the normal vector can point in two possible directions.  Following the apartment example used herein throughout, the normal vector of a wall can point inside towards the room, or outside the apartment.  To align all normal vectors in the same direction, the [orient normals filter](#orientnormalshead) can be used.

|Parameter  |Description  |Default value    |Allowable range|
|---------  |:---------:|----------------:|:--------------|
|knn       | number of neighboring points (including the point itself) to consider when extracting surface normal | 5 | min: 3, max: 2147483647 |
|epsilon       | approximation used in nearest neighbor search | 0.0 | min: 0.0, max: inf |
|keepNormals       | add the normal vector to descriptors | 1 | 1: true, 0: false |
|keepDensities     | add point cloud density to descriptors | 0 | 1: true, 0: false |
|keepEigenValues   | add eigen values to descriptors    | 0 | 1: true, 0: false |
|keepEigenVectors   | add eigen vectors to descriptors    | 0 | 1: true, 0: false |
|keepMatchedIds   | add identifiers of matched points to descriptors (see)    | 0 | 1: true, 0:false |

[1]:http://people.maths.ox.ac.uk/wendland/research/old/reconhtml/node3.html "Holger, Wendland, Mathematical Institute, University of Oxford"

### Example
In this example, we again use a local point cloud of the apartment. You may recognize the input point cloud as a small portion of the local cloud used in the observation direction filter.  The surface normals are extracted using 5 neighboring points and epsilon=5.  A random set of normal vectors is shown in the figure with arrows.  When looking at the ceiling - shown in red - one may see normal vectors either pointing downwards into the apartment or outside the apartment.

![norm after](images/norm.png "Extracting surface normals of a portion of the apartment point cloud")

**Figure:** Extracting surface normals of a portion of the apartment point cloud

## Orient Normals Filter <a name="orientnormalshead"></a>

### Description
As explained previously, neighboring surface normal vectors obtained from the surface normals filter, do not have the same orientation.  This filter enforces this constraint and reorients vectors from the same surface in a consistent direction.  Vectors are reoriented to either point towards the center (inwards), or away from the center (outwards).

__Required descriptors:__  
  `observationDirections` (see ObservationDirectionDataPointsFilter)  
  `normals` (see SurfaceNormalDataPointsFilter, SamplingSurfaceNormalDataPointsFilter)    
__Output descriptor:__ none  
__Sensor assumed to be at the origin:__ no  
__Impact on the number of points:__ none


|Parameter  |Description  |Default value    |Allowable range|
|---------  |:---------:|----------------:|:--------------|
|towardCenter     | Orient vectors to point towards the center | 1 | 1: true, 0: false |

*IMPORTANT:* Both the normal vector descriptor and observation direction descriptors must be present in the input point cloud.  Consequently, both the surface normal and observation filters should be applied prior to using this filter.

### Example
The same input section is used as for extracting the surface normals in the previous section.  The vectors are reoriented to point towards the center which lies in the bottom left corner of the image below.  We now observe that adjacent surface normal vectors point in a consistent direction.

|Figure:  Normal vectors reoriented to point towards the center   |Parameters used   |
|---|:---|
|![norm after](images/orient_norm.png "Normal vectors reoriented to point towards the center") |towardCenter: 1 |


## Sampling Surface Normal Filter <a name="samplingnormhead"></a>

### Description
The above samples extract surface normals at every point in the point cloud.  In point clouds representing planar surfaces however, a significant of redundant information is contained in adjacent normal vectors.  This filter attempts to both reduce the number of points within a point cloud and the number of different normal vectors.  The first is achieved by performing either random sub-sampling as seen previously, or by using one point per box (bin sub-sampling).  The latter is achieved by recursively decomposing the point-cloud space into boxes until each box contains at most knn points.  A single normal vector is computed from the knn points in each box.

|Parameter  |Description  |Default value    |Allowable range|
|---------  |:---------:|----------------:|:--------------|
|ratio     | Ratio of points (sampled randomly) that are preserved within a box | 0.5 | min: 0.0000001, max: 0.9999999 |
|knn       | Number of points contained within a box and number of neighboring points (including the point itself) to consider when extracting surface normal | 7 | min: 3, max: 2147483647 |
|samplingMethod       |  | 0 | 0: random sub-sampling, 1:bin sub-sampling with resulting number of points 1/knn |
|maxBoxDim| Maximum allowed length of a box above with boxes are discarded | inf | min: 0.0000001, max: inf |
|averageExistingDescriptors | Average existing descriptors within a box or keep existing values | 1 | 0: keep existing 1: average existing|
|keepNormals       | add the normal vector to descriptors | 1 | 1: true, 0: false |
|keepDensities     | add point cloud density to descriptors | 0 | 1: true, 0: false |
|keepEigenValues   | add eigen values to descriptors    | 0 | 1: true, 0: false |
|keepEigenVectors   | add eigen vectors to descriptors    | 0 | 1: true, 0: false |
|keepMatchedIds   | add identifiers of matched points to descriptors (see)    | 0 | 1: true, 0: false |

### Example
We reuse the same apartment section to illustrate the sampling of normal vectors.  The decimation rate used is 0.5 such that half of the points from the original point cloud are discarded randomly.  We use a knn value of 100 such that each box contains at most 100 points.  The box centers are shown in red on the following picture.  It is clear that as opposed to previous filters, adjacent points within a box have the same normal vector.

![samp norm after](images/samp_norm.png "Sampled surface normal vectors, with a sampling ratio of 0.5 and 100 nearest neighbors per normal vector")

**Figure:** Sampled surface normal vectors, with a sampling ratio of 0.5 and 100 nearest neighbors per normal vector, box centers shown in red

## Simple Sensor Noise Filter <a name="sensornoisehead"></a>

### Description
This filter is used to augment points with an estimation of position uncertainty based on sensor specifications.  So far only [SICK LMS](http://www.sick.com/group/EN/home/products/product_news/laser_measurement_systems/Pages/lms100.aspx) is supported.  The uncertainty or noise radius is represented in meters, and can be adjusted by varying a gain parameter which amplifies predefined uncertainty levels.

|Parameter  |Description  |Default value    |Allowable range|
|---------  |:---------:|----------------:|:--------------|
|sensorType | The type of sensor used | 0 | 0: SICK LMS |
|gain       | Used to augment uncertainty | 1 | min: 1, max: inf|

## Fixed Step Sampling Filter (To be completed)
The number of points in a point cloud can be reduced by taking random point subsamples.  The filter is parametrized so that a fixed number of points - selected uniformly at random - are 'rejected' in the filtering process.
