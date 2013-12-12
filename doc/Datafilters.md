# Datapoint Filters

In this section, the various filters which can be applied to the reading and reference point clouds before performing the ICP process.

As a reminder, *datapoint filters* can have several purposes:

* Removing noisy points which render the alignment of point clouds difficult.
* Removing redundant points so as to speed up alignment 
* Adding descriptive information to the points such as a surface normal vector, or the direction from the point to the sensor.

Note that *datapoint filters* differ from *outlier filters* which appear further down the ICP chain and have a different purpose.

[Libpointmatcher](https://github.com/ethz-asl/libpointmatcher) provides developers with a number of datapoint filters which process an input point cloud into an intermediate point cloud used in the alignment procedure.  Filters function as independent modules that can and often are combined into chains.  Sequential chains of datapoint filters can thus be adapted to the alignment problem at hand.

## An Example Point Cloud View of an Appartment
![alt text](images/floor_plan.png "Floor plan of the apartment")

The following examples are drawn from the apartment dataset available for [download](http://projects.asl.ethz.ch/datasets/doku.php?id=laserregistration:apartment:home) from the ASL at ETH Zurich.  A top-down view of the point cloud is depicted below, with the colors showing vertical elevation.  The ceiling has been removed from the point cloud such that the floor (in blue) and the walls (in red) are clearly visible.  Note that the coordinate origin is placed in the kitchen at the starting point of data capture, which is in the top-left of the top-down view and floor plan.

![alt text](images/appt_top.png "Top down view of point cloud from appartment dataset")

## Bounding Box Filter
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

## Maximum Density Filter
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

## Maximum and Minimum Distance Filters
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

## Maximum Point Count Filter
### Description
The size of the point cloud is reduced by randomly rejecting points if the total number of points exceeds a threshold.

|Parameter  |Description  |Default value    |Allowable range|
|---------  |:---------:|----------------:|:--------------|
|prob        | Probability of keeping a point, 1/decimation factor | 0.75 | min: 0 max:1 |
|maxCount |number of points beyond which subsampling occurs | 1000 | min: 0, max: 2147483647|

### Example
For the following example we apply a maximum count of 1000, noting that there are over 400,000 points in the input point cloud and a sampling probability of 0.1.  We observe in the output point cloud a point set representing approximately 10% the original number of points selected evenly from the input cloud. 

![max count after](images/max_num.png "After applying maximum point count filter with a maximum count of 1000 and a sampling probability of 0.1")

**Figure:** After applying maximum point count filter with a maximum count of 1000 and a sampling probability of 0.1.  Input point cloud contains 404,395 points, filtered point cloud contains 40,200 points.

## Maximum Quantile on Axis Filter
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

## Observation Direction Filter

### Description
As opposed to the previous filters, the following does not yield a sub-sample of points but rather augments the input point cloud with additional information.  In particular, this filter adds a descriptor vector to each point representing its direction to the sensor used for capturing the point cloud.  Remark that adding a direction vector is useful for locally captured point clouds in which the position of the sensor is fixed.  In contrast global point clouds which are formed from several local point clouds do not have a fixed sensor position.

The returned direction vector is a vector connecting the point and the sensor, whose positions can be specified in the filter parameters.

|Parameter  |Description  |Default value    |Allowable range|
|---------  |:---------:|----------------:|:--------------|
|x       | x-coordinate of the sensor position | 0.0 | min: -inf, max:inf |
|y       | y-coordinate of the sensor position | 0.0 | min: -inf, max:inf |
|z       | z-coordinate of the sensor position | 0.0 | min: -inf, max:inf |

### Example
**Remark:** The following example uses a local point cloud representing only a portion of the apartment as opposed to previous examples which used a global point cloud representing the entire apartment.  The filter is used to extract direction informations and a small subset of these directions is shown in the following image.  The arrows point towards the position of the sensor.  The input point cloud is color coded according to the z-elevation of the points (red represents the ceiling and blue the floor).

![dir after](images/appt_dir.png "Applying the observation direction filter to a local point cloud")

**Figure:** Applying the observation direction filter to a local point cloud.  A small subset of point observation directions are displayed.

## Surface Normal Filter

### Description
The normal to each point is extracted by finding a number of neighboring points and taking the eigen-vector corresponding to the smallest eigen-value of all neighboring points.  For more theoretical background on the estimation of normal vectors for a point cloud the reader is invited to visit [the following][1].

[1]:http://people.maths.ox.ac.uk/wendland/research/old/reconhtml/node3.html "Holger, Wendland, Mathematical Institute, University of Oxford"


## Orient Normals Filter

### Description

## Fixed Step Sampling Filter (To be completed)
The number of points in a point cloud can be reduced by taking random point subsamples.  The filter is parametrized so that a fixed number of points - selected uniformly at random - are 'rejected' in the filtering process.
