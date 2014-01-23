| [Tutorials Home](Tutorials.md)    | [Previous]() | [Next]() |
| ------------- |:-------------:| -----:|

# Applying Transformations to Point Clouds
######Latest update January 24, 2014 by Samuel Charreyron

The outcome of a point cloud registration is some rigid transformation which, when applied to the reading point cloud, best aligns it with the reference point cloud.  This transformation can be represented algebraically with a square matrix of the dimensions of the homogeneous point coordinates. A point cloud is transformed by left-multiplying it by the transformation matrix.

In libpointmatcher, transformations are encapsulated in a TransformationParameters object, which is itself simply an Eigen matrix.

### Example: Applying the identity transformation
In the following example we manually define a transformation by specifying the TransformationParameters object to be an Identity square Eigen matrix of dimension 4.  We apply this transformation to an input point cloud and we expect the result to be identical to the input.

```cpp
#include <pointmatcher/PointMatcher.h>
#include <iostream>
#include <fstream>

typedef PointMatcher<float> PM;

int main(int argc, char *argv[]) {

    if (argc != 2) {
        std::cerr << "Error: invalid number of arguments" << std::endl;
    }

	PM::TransformationParameters T;
	T = PM::TransformationParameters::Identity(4,4);
	std::cout << "Transformation Matrix: " << std::endl << T << std::endl;

	// Load a point cloud from a file
	PM::DataPoints pointCloud;
	std::string inputFile = argv[1];
	pointCloud = PM::DataPoints::load(inputFile);

	// Compute the transformation
	PM::Matrix transformedPoints =  T * pointCloud.features;

	if (transformedPoints == pointCloud.features) {
	   std::cout << "The transformation was completed successfully" << std::endl;
	   return 0;
	} else {
	   std::cerr << "Error: The transformation was not successful¨ << std::endl";
	   return -1;
	}
}
```