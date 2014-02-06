| [Tutorials Home](Tutorials.md)    | [Previous](Pointclouds.md) | [Next]() |
| ------------- |:-------------:| -----:|

# Applying Transformations to Point Clouds
######Latest update January 24, 2014 by Samuel Charreyron

The outcome of a point cloud registration is some rigid transformation which, when applied to the reading point cloud, best aligns it with the reference point cloud.  This transformation can be represented algebraically with a square matrix of the dimensions of the homogeneous point coordinates. A point cloud is transformed by left-multiplying it by the transformation matrix.

In libpointmatcher, transformations are encapsulated in a `TransformationParameters` object, which is itself simply an Eigen matrix.

### Example: Applying a Translation

#### Creating a Rigid Transformation Object  
As an example, we wish to apply a simple translation to all points in a specified point cloud.  A rigid transformation is one that moves the point cloud while preserving the distances between points in the cloud.  Rigid transformations can be rotations, translations, and combinations of the two, but not reflections.  A rigid transformation is parametrized by a transformation matrix in homogeneous coordinates.  Therefore for 2D transformations this is a square 3x3 matrix, and for 3D transformations a 4x4 matrix.  Because point cloud registration is concerned with finding an alignment between two point clouds, libpointmatcher provides us with a module for performing rigid transformations.

|**Figure 1:** A 2D transformation matrix representing a translation (tx,ty) and a clockwise rotation of angle theta about the origin  |
|:---|
|![2d transformation matrix](images/2dTransMatrix.gif)|

In the following example we will apply a simple translation: moving each point in the point cloud by 50 meters in the x direction.  The associated transformation matrix is shown below:

|**Figure 2:** A 3D transformation matrix representing a translation of 50 units in the x direction  |
|:---|
|![2d transformation matrix](images/3d50mTrans.gif)|

We make use of the "RigidTransformation" module provided by libpointmatcher.  The `RigidTransformation` class provides several key functions. The `checkParameters` function verifies if a transformation matrix T satisfies the constraints of a rigid transformation: namely that T is orthogonal and has a determinant of 1.  The `correctParameters` function can be called if a transformation does not satisfy orthogonal constraints.  An orthogonal approximation for T will then be computed.  Finally the `compute` function applies the transformation contained in a `TransformationParameters` object.

In the following example we define a transformation by specifying the TransformationParameters object to represent a translation in the x direction.  We do so by setting the top right element of the transformation Eigen matrix to be 50, such that the point cloud will move by 50 meters in the x direction.  We apply this transformation to an input point cloud and save the output point cloud.

```cpp
int main(int argc, char *argv[]) {
	if (argc != 3) {
		std::cerr << "Error: invalid number of arguments" << std::endl;
	}

	PM::TransformationParameters T;
	T = PM::TransformationParameters::Identity(4,4);
	// Applying a translation in the x direction
	T(0,3) = 50;

	std::cout << "Transformation Matrix: " << std::endl << T << std::endl;

	PM::Transformation* rigidTrans;
	rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

	if (!rigidTrans->checkParameters(T)) {
		std::cout << "WARNING: T does not represent a valid rigid transformation\nProjecting onto an orthogonal basis"
				<< std::endl;
		T = rigidTrans->correctParameters(T);
	}

	// Load a point cloud from a file
	PM::DataPoints pointCloud;
	std::string inputFile = argv[1];
	pointCloud = PM::DataPoints::load(inputFile);

	// Compute the transformation
	PM::DataPoints outputCloud =  rigidTrans->compute(pointCloud,T);

	outputCloud.save(argv[2]);

	std::cout << "Transformed cloud saved to " << argv[2] << std::endl;
	return 0;
}
```

The output point cloud can be visualized in Paraview.  We see on the following figure that all points have been shifted by 50 meters in the x direction.
  
|**Figure 3:** Result of the transformation on `examples/data/car_cloud400.csv`.  The white points form the original point cloud, while the green points are the points which were translated.  |
|:---|
|![car translated](images/car_translated.png)|

#### Defining your own Transformation Object
While rigid transformations cover most of the geometric transformations that are used in point cloud registration, we may be interested in defining our own transformations.  In that case we will have to define our own class to represent it and derive this class from the `Transformation` interface class.  

Suppose we are interested in defining a transformation type that only includes 3D translations.  For that, we require that the transformation matrix have the following form.

|**Figure 4:** A 3D transformation matrix representing a pure translation |
|:---|
|![2d translation matrix](images/3dTransMatrix.gif)|

We will name our new transformation class `PureTranslation`.  The `Transformation` interface requires us to define three pure virtual functions: `compute`, `checkParameters`, and `correctParameters`.  The class declaration for `PureTranslation` is as follows:

```cpp
template<typename T>
class PureTranslation : public PointMatcher<T>::Transformation {

	typedef PointMatcher<T> PM;
	typedef typename PM::Transformation Transformation;
	typedef typename PM::DataPoints DataPoints;
	typedef typename PM::TransformationParameters TransformationParameters;

public:
	virtual DataPoints compute(const DataPoints& input, const TransformationParameters& parameters) const;
	virtual bool checkParameters(const TransformationParameters& parameters) const;
	virtual TransformationParameters correctParameters(const TransformationParameters& parameters) const;
};
```

We first implement the `checkParameters` function whose purpose is to determine if the transformation parameters match the type of transformation that is to be performed.  In our case, we wish to check that the transformation represented in the transformation matrix is indeed a pure translation.  This translates to checking that subtracting the rightmost column while ignoring the bottom row from the transformation matrix returns an identity matrix.

```cpp
template<typename T>
inline bool PureTranslation<T>::checkParameters(
		const TransformationParameters& parameters) const {
	const int rows = parameters.rows();
	const int cols = parameters.cols();

	// make a copy of parameters to perform the check
	TransformationParameters parameters_(parameters);

	// set the translation components of the transformation matrix to 0
	parameters_.block(0,cols-1,rows-1,1).setZero();

	// If we have the identity matrix, than this is indeed a pure translation
	if (parameters_.isApprox(TransformationParameters::Identity(rows,cols)))
		return true;
	else
		return false;
}
```

Next, we implement the `correctParameters` function to create a pure translation from a regular transformation.  In other words, we set the transformation matrix to the identity matrix and add the translation components.

```cpp
template<typename T>
inline typename PureTranslation<T>::TransformationParameters PureTranslation<T>::correctParameters(
		const TransformationParameters& parameters) const {
	const int rows = parameters.rows();
	const int cols = parameters.cols();

	// make a copy of the parameters to perform corrections on
	TransformationParameters correctedParameters(parameters);

	// set the top left block to the identity matrix
	correctedParameters.block(0,0,rows-1,cols-1).setIdentity();

	// fix the bottom row
	correctedParameters.block(rows-1,0,1,cols-1).setZero();
	correctedParameters(rows-1,cols-1) = 1;

	return correctedParameters;
}
``` 
Now that we have properly defined our transformation object representing pure translations, we can use it in our code.  We modify the code from the previous example to use our `PureTranslation` object.  Running this code should produce the same results as in the previous case.

```cpp
int main(int argc, char *argv[]) {
	if (argc != 3) {
		std::cerr << "Error: invalid number of arguments" << std::endl;
	}

	PM::TransformationParameters T;
	T = PM::TransformationParameters::Identity(4,4);

	// Applying a translation in the x direction
	T(0,3) = 50;

	std::cout << "Transformation Matrix: " << std::endl << T << std::endl;

	PureTranslation<float> translation;
	//translation = PM::get().REG(Transformation).create("RigidTransformation");

	if (!translation.checkParameters(T)) {
		std::cout << "WARNING: T does not represent a valid rigid transformation\nProjecting onto an orthogonal basis"
				<< std::endl;
		T = translation.correctParameters(T);
	}

	// Load a point cloud from a file
	PM::DataPoints pointCloud;
	std::string inputFile = argv[1];
	pointCloud = PM::DataPoints::load(inputFile);

	// Compute the transformation
	PM::DataPoints outputCloud =  translation.compute(pointCloud,T);

	outputCloud.save(argv[2]);

	std::cout << "Transformed cloud saved to " << argv[2] << std::endl;
	return 0;
}
```
##### Making a Custom Transformation a libpointmatcher Module
Suppose we have defined a useful transformation that we wish to add to libpointmatcher for future use.  We can as an example make a new libpointmatcher module out of the `PureTranslation` transformation class we just designed.

Transformation modules live in the `pointmatcher/TransformationsImpl.h` and implemented in the cpp file of the same name.  Before copying in our `PureTranslation` class declaration, we will add to it an additional function. The `description` function should return some useful information about the transformation such as its name, requirements, and possible parameters that are used in the transformation.

```cpp
inline static const std::string description()
	{
		return "Pure translation transformation\nA rigid transformation with no rotation.";
	}
```

After adding the class to `TransformationsImpl`, we will add it to the registry as a libpointmatcher module.  We do so by adding the following macro in `pointmatcher/registrar.cpp`

```cpp
ADD_TO_REGISTRAR_NO_PARAM(Transformation, PureTranslation, typename TransformationsImpl<T>::PureTranslation)
```

Now recompile the library and check that the new transformation is listed as an available module by running `pcmip -l | grep -C 10 PureTranslation`.
		   

#### Applying a Manual Transformation
We can also perform transformations by directly applying a transformation on a point cloud.  In the following example, we perform a transformation by multiplying a transformation matrix to the original point cloud.  Note that **this does not apply the transformation to associated descriptors** such as surface normals or orientation directions.  For this reason, this approach is strongly discouraged in practice.  The following example code performs the same transformation as in the previous cases:

```cpp
#include <pointmatcher/PointMatcher.h>
#include <iostream>
#include <fstream>

typedef PointMatcher<float> PM;

int main(int argc, char *argv[]) {

    if (argc != 3) {
        std::cerr << "Error: invalid number of arguments" << std::endl;
    }

	PM::TransformationParameters T;
	T = PM::TransformationParameters::Identity(4,4);
	// Applying a translation in the x direction
	T(0,3) = 50;

	std::cout << "Transformation Matrix: " << std::endl << T << std::endl;

	// Load a point cloud from a file
	PM::DataPoints pointCloud;
	std::string inputFile = argv[1];
	pointCloud = PM::DataPoints::load(inputFile);

	// Create a second point cloud of same dimension to store transformed cloud
	PM::DataPoints outputCloud = pointCloud.createSimilarEmpty();

	// Apply transformation and store in output point cloud
	outputCloud.features = T * pointCloud.features;

	outputCloud.save(argv[2]);

	std::cout << "Transformed cloud saved to " << argv[2] << std::endl;
	return 0;
}
```