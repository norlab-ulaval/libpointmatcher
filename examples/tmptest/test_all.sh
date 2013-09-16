
echo defaultSampling0
../../build/examples/pmicp --config defaultSampling0.yaml ../data/cloud.00000.vtk ../data/cloud.00001.vtk  


echo defaultSampling1Avg0
../../build/examples/pmicp --config defaultSampling1Avg0.yaml ../data/cloud.00000.vtk ../data/cloud.00001.vtk  


echo defaultSampling1Avg1
../../build/examples/pmicp --config defaultSampling1Avg1.yaml ../data/cloud.00000.vtk ../data/cloud.00001.vtk  


echo defaultIdentity
../../build/examples/pmicp --config defaultIdentity.yaml ../data/cloud.00000.vtk ../data/cloud.00001.vtk  


echo defaultRemoveNan
../../build/examples/pmicp --config defaultRemoveNan.yaml ../data/cloud.00000.vtk ../data/cloud.00001.vtk  


echo defaultMaxDistFilter
../../build/examples/pmicp --config defaultMaxDistFilter.yaml ../data/cloud.00000.vtk ../data/cloud.00001.vtk  


echo defaultMinDistFilter
../../build/examples/pmicp --config defaultMinDistFilter.yaml ../data/cloud.00000.vtk ../data/cloud.00001.vtk  


echo defaultBbboxFilter
../../build/examples/pmicp --config defaultBbboxFilter.yaml ../data/cloud.00000.vtk ../data/cloud.00001.vtk  


echo defaultMqFilter
../../build/examples/pmicp --config defaultMqFilter.yaml ../data/cloud.00000.vtk ../data/cloud.00001.vtk  


echo defaultMaxDensityDataPointsFilter
../../build/examples/pmicp --config defaultMaxDensityDataPointsFilter.yaml ../data/cloud.00000.vtk ../data/cloud.00001.vtk  


echo defaultOrientNormalsDataPointsFilter
../../build/examples/pmicp --config defaultOrientNormalsDataPointsFilter.yaml ../data/cloud.00000.vtk ../data/cloud.00001.vtk  
 

echo defaultMaxPtCountFilter
../../build/examples/pmicp --config defaultMaxPtCountFilter.yaml ../data/cloud.00000.vtk ../data/cloud.00001.vtk  


echo defaultFixStepSamplingDataPointsFilter
../../build/examples/pmicp --config defaultFixStepSamplingDataPointsFilter.yaml ../data/cloud.00000.vtk ../data/cloud.00001.vtk  


echo defaultShadowDataPointsFilter
../../build/examples/pmicp --config defaultShadowDataPointsFilter.yaml ../data/cloud.00000.vtk ../data/cloud.00001.vtk  


echo defaultSimpleSensorNoiseDataPointsFilter
../../build/examples/pmicp --config defaultSimpleSensorNoiseDataPointsFilter.yaml ../data/cloud.00000.vtk ../data/cloud.00001.vtk  


echo defaultObservationDirectionDataPointsFilter
../../build/examples/pmicp --config defaultObservationDirectionDataPointsFilter.yaml ../data/cloud.00000.vtk ../data/cloud.00001.vtk  


