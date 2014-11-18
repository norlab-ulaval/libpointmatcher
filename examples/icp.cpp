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
#include "pointmatcher/Bibliography.h"

#include "boost/filesystem.hpp"

#include <cassert>
#include <fstream>
#include <iostream>
#include <algorithm>

using namespace std;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;
typedef PointMatcherSupport::CurrentBibliography CurrentBibliography;

void listModules();
int validateArgs(const int argc, const char *argv[],
                 bool& isCSV, bool& isTransfoSaved,
                 string& configFile,
                 string& outputBaseFile,
                 string& initTranslation, string& initRotation);
PM::TransformationParameters parseTranslation(string& translation);
PM::TransformationParameters parseRotation(string& rotation);
void usage(const char *argv[]);

/**
  * Code example for ICP taking 2 points clouds (2D or 3D) relatively close
  * and computing the transformation between them.
  *
  * This code is more complete than icp_simple. It can load parameter files and
  * has more options.
  */
int main(int argc, const char *argv[])
{
    bool isCSV = true;
    bool isTransfoSaved = true;
    string configFile;
    string outputBaseFile("test");
    string initTranslation("0,0,0");
    string initRotation("1,0,0;0,1,0;0,0,1");
    const int ret = validateArgs(argc, argv, isCSV, isTransfoSaved, configFile,
                                 outputBaseFile, initTranslation, initRotation);
    if (ret != 0)
        return ret;
    const char *refFile(argv[argc-2]);
    const char *dataFile(argv[argc-1]);

    // Load point clouds
    const DP ref(DP::load(refFile));
    const DP data(DP::load(dataFile));

    // Create the default ICP algorithm
    PM::ICP icp;

    if (configFile.empty())
    {
        // See the implementation of setDefault() to create a custom ICP algorithm
        icp.setDefault();
    }
    else
    {
        // load YAML config
        ifstream ifs(configFile.c_str());
        if (!ifs.good())
        {
            cerr << "Cannot open config file " << configFile << ", usage:"; usage(argv); exit(1);
        }
        icp.loadFromYaml(ifs);
    }

    PM::TransformationParameters translation =
            parseTranslation(initTranslation);
    PM::TransformationParameters rotation = parseRotation(initRotation);
    PM::TransformationParameters initTransfo = translation*rotation;

    PM::Transformation* rigidTrans;
        rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

    if (!rigidTrans->checkParameters(initTransfo)) {
        cerr << endl
             << "Initial transformation is not rigid, identiy will be used"
             << endl;
        initTransfo = PM::TransformationParameters::Identity(4,4);
    }

    const DP initializedData = rigidTrans->compute(data, initTransfo);

    // Compute the transformation to express data in ref
    PM::TransformationParameters T = icp(initializedData, ref);
    cout << "match ratio: " << icp.errorMinimizer->getWeightedPointUsedRatio() << endl;

    // Transform data to express it in ref
    DP data_out(initializedData);
    icp.transformations.apply(data_out, T);

    // Safe files to see the results
    ref.save(outputBaseFile + "_ref.vtk");
    data.save(outputBaseFile + "_data_in.vtk");
    data_out.save(outputBaseFile + "_data_out.vtk");
    if(isTransfoSaved){
        ofstream transfoFile;
        string transfoFileName = outputBaseFile + "_transformation_matrix.txt";
        transfoFile.open(transfoFileName.c_str());
        if(transfoFile.is_open()) {
            transfoFile << "Initial transformation :" << endl;
            transfoFile << initTransfo << endl << endl;
            transfoFile << "ICP transformation :" << endl;
            transfoFile << T << endl << endl;
            transfoFile << "Complete transformation :" << endl;
            transfoFile << T*initTransfo;
            transfoFile.close();
        } else {
            cout << "Unable to write the transformation matrix file\n" << endl;
        }
    }
    else {
        cout << "ICP transformation:" << endl << T << endl;
    }

    return 0;
}

// The following code allows to dump all existing modules
template<typename R>
void dumpRegistrar(const PM& pm, const R& registrar, const std::string& name,
                   CurrentBibliography& bib)
{
    cout << "* " << name << " *\n" << endl;
    for (BOOST_AUTO(it, registrar.begin()); it != registrar.end(); ++it)
    {
        cout << it->first << endl;
        cout << getAndReplaceBibEntries(it->second->description(), bib) << endl;
        cout << it->second->availableParameters() << endl;
    }
    cout << endl;
}

#define DUMP_REGISTRAR_CONTENT(pm, name, bib) \
    dumpRegistrar(pm, pm.REG(name), # name, bib);

void listModules()
{
    CurrentBibliography bib;

    DUMP_REGISTRAR_CONTENT(PM::get(), Transformation, bib)
            DUMP_REGISTRAR_CONTENT(PM::get(), DataPointsFilter, bib)
            DUMP_REGISTRAR_CONTENT(PM::get(), Matcher, bib)
            DUMP_REGISTRAR_CONTENT(PM::get(), OutlierFilter, bib)
            DUMP_REGISTRAR_CONTENT(PM::get(), ErrorMinimizer, bib)
            DUMP_REGISTRAR_CONTENT(PM::get(), TransformationChecker, bib)
            DUMP_REGISTRAR_CONTENT(PM::get(), Inspector, bib)
            DUMP_REGISTRAR_CONTENT(PM::get(), Logger, bib)

            cout << "* Bibliography *" << endl << endl;
    bib.dump(cout);
}

// Make sure that the command arguments make sense
int validateArgs(const int argc, const char *argv[],
                 bool& isCSV, bool& isTransfoSaved,
                 string& configFile,
                 string& outputBaseFile,
                 string& initTranslation, string& initRotation)
{
    if (argc == 1)
    {
        cerr << "Not enough arguments, usage:";
        usage(argv);
        return 1;
    }
    else if (argc == 2)
    {
        if (string(argv[1]) == "-l")
        {
            listModules();
            return -1; // we use -1 to say that we wish to quit but in a normal way
        }
        else
        {
            cerr << "Wrong option, usage:";
            usage(argv);
            return 2;
        }
    }

    const int endOpt(argc - 2);
    for (int i = 1; i < endOpt; i += 2)
    {
        const string opt(argv[i]);
        if (i + 1 > endOpt)
        {
            cerr << "Missing value for option " << opt << ", usage:"; usage(argv); exit(1);
        }
        if (opt == "--isTransfoSaved") {
            if (strcmp(argv[i+1], "1") == 0 || strcmp(argv[i+1], "true") == 0) {
                isTransfoSaved = true;
            }
            else if (strcmp(argv[i+1], "0") == 0
                     || strcmp(argv[i+1],"false") == 0) {
                isTransfoSaved = false;
            }
            else {
                cerr << "Invalid value for parameter isTransfoSaved." << endl
                     << "Value must be true or false or 1 or 0." << endl
                     << "Default value will be used." << endl;
            }
        }
        else if (opt == "--config") {
            configFile = argv[i+1];
        }
        else if (opt == "--output") {
            outputBaseFile = argv[i+1];
        }
        else if (opt == "--initTranslation") {
            initTranslation = argv[i+1];
        }
        else if (opt == "--initRotation") {
            initRotation = argv[i+1];
        }
        else
        {
            cerr << "Unknown option " << opt << ", usage:"; usage(argv); exit(1);
        }
    }
    return 0;
}

PM::TransformationParameters parseTranslation(string& translation) {
    PM::TransformationParameters parsedTranslation;
    parsedTranslation = PM::TransformationParameters::Identity(4,4);

    translation.erase(std::remove(translation.begin(), translation.end(), '['),
                      translation.end());
    translation.erase(std::remove(translation.begin(), translation.end(), ']'),
                      translation.end());
    std::replace( translation.begin(), translation.end(), ',', ' ');
    std::replace( translation.begin(), translation.end(), ';', ' ');

    float translationValues[3] = {0};
    stringstream translationStringStream(translation);
    for( int i = 0; i < 3; i++) {
        if(!(translationStringStream >> translationValues[i])) {
            cerr << "An error occured while trying to parse the initial "
                 << "translation." << endl
                 << "No initial translation will be used" << endl;
            return parsedTranslation;
        }
    }

    for( int i = 0; i < 3; i++) {
        parsedTranslation(i,3) = translationValues[i];
    }

    return parsedTranslation;
}

PM::TransformationParameters parseRotation(string &rotation){
    PM::TransformationParameters parsedRotation;
    parsedRotation = PM::TransformationParameters::Identity(4,4);

    rotation.erase(std::remove(rotation.begin(), rotation.end(), '['),
                      rotation.end());
    rotation.erase(std::remove(rotation.begin(), rotation.end(), ']'),
                      rotation.end());
    std::replace( rotation.begin(), rotation.end(), ',', ' ');
    std::replace( rotation.begin(), rotation.end(), ';', ' ');

    float rotationMatrix[9] = {0};
    stringstream translationStringStream(rotation);
    for( int i = 0; i < 9; i++) {
        if(!(translationStringStream >> rotationMatrix[i])) {
            cerr << "An error occured while trying to parse the initial "
                 << "rotation." << endl
                 << "No initial translation will be used" << endl;
            return parsedRotation;
        }
    }

    for( int i = 0; i < 9; i++) {
        parsedRotation(i/3,i%3) = rotationMatrix[i];
    }

    return parsedRotation;
}

// Dump command-line help
void usage(const char *argv[])
{
    cerr << endl << endl;
    cerr << "* To list modules:" << endl;
    cerr << "  " << argv[0] << " -l" << endl;
    cerr << endl;
    cerr << "* To run ICP:" << endl;
    cerr << "  " << argv[0] << " [OPTIONS] reference.csv reading.csv" << endl;
    cerr << endl;
    cerr << "OPTIONS can be a combination of:" << endl;
    cerr << "--config YAML_CONFIG_FILE  Load the config from a YAML file (default: default parameters)" << endl;
    cerr << "--output FILENAME          Name of output files (default: test)" << endl;
    cerr << endl;
    cerr << "Running this program with a VTKFileInspector as Inspector will create three" << endl;
    cerr << "vtk ouptput files: ./test_ref.vtk, ./test_data_in.vtk and ./test_data_out.vtk" << endl;
    cerr << endl << "2D Example:" << endl;
    cerr << "  " << argv[0] << " examples/data/2D_twoBoxes.csv examples/data/2D_oneBox.csv" << endl;
    cerr << endl << "3D Example:" << endl;
    cerr << "  " << argv[0] << " examples/data/car_cloud400.csv examples/data/car_cloud401.csv" << endl;
    cerr << endl;
}
