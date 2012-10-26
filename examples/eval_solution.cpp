// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2011,
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
#include "pointmatcher/Timer.h"
#include <cassert>
#include <iostream>
#include <fstream>
#include <map>

#include <boost/program_options.hpp>
#include "boost/filesystem.hpp"

using namespace std;
using namespace PointMatcherSupport;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

typedef PointMatcher<float> PM;
typedef PM::TransformationParameters TP;

struct DataSetInfo
{
	string name;
	bool downloaded;
	string path;
	DataSetInfo(){};
	DataSetInfo(string name, bool downloaded):
		name(name),
		downloaded(downloaded)
	{}
};

struct Config
{
	// Default values
	string path_config;
	string path_download;
	string path_result;
	map<string, DataSetInfo> dataSetStatus;
	
	Config()
	{
		path_config = string(getenv("HOME")) + "/.lpm/eval_solution.conf";
		path_download = string(getenv("HOME")) + "/.lpm/download/";
		path_result = "./";
		DataSetInfo info;
		info = DataSetInfo("apartment", false);
		info.path = "robotics.ethz.ch/~asl-datasets/apartment_03-Dec-2011-18_13_33/csv_local/local_frame.zip";
		dataSetStatus[info.name] = info;
		info = DataSetInfo("eth", false);
		info.path = "http://robotics.ethz.ch/~asl-datasets/ETH_hauptgebaude_23-Aug-2011-18_43_49/csv_local/local_frame.zip";
		dataSetStatus[info.name] = info;
		info = DataSetInfo("plain", false);
		info.path = "http://robotics.ethz.ch/~asl-datasets/plain_01-Sep-2011-16_39_18/csv_local/local_frame.zip";
		dataSetStatus[info.name] = info;
		info = DataSetInfo("stairs", false);
		info.path = "http://robotics.ethz.ch/~asl-datasets/stairs_26-Aug-2011-14_26_14/csv_local/local_frame.zip";
		dataSetStatus[info.name] = info;
		info = DataSetInfo("gazebo", false);
		info.path = "http://robotics.ethz.ch/~asl-datasets/gazebo_winter_18-Jan-2012-16_10_04/csv_local/local_frame.zip";
		dataSetStatus[info.name] = info;
		info = DataSetInfo("wood", false);
		info.path = "http://robotics.ethz.ch/~asl-datasets/wood_summer_25-Aug-2011-13_00_30/csv_local/local_frame.zip";
		dataSetStatus[info.name] = info;
	}

};

po::options_description setupOptions(const string & name);
string outputStatus(map<string, DataSetInfo> dataSetStatus);
string enterValidPath(string message);
void setConfig(Config& config);
void saveConfig(Config& config);
void loadConfig(Config& config);
void downloadDataSets(Config& config, po::variables_map &vm);
void validateFileInfo(const PM::FileInfo &fileInfo);
void displayLoadingBar(const int &i, const int &total, const double sec);

int main(int argc, char *argv[])
{
	// Option parsing
	po::options_description desc = setupOptions(argv[0]);
	po::positional_options_description p;
	p.add("icp-config", -1);

	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).
	          options(desc).positional(p).run(), vm);
	po::notify(vm);

	// Evaluation configuration
	Config config;

	if (vm.count("help")) {
		cout << desc << endl;
		return 1;
	}

	// Check for config
	fs::path c_path(config.path_config);
	if(fs::exists(c_path) == false)
	{
		fs::create_directories(c_path.parent_path());
		cout << ">> no configuration found. Using default values." << endl;
		
	}
	else
	{
		loadConfig(config);
	}

	if (vm.count("config")) 
	{
		setConfig(config);
		saveConfig(config);
		return 0;
	}
	
	if (vm.count("download")) 
	{
		downloadDataSets(config, vm);
		saveConfig(config);
		return 0;
	}

	if (vm.count("icp-config") == false)
	{
		cerr << "You must enter a YAMl file to evaluate it." << endl;
		return 1;
	}


	PM::DataPoints refCloud, readCloud;
	const string yaml_config = vm["icp-config"].as<string>();

	// Starting evaluation
	for(auto it=config.dataSetStatus.begin(); it != config.dataSetStatus.end(); ++it)
	{
		if(vm.count("all") || vm.count(it->second.name))
		{
			if(it->second.downloaded == false)
			{
				cerr << ">> Please download data set first." << endl
				     << ">> You can use the option --download -A to download them all." << endl;
					 return 1;
			}
			
			const string protocol_name = config.path_download + "protocols/" + it->second.name + "_protocol.csv";
			const string data_directory = config.path_download + it->second.name + "/";
			
			const PM::FileInfoVector eval_list(protocol_name, data_directory, "");

			// Ensure that all required columns are there
			validateFileInfo(eval_list[0]);

			cout << endl << "<<< Evaluating " << it->second.name << " >>>" << endl;
			string last_read_name = "";
			string last_ref_name = "";
			// Start evaluation for every line
			for(unsigned i=0; i < eval_list.size(); i++)
			{
				timer t_all;

				// Load point clouds
				if(last_read_name != eval_list[i].readingFileName)
				{
					readCloud = PM::loadCSV(eval_list[i].readingFileName);
					last_read_name = eval_list[i].readingFileName;
					//cout << "Reading cloud " << eval_list[i].readingFileName << " loaded" << endl;
				}

				if(last_ref_name != eval_list[i].referenceFileName)
				{
					refCloud = PM::loadCSV(eval_list[i].referenceFileName);
					last_ref_name = eval_list[i].referenceFileName;
					//cout << "Reference cloud " << eval_list[i].referenceFileName << " loaded" << endl;
				}

				// Build ICP based on config file
				PM::ICP icp;
				ifstream ifs(yaml_config);
				icp.loadFromYaml(ifs);

				const TP Tinit = eval_list[i].initialTransformation;

				displayLoadingBar(i, eval_list.size(), t_all.elapsed());
			}

			cout << endl;

		}
	}
	
	return 0;
}


po::options_description setupOptions(const string & name)
{
	po::options_description desc("Allowed options");
	desc.add_options()
	    ("help,h", "Print this message")
		("icp-config", po::value<string>(), "YAML configuration file")
		("config,C", "Interactive configuration")
		("download,D", "Download selected data sets from the web")
		("evaluate,E", "Evaluate a solution over selected data sets")
		("all,A", "Apply action for all data sets")
		("apartment,a", "Apply action only on the data set Apartment")
		("eth,e", "Apply action only on the data set ETH Hauptgebaude")
		("plain,p", "Apply action only on the data set Mountain Plain")
		("stairs,s", "Apply action only on the data set Stairs")
		("gazebo,g", "Apply action only on the data set Gazebo Winter")
		("wood,w", "Apply action only on the data set Wood Summer")
		;

	return desc;
}

string outputStatus(map<string, DataSetInfo> dataSetStatus)
{
	stringstream ss;

	for(auto it=dataSetStatus.begin(); it != dataSetStatus.end(); ++it)
	{
		string paddedName = it->second.name + ":";
		while (paddedName.length() < 12) paddedName += " ";
		ss << "\t" << paddedName;
		if(it->second.downloaded)
		{
			ss << "downloaded.";
		}
		else
		{
			ss << "not on your system.";
		}
		
		ss << endl;
	}

	return ss.str();
}


string enterValidPath(string message)
{
	bool validPath = false;
	string path_name;
	while(!validPath)
	{
		cout << message;
		getline(cin, path_name);
		validPath = fs::is_directory(fs::path(path_name));
		if(validPath == false)
			cout << ">> Not a valid path" << endl;
	}

	return path_name;
}


void setConfig(Config& config)
{
	string answer = "0";
	cout << "Current configuration:" << endl
		 << "\t Download path: " << config.path_download << endl
		 << "\t Result path: " << config.path_result << endl;

	cout << "Data set status:" << endl;
	cout << outputStatus(config.dataSetStatus);

	while(!(answer == "" || answer == "y" ||answer == "Y" || answer == "n" || answer == "N")) 
	{
		cout << endl << "Do you want to change something? [y/N]: ";
		getline(cin, answer);

		//if(answer == "" || answer == "n" ||answer == "N" )
		//	return 1;
		if(answer == "y" ||answer == "Y" )
		{
			config.path_download = enterValidPath("Enter data set path or where they will be downloaded: ");
			cout << endl;
			config.path_result = enterValidPath("Enter the result path (where the result of the test will be saved): ");

		}
	}
}

void saveConfig(Config& config)
{
	YAML::Emitter emitter;
		
		emitter << YAML::BeginMap;
		emitter << YAML::Key << "path_download";
		emitter << YAML::Value << config.path_download;
		emitter << YAML::Key << "path_result";
		emitter << YAML::Value << config.path_result;

		for(auto it=config.dataSetStatus.begin(); it != config.dataSetStatus.end(); ++it)
		{
			emitter << YAML::Key << it->second.name;
			emitter << YAML::Value;
			emitter << YAML::BeginMap;
				emitter << YAML::Key << "downloaded";
				emitter << YAML::Value << it->second.downloaded;
			emitter << YAML::EndMap;
		}
		emitter << YAML::EndMap;

		std::ofstream fout(config.path_config);
		fout << emitter.c_str();
		fout.close();
}

void loadConfig(Config& config)
{
	ifstream f_config(config.path_config);
	YAML::Parser parser(f_config);

	YAML::Node doc;
	while(parser.GetNextDocument(doc)) 
	{
		doc["path_download"] >> config.path_download;
		doc["path_result"] >> config.path_result;
		for(auto it=config.dataSetStatus.begin(); it != config.dataSetStatus.end(); ++it)
		{
			string dataSetName = it->second.name;
			const YAML::Node &node = doc[dataSetName];
			node["downloaded"] >> it->second.downloaded;
		}
	}
}

void downloadDataSets(Config& config, po::variables_map &vm)
{
	for(auto it=config.dataSetStatus.begin(); it != config.dataSetStatus.end(); ++it)
	{
		if(vm.count("all") || vm.count(it->second.name))
		{
			cout << "Downloading " << it->second.name << "..." << endl;
			fs::path d_path(config.path_download+it->second.name);
			if(!fs::is_directory(d_path))
			{
				cout << d_path.string() << endl;
				fs::create_directories(d_path);
			}
			string cmd = "wget -P " + d_path.string() + " " + it->second.path;
			int sysRes;
			sysRes = system(cmd.c_str());
			
			cmd = "unzip -q " + d_path.string() + "/local_frame.zip -d " + d_path.string() + "/";
			sysRes = system(cmd.c_str());
			
			cmd = "rm " + d_path.string() + "/local_frame.zip";
			sysRes = system(cmd.c_str());

			it->second.downloaded = true;
		}
	}
}

void validateFileInfo(const PM::FileInfo &fileInfo)
{
	if(fileInfo.initialTransformation.rows() == 0)
	{
		cout << "Missing columns representing initial transformation \"iTxy\"" << endl;
		abort();
	}

	if(fileInfo.readingFileName == "")
	{
		cout << "Missing column named \"reading\"" << endl;
		abort();
	}

	if(fileInfo.referenceFileName == "")
	{
		cout << "Missing column named \"reference\"" << endl;
		abort();
	}
	
}

double total_time = 0;
void displayLoadingBar(const int &i, const int &total, const double sec)
{
	total_time += sec;
	const double average_time = total_time/double(i+1);
	const int eta = average_time*double(total-i);
	const int m = eta/60;
	const int h = m/60;

	cout << "\r   " << i << "/" << total << "     last dur: " <<  sec << " sec, avr: " << average_time << " sec, eta: " << h << "h" << m % 60 << "m" << eta % 60 << "s             "; 	
	cout.flush();
}
