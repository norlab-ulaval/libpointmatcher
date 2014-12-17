#include "../utest.h"

using namespace std;
using namespace PointMatcherSupport;

//---------------------------
// Loggers
//---------------------------

//TODO: FileLogger
//Log using std::stream.
//- infoFileName (default: /dev/stdout) - name of the file to output infos to
//- warningFileName (default: /dev/stderr) - name of the file to output warnings to
//- displayLocation (default: 0) - display the location of message in source code
TEST(Loggers, FileLogger)
{
	string infoFileName = "utest_info";
	string warningFileName = "utest_warn";

	Logger* fileLog = 
		PM::get().REG(Logger).create(
			"FileLogger", map_list_of
				("infoFileName", infoFileName)
				("warningFileName", warningFileName)
				("displayLocation", "1")
		)
	;
	//TODO: we only test constructor here, check other things...

	delete fileLog;

	// Remove file from disk
	EXPECT_TRUE(boost::filesystem::remove(boost::filesystem::path(infoFileName)));
	EXPECT_TRUE(boost::filesystem::remove(boost::filesystem::path(warningFileName)));
}
