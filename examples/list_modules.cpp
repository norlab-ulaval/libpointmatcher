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
#include "pointmatcher/Bibliography.h"

using namespace std;
using namespace PointMatcherSupport;

typedef PointMatcherSupport::Parametrizable::ParametersDoc ParametersDoc;
typedef PointMatcher<double> PM;

void dumpWiki(const ParametersDoc& paramsDoc)
{
	cout << endl;
	if (!paramsDoc.empty())
		for (auto it = paramsDoc.cbegin(); it != paramsDoc.cend(); ++it)
		{
			cout << "`" << it->name << "` (default: `" << it->defaultValue << "`";
			if (!it->minValue.empty())
				cout << ", min: `" << it->minValue << "`";
			if (!it->maxValue.empty())
				cout << ", max: `" << it->maxValue << "`";
			cout << ")" << endl;
			cout << endl;
			cout << " . " << it->doc << endl;
			cout << endl;
		}
	else
		cout << " . no parameters" << endl;
}

template<typename R>
void dumpRegistrar(const PM& pm, const R& registrar, const std::string& name, CurrentBibliography& bib)
{
	if (bib.mode == CurrentBibliography::ROSWIKI)
		cout << "=== " << name << " ===\n" << endl;
	else
		cout << "* " << name << " *\n" << endl;
	for (auto it = registrar.begin(); it != registrar.end(); ++it)
	{
		if (bib.mode == CurrentBibliography::ROSWIKI)
			cout << "==== " << it->first << " ====\n" << endl;
		else
			cout << it->first << endl;
		
		cout << getAndReplaceBibEntries(it->second->description(), bib) << endl;
		if (bib.mode == CurrentBibliography::ROSWIKI)
			dumpWiki(it->second->availableParameters());
		else
			cout << it->second->availableParameters();
		cout << endl;
	}
	cout << endl;
}


#define DUMP_REGISTRAR_CONTENT(pm, name, bib) \
	dumpRegistrar(pm, pm.REG(name), # name, bib);

int main(int argc, char *argv[])
{
	PM pm;
	
	// choose bibliography mode
	CurrentBibliography::Mode mode(CurrentBibliography::NORMAL);
	if (argc == 2)
	{
		const string cmd(argv[1]);
		if (cmd == "--roswiki")
			mode = CurrentBibliography::ROSWIKI;
		else if (cmd == "--bibtex")
			mode = CurrentBibliography::BIBTEX;
	}
	CurrentBibliography bib(mode);
	
	DUMP_REGISTRAR_CONTENT(pm, Transformation, bib)
	DUMP_REGISTRAR_CONTENT(pm, DataPointsFilter, bib)
	DUMP_REGISTRAR_CONTENT(pm, Matcher, bib)
	DUMP_REGISTRAR_CONTENT(pm, OutlierFilter, bib)
	DUMP_REGISTRAR_CONTENT(pm, ErrorMinimizer, bib)
	DUMP_REGISTRAR_CONTENT(pm, TransformationChecker, bib)
	DUMP_REGISTRAR_CONTENT(pm, Inspector, bib)
	DUMP_REGISTRAR_CONTENT(pm, Logger, bib)
	
	switch (mode)
	{
		case CurrentBibliography::NORMAL:
		cout << "* Bibliography *" << endl << endl;
		break;
		
		case CurrentBibliography::ROSWIKI:
		cout << "=== Bibliography ===" << endl << endl;
		break;
		
		default:
		break;
	}
	bib.dump(cout);
	
	return 0;
}