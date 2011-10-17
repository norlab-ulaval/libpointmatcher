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
using namespace PointMatcherSupport::BibMode;

typedef PointMatcherSupport::Parametrizable::ParametersDoc ParametersDoc;
typedef PointMatcher<double> PM;

struct Bib
{
	BibIndices indices;
	StringVector entries;
	Bibliography biblio;
	
	Bib():biblio(bibliography()) {}
	void dump() const;
	void dumpWiki() const;
	void dumpBibtex() const;
};

void Bib::dump() const
{
	for (size_t i = 0; i < entries.size(); ++i)
	{
		const string& entryName(entries[i]);
		if (!biblio.contains(entryName))
			throw runtime_error(string("Broken bibliography, missing entry " + entryName));
		const StringMap& entry(biblio.get(entryName));
		
		cout << "[" << i+1 << "]";
		if (entry.contains("title"))
			cout << " " << entry.get("title") << ".";
		if (entry.contains("author"))
			cout << " " << entry.get("author") << "";
		if (entry.contains("booktitle"))
			cout << " In " << entry.get("booktitle") << ".";
		if (entry.contains("journal"))
			cout << " " << entry.get("journal") << ".";
		if (entry.contains("pages"))
			cout << " " << entry.get("pages") << ".";
		if (entry.contains("year"))
			cout << " " << entry.get("year") << ".";
		cout << endl << endl;
	}
}

void Bib::dumpWiki() const
{
	for (size_t i = 0; i < entries.size(); ++i)
	{
		const string& entryName(entries[i]);
		if (!biblio.contains(entryName))
			throw runtime_error(string("Broken bibliography, missing entry " + entryName));
		const StringMap& entry(biblio.get(entryName));
		
		cout << " * " << "<<Anchor(" << entryName << ")>>[" << i+1 << "] -";
		if (entry.contains("title"))
			cout << " '''" << entry.get("title") << ".'''";
		if (entry.contains("author"))
			cout << " " << entry.get("author") << "";
		if (entry.contains("booktitle"))
			cout << " ''In " << entry.get("booktitle") << ".''";
		if (entry.contains("journal"))
			cout << " " << entry.get("journal") << ".";
		if (entry.contains("pages"))
			cout << " " << entry.get("pages") << ".";
		if (entry.contains("year"))
			cout << " " << entry.get("year") << ".";
		if (entry.contains("doi"))
			cout << " DOI: [[http://dx.doi.org/" << entry.get("doi") << "|" << entry.get("doi") << "]].";
		if (entry.contains("fulltext"))
			cout << " [[" << entry.get("fulltext") << "|full text]].";
		cout << endl;
	}
}

void Bib::dumpBibtex() const
{
	for (size_t i = 0; i < entries.size(); ++i)
	{
		const string& entryName(entries[i]);
		if (!biblio.contains(entryName))
			throw runtime_error(string("Broken bibliography, missing entry " + entryName));
		const StringMap& entry(biblio.get(entryName));
		
		cout << "@" << entry.get("type") << "{" << entryName << endl;
		if (entry.contains("title"))
			cout << "\ttitle={" << entry.get("title") << "}," << endl;
		if (entry.contains("author"))
			cout << "\tauthor={" << entry.get("author") << "}," << endl;
		if (entry.contains("booktitle"))
			cout << "\tbooktitle={" << entry.get("booktitle") << "}," << endl;
		if (entry.contains("journal"))
			cout << "\tjournal={" << entry.get("journal") << "}," << endl;
		if (entry.contains("pages"))
			cout << "\tpages={" << entry.get("pages") << "}," << endl;
		if (entry.contains("year"))
			cout << "\tyear={" << entry.get("year") << "}," << endl;
		cout << "}" << endl << endl;
	}
}

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
void dumpRegistrar(const PM& pm, const R& registrar, const std::string& name, Bib& bib, Mode mode)
{
	if (mode == ROSWIKI)
		cout << "=== " << name << " ===\n" << endl;
	else
		cout << "* " << name << " *\n" << endl;
	for (auto it = registrar.begin(); it != registrar.end(); ++it)
	{
		if (mode == ROSWIKI)
			cout << "==== " << it->first << " ====\n" << endl;
		else
			cout << it->first << endl;
		
		cout << getAndReplaceBibEntries(it->second->description(), bib.indices, bib.entries, mode) << endl;
		if (mode == ROSWIKI)
			dumpWiki(it->second->availableParameters());
		else
			cout << it->second->availableParameters();
		cout << endl;
	}
	cout << endl;
}


#define DUMP_REGISTRAR_CONTENT(pm, name, bib, mode) \
	dumpRegistrar(pm, pm.REG(name), # name, bib, mode);

int main(int argc, char *argv[])
{
	PM pm;
	Bib bib;
	
	Mode mode(NORMAL);
	if (argc == 2)
	{
		const string cmd(argv[1]);
		if (cmd == "--roswiki")
			mode = ROSWIKI;
		else if (cmd == "--bibtex")
			mode = BIBTEX;
	}
	
	DUMP_REGISTRAR_CONTENT(pm, Transformation, bib, mode)
	DUMP_REGISTRAR_CONTENT(pm, DataPointsFilter, bib, mode)
	DUMP_REGISTRAR_CONTENT(pm, Matcher, bib, mode)
	DUMP_REGISTRAR_CONTENT(pm, FeatureOutlierFilter, bib, mode)
	DUMP_REGISTRAR_CONTENT(pm, DescriptorOutlierFilter, bib, mode)
	DUMP_REGISTRAR_CONTENT(pm, ErrorMinimizer, bib, mode)
	DUMP_REGISTRAR_CONTENT(pm, TransformationChecker, bib, mode)
	DUMP_REGISTRAR_CONTENT(pm, Inspector, bib, mode)
	DUMP_REGISTRAR_CONTENT(pm, Logger, bib, mode)
	
	switch (mode)
	{
		case NORMAL:
		cout << "* Bibliography *" << endl << endl;
		bib.dump();
		break;
		
		case ROSWIKI:
		cout << "=== Bibliography ===" << endl << endl;
		bib.dumpWiki();
		break;
		
		case BIBTEX:
		bib.dumpBibtex();
		break;
	}
	
	return 0;
}