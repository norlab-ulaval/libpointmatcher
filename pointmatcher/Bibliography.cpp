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

#include "Bibliography.h"

#include <boost/lexical_cast.hpp>
#include <iostream>

namespace PointMatcherSupport
{
	using namespace std;
	
	Bibliography bibliography()
	{
		return {
			{ "Phillips2007VarTrimmed",
				{
					{ "type", "inproceedings" },
					{ "title", "Outlier robust ICP for minimizing fractional RMSD" },
					{ "author", "Phillips, J.M. and Liu, R. and Tomasi, C." },
					{ "booktitle", "3-D Digital Imaging and Modeling, 2007. 3DIM '07. Sixth International Conference on" },
					{ "year", "2007" },
					{ "pages", "427--434" },
					{ "publisher", "IEEE Press" },
					{ "doi", "10.1109/3DIM.2007.39" },
					{ "fulltext", "http://x86.cs.duke.edu/~tomasi/papers/phillips/phillips3DIM07.pdf" }
				} 
			},
			{ "Chetverikov2002Trimmed", 
				{
					{ "type", "inproceedings" },
					{ "title", "The Trimmed Iterative Closest Point Algorithm" },
					{ "author", "Chetverikov, D. and Svirko, D. and Stepanov, D. and Krsek, P." },
					{ "booktitle", "Pattern Recognition, 2002. Proceedings. 16th International Conference on" },
					{ "year", "2002" },
					{ "pages", "545--548" },
					{ "publisher", "IEEE Press" },
					{ "doi", "10.1109/ICPR.2002.1047997 " },
					{ "fulltext", "http://hci.iwr.uni-heidelberg.de/publications/dip/2002/ICPR2002/DATA/10_1_03.PDF"}
				}
			}
		};
	}
	
	static StringVector splitString(const string& text, char delim)
	{
		StringVector res;
		size_t pos = 0;
		while(true)
		{
			const size_t nextPos = text.find(delim, pos);
			if (nextPos == text.npos)
			{
				res.push_back(text.substr(pos));
				break;
			}
			else
				res.push_back(text.substr(pos, nextPos - pos));
			pos = nextPos + 1;
		}
		return res;
	}
	
	std::string getAndReplaceBibEntries(const std::string& text, BibIndices& indices, StringVector& entries, bool rosWikiAnchor)
	{
		string newText;
		const StringVector words(splitString(text, ' '));
		for (size_t i = 0; i < words.size(); ++i)
		{
			const string& word(words[i]);
			const size_t l(word.length());
			const size_t p(word.find('}'));
			if ((l > 7) && (word.substr(0, 6) == "\\cite{") && (p != string::npos))
			{
				if (rosWikiAnchor)
					newText += "&#91;";
				else
					newText += '[';
				const StringVector keys(splitString(word.substr(6, p-6), ','));
				for (size_t j = 0; j < keys.size(); ++j)
				{
					const string key(keys[j]);
					if (rosWikiAnchor)
						newText += "[[#" + key + "|";
					if (indices.contains(key))
					{
						newText += boost::lexical_cast<string>(indices.get(key)+1);
					}
					else
					{
						size_t index(entries.size());
						entries.push_back(key);
						indices[key] = index;
						newText += boost::lexical_cast<string>(index+1);
					}
					if (rosWikiAnchor)
						newText += "]]";
					if (j+1 != keys.size())
						newText += ',';
				}
				if (rosWikiAnchor)
					newText += "&#93;";
				else
					newText += ']';
				newText += word.substr(p+1);
			}
			else
				newText += word;
			if (i+1 != words.size())
				newText += ' ';
		}
		return newText;
	}
}; // PointMatcherSupport
