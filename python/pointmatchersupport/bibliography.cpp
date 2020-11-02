#include "bibliography.h"

#include "Bibliography.h"

namespace python
{
	namespace pointmatchersupport
	{
		void pybindBibliography(py::module& p_module)
		{
			py::bind_vector<pms::StringVector>(p_module, "StringVector");

			py::bind_map<pms::Bibliography>(p_module, "Bibliography")
				.def("clear", &pms::Bibliography::clear, "Remove all items from D.");

			py::bind_map<pms::BibIndices>(p_module, "BibIndices")
				.def("clear", &pms::BibIndices::clear, "Remove all items from D.");

			using CurrentBibliography = pms::CurrentBibliography;
			py::class_<pms::CurrentBibliography> pyCurrentBibliography(p_module, "CurrentBibliography");

			using Mode = CurrentBibliography::Mode;
			py::enum_<Mode>(pyCurrentBibliography, "Mode").value("NORMAL", Mode::NORMAL).value("ROSWIKI", Mode::ROSWIKI)
				.value("BIBTEX", Mode::BIBTEX);

			pyCurrentBibliography.def_readwrite("mode", &CurrentBibliography::mode)
				.def_readwrite("indices", &CurrentBibliography::indices)
				.def_readwrite("entries", &CurrentBibliography::entries)

				.def(py::init<Mode>(), py::arg("mode") = Mode::NORMAL).def("dump", [](const CurrentBibliography& self)
				{
					std::ostringstream oss;
					self.dump(oss);
					py::print(oss.str());
				});

			p_module.def("getAndReplaceBibEntries", &pms::getAndReplaceBibEntries, py::arg(""), py::arg("currentBib"));
		}
	}
}
