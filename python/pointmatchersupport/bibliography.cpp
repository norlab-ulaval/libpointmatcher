#include "bibliography.h"

#include "Bibliography.h"

namespace pointmatcher
{
	void pybindBibliography(py::module& p_module)
	{
		using CurrentBibliography = pms::CurrentBibliography;
		py::class_<pms::CurrentBibliography> pyCurrentBibliography(p_module, "CurrentBibliography");

		using Mode = CurrentBibliography::Mode;
		py::enum_<Mode>(pyCurrentBibliography, "Mode")
		    .value("NORMAL", Mode::NORMAL)
			.value("ROSWIKI", Mode::ROSWIKI)
			.value("BIBTEX", Mode::BIBTEX)
			.export_values();

		pyCurrentBibliography
			.def_readwrite("", &CurrentBibliography::mode)
			.def_readwrite("", &CurrentBibliography::indices)
			.def_readwrite("", &CurrentBibliography::entries)

			.def(py::init<Mode>(), py::arg("mode") = Mode::NORMAL)
			.def("dump", [](const CurrentBibliography& self)
				{
					std::ostringstream oss;
					self.dump(oss);
					py::print(oss.str());
				});

		p_module.def("getAndReplaceBibEntries", &pms::getAndReplaceBibEntries, py::arg(""), py::arg("currentBib"));
	}
}