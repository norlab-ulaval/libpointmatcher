#ifndef __POINTMATCHER_IO_H
#define __POINTMATCHER_IO_H

#include "Core.h"
#include <string>
#include <iostream>

template<typename T>
typename MetricSpaceAligner<T>::DataPoints loadCSV(const std::string& fileName);

template<typename T>
typename MetricSpaceAligner<T>::DataPoints loadCSV(std::istream& is);

template<typename T>
typename MetricSpaceAligner<T>::DataPoints loadVTK(const std::string& fileName);

template<typename T>
typename MetricSpaceAligner<T>::DataPoints loadVTK(std::istream& is);

template<typename T>
void saveVTK(const typename MetricSpaceAligner<T>::DataPoints& data, const std::string& fileName);

template<typename T>
void saveVTK(const typename MetricSpaceAligner<T>::DataPoints& data, std::ostream& os);

#endif // __POINTMATCHER_IO_H
