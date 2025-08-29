//
//  BrepIO.hpp
//
//  Created by Shi Chen
//	2025.02.01
//  :)

#pragma once
#include <omp.h> 
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <set>

#include <Interface_Static.hxx>
#include <Transfer_TransientProcess.hxx>
#include <XSControl_WorkSession.hxx>
#include <STEPControl_Reader.hxx>
#include <IGESControl_Reader.hxx>
#include <StlAPI_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <IGESControl_Controller.hxx>
#include <IGESControl_Writer.hxx>

#include <TopExp_Explorer.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS.hxx>

namespace BrepIO
{
	std::string generateOutputPath(const std::string& inputPath, 
		const std::string& extension);
	bool importBrep(const std::string& path, 
		TopoDS_Shape& shape);
	bool exportBrep(const std::string& path,
		const TopoDS_Shape& shape);
}

