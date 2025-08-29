#pragma once

#include <vector>
#include <fstream>
#include <iostream>
#include <filesystem>

#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS.hxx>

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepAlgoAPI_Section.hxx> 
#include <BRepAlgoAPI_Fuse.hxx>    
#include <BOPAlgo_Splitter.hxx>
#include <BRep_Builder.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ListOfShape.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <gp_Pln.hxx>
#include <gp_Sphere.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Cone.hxx>
#include <gp_Torus.hxx>
#include <gp_Ax3.hxx>
#include <gp_Pnt.hxx>
#include <gp_Ax2.hxx>

#include "BrepIO.hpp"
#include "common.h"


std::vector<TopoDS_Face> IntersectFacesAndGetFragments(const std::vector<TopoDS_Face>& inputFaces);
void BlindlyExtendFace(TopoDS_Face& face,
	double expansionFactor = 1.0);

void ProcessAllStpAndExportsFragmentedFaces(const std::string& dataDirPath);