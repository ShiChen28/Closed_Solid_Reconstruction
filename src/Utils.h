#pragma once

#include <vector>
#include <iostream>
#include <random>
#include <fstream>
#include <filesystem>

#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS.hxx>

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
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


#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/writePLY.h>
#include <igl/bounding_box.h>
#include <igl/winding_number.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>

#include <Eigen/Core>

#include "BrepIO.hpp"
#include "common.h"
#include "AlgorithmTimer.h"

std::vector<TopoDS_Face> IntersectFacesAndGetFragments(const std::vector<TopoDS_Face>& inputFaces);
void BlindlyExtendFace(TopoDS_Face& face,
	double expansionFactor = 1.0);

void ProcessAllStpAndExportsFragmentedFaces(const std::string& dataDirPath);

bool ProcessStpAndCalculateWindingNumber(
    const std::string& stpFilePath,
    double meshDeflection = 0.1,
    int numRandomPoints = 10000,
    double windingNumberThreshold = 0.5);