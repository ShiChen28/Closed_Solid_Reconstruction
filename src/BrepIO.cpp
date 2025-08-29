//
//  BrepIO.hpp
//
//  Created by Shi Chen
//	2025.02.01
//  :)

#include "BrepIO.hpp"

std::string BrepIO::generateOutputPath(const std::string& inputPath, const std::string& extension) {
	// Use filesystem to extract the file name and its extension
	std::filesystem::path inputFilePath(inputPath);

	// Get the base file name without extension and the file extension
	std::string baseName = inputFilePath.stem().string();  // Removes extension

	// Construct the output file path by adding suffix to the base name
	std::string outputFileName = baseName + extension;

	// Construct the full output path by combining the directory and new file name
	std::string outputPath = inputFilePath.parent_path().string() + "\\" + outputFileName;

	return outputPath;
}


bool BrepIO::importBrep(const std::string& path, TopoDS_Shape& shape) {
	std::filesystem::path brep_path(path);
	std::string path_ext = brep_path.extension().string();
	std::transform(path_ext.begin(), path_ext.end(), path_ext.begin(), [](unsigned char c) {
		return std::toupper(c);
		});
	if (path_ext == ".STEP" || path_ext == ".STP") {
		STEPControl_Reader aReader;
		IFSelect_ReturnStatus status = aReader.ReadFile(path.c_str());
		if (status == IFSelect_RetDone) {
			aReader.WS()->MapReader()->SetTraceLevel(2);
			Standard_Integer nbr = aReader.NbRootsForTransfer();
			Standard_Integer nbt = aReader.TransferRoots();
			shape = aReader.OneShape();
			if (shape.IsNull()) {
				std::cout << "Failed to read STEP file" << std::endl;
				return false;
			}
		}
		else {
			std::cout << "Failed to read STEP file" << std::endl;
			return false;
		}

	}
	else if (path_ext == ".IGES" || path_ext == ".IGS") {
		IGESControl_Reader aReader;
		Standard_Integer status = aReader.ReadFile(path.c_str());
		if (status == IFSelect_RetDone) {
			aReader.TransferRoots();
			shape = aReader.OneShape();
		}
		else {
			std::cout << "Failed to read IGES file" << std::endl;
			return false;
		}
	}
	return true;
}

bool BrepIO::exportBrep(const std::string& path, const TopoDS_Shape& shape) {
	std::filesystem::path brep_path(path);
	std::string path_ext = brep_path.extension().string();
	std::transform(path_ext.begin(), path_ext.end(), path_ext.begin(), [](unsigned char c) {
		return std::toupper(c);
		});


	if (path_ext == ".STEP" || path_ext == ".STP") {
		STEPControl_StepModelType StepControlType = STEPControl_AsIs;
		STEPControl_Writer aWriter;
		IFSelect_ReturnStatus status = aWriter.Transfer(shape, STEPControl_AsIs);
		if (status != IFSelect_RetDone) {
			std::cout << "Failed to write STEP file" << std::endl;
			return false;
		}
		Standard_Integer result = aWriter.Write(path.c_str());
		if (!result) {
			std::cout << "Failed to write STEP file" << std::endl;
			return false;
		}
	}
	else if (path_ext == ".IGES" || path_ext == ".IGS")
	{
		IGESControl_Controller::Init();
		IGESControl_Writer aWriter(Interface_Static::CVal("XSTEP.iges.unit"),
			Interface_Static::IVal("XSTEP.iges.writebrep.mode"));
		aWriter.AddShape(shape);
		aWriter.ComputeModel();
		Standard_Integer result = aWriter.Write(path.c_str());
		if (!result) {
			std::cout << "Failed to write IGES file" << std::endl;
			return false;
		}
	}

	std::cout << " Write Brep to " << path << std::endl;
	return true;
}

