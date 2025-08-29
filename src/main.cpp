//
//  Closed Solid Reconstruction Project
//  Created by Shi Chen
//	2025.08.27
//	:)


#include <fstream>
#include <iostream>
#include <filesystem>
#include <numeric> 

#include "Utils.h"


int main(int argc, char* argv[])
{
    std::string dataDirPath = "..\\data\\";
    ProcessAllStpAndExportsFragmentedFaces(dataDirPath);

    return 0;
}