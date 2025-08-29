#include "Utils.h"

std::vector<TopoDS_Face> splitByFaces(const TopoDS_Face& face, const std::vector<TopoDS_Face>& faces)
{
    BOPAlgo_Splitter splitter;
    splitter.AddArgument(face);
    for (int i = 0; i < faces.size(); i++)
    {
        splitter.AddTool(faces[i]);
    }
    splitter.SetFuzzyValue(SplitterFuzzyValue);
    splitter.Perform();
    TopoDS_Shape shape = splitter.Shape();

    std::vector<TopoDS_Face> splitFaceList;
    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
    {
        splitFaceList.push_back(TopoDS::Face(exp.Current()));
    }

    return splitFaceList;
}

std::vector<TopoDS_Face> IntersectFacesAndGetFragments(const std::vector<TopoDS_Face>& inputFaces)
{
    std::vector<TopoDS_Face> allFragmentFaces;
    if (inputFaces.empty())
    {
        return allFragmentFaces;
    }

    // 创建一个临时的BOPAlgo_Splitter对象
    BOPAlgo_Splitter splitter;

    // 将所有输入的面作为参数和工具添加到分离器中
    // 这将确保每个面都与所有其他面进行求交和分割
    for (auto face : inputFaces)
    {
		BlindlyExtendFace(face);
        splitter.AddArgument(face); // 将面添加到要被分割的几何体列表中
        splitter.AddTool(face);     // 将面添加到作为切割工具的几何体列表中
    }

    // 设置模糊值，以处理几何容差
    splitter.SetFuzzyValue(SplitterFuzzyValue); 

    // 执行布尔操作
    splitter.Perform();

    // 获取操作结果
    TopoDS_Shape resultShape = splitter.Shape();

    for (TopExp_Explorer anExp(resultShape, TopAbs_FACE); anExp.More(); anExp.Next())
    {
        allFragmentFaces.push_back(TopoDS::Face(anExp.Current()));
    }

    return allFragmentFaces;
}

void BlindlyExtendFace(TopoDS_Face& face, double expansionFactor)
{
    BRepAdaptor_Surface surface(face);
    GeomAbs_SurfaceType label = surface.GetType();
    switch (label)
    {
    case GeomAbs_Plane:
    {
        gp_Pln pln = surface.Plane();
        Standard_Real umin = surface.FirstUParameter();
        Standard_Real umax = surface.LastUParameter();
        Standard_Real vmin = surface.FirstVParameter();
        Standard_Real vmax = surface.LastVParameter();
        Standard_Real du = umax - umin;
        Standard_Real dv = vmax - vmin;
        Standard_Real t = expansionFactor;
        face = BRepBuilderAPI_MakeFace(pln,
            umin - t * du,
            umax + t * du,
            vmin - t * dv,
            vmax + t * dv).Face();
    }
    break;
    case GeomAbs_Cylinder:
    {
        gp_Cylinder cylinder = surface.Cylinder();
        Standard_Real vmin = surface.FirstVParameter();
        Standard_Real vmax = surface.LastVParameter();
        Standard_Real dv = vmax - vmin;
        Standard_Real t = expansionFactor;
        face = BRepBuilderAPI_MakeFace(cylinder,
            0.0,
            2 * M_PI,
            vmin - t * dv,
            vmax + t * dv).Face();
    }
    break;
    case GeomAbs_Sphere:
    {
        gp_Sphere sphere = surface.Sphere();
        face = BRepBuilderAPI_MakeFace(sphere).Face();
    }
    break;
    case GeomAbs_Cone:
    {
        gp_Cone cone = surface.Cone();
        Standard_Real vmin = surface.FirstVParameter();
        Standard_Real vmax = surface.LastVParameter();
        Standard_Real dv = vmax - vmin;
        Standard_Real t = expansionFactor;
        face = BRepBuilderAPI_MakeFace(cone,
            0.0,
            2 * M_PI,
            vmin - t * dv,
            vmax + t * dv).Face();
    }
    break;
    case GeomAbs_Torus:
    {
        gp_Torus torus = surface.Torus();
        Standard_Real vmin = surface.FirstVParameter();
        Standard_Real vmax = surface.LastVParameter();
        Standard_Real dv = vmax - vmin;
        Standard_Real t = expansionFactor;
        face = BRepBuilderAPI_MakeFace(torus).Face();
    }
    break;
    default:
        break;
    }

}

void ProcessAllStpAndExportsFragmentedFaces(const std::string& dataDirPath)
{
    namespace fs = std::filesystem;

    if (!fs::exists(dataDirPath) || !fs::is_directory(dataDirPath)) {
        std::cerr << "Error: Data directory not found or is not a directory at " << dataDirPath << std::endl;
        return;
    }

    std::cout << "Scanning for .stp and .step files in directory: " << dataDirPath << std::endl;

    // 1. Collect all initial .stp and .step file paths
    std::vector<std::string> stpFilePathsToProcess;
    for (const auto& entry : fs::directory_iterator(dataDirPath))
    {
        if (entry.is_regular_file())
        {
            std::string filePath = entry.path().string();
            std::string fileExtension = entry.path().extension().string();
            std::string fileName = entry.path().filename().string();

            // Convert extension to lowercase for case-insensitive comparison
            std::transform(fileExtension.begin(), fileExtension.end(), fileExtension.begin(),
                [](unsigned char c) { return std::tolower(c); });

            // Skip files that are outputs (start with fragmented_faces_ or shell_)
            if (fileName.rfind("fragmented_faces_", 0) == 0 ||
                fileName.rfind("shell_", 0) == 0 ||
                fileName.rfind("solid_", 0) == 0) {
                continue;
            }

            if (fileExtension == ".stp" || fileExtension == ".step")
            {
                stpFilePathsToProcess.push_back(filePath);
            }
        }
    }

    if (stpFilePathsToProcess.empty()) {
        std::cout << "No .stp or .step files found in " << dataDirPath << ". Exiting." << std::endl;
        return;
    }

    std::cout << "Found " << stpFilePathsToProcess.size() << " .stp/.step files to process." << std::endl;

    // 2. Process each file
    for (const std::string& filePath : stpFilePathsToProcess)
    {
        std::cout << "\n--- Processing file: " << filePath << " ---" << std::endl;

        TopoDS_Shape shape;
        try {
            BrepIO::importBrep(filePath, shape);
        }
        catch (const std::exception& e) {
            std::cerr << "Error importing " << filePath << ": " << e.what() << ". Skipping." << std::endl;
            continue;
        }

        // Extract all faces
        std::vector<TopoDS_Face> inputFaces;
        for (TopExp_Explorer anExp(shape, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            inputFaces.push_back(TopoDS::Face(anExp.Current()));
        }

        if (inputFaces.empty()) {
            std::cout << "No faces found in " << filePath << ". Skipping." << std::endl;
            continue;
        }

        std::cout << "Found " << inputFaces.size() << " faces in the input shape." << std::endl;

        // Get fragment faces
        std::vector<TopoDS_Face> fragmentFaces = IntersectFacesAndGetFragments(inputFaces);
        std::cout << "Intersected faces and got " << fragmentFaces.size() << " fragment faces." << std::endl;

        // Merge into compound
        BRep_Builder builder;
        TopoDS_Compound resultCompound;
        builder.MakeCompound(resultCompound);

        for (const auto& face : fragmentFaces) {
            builder.Add(resultCompound, face);
        }

        // Export with new naming
        fs::path inputPath = filePath;
        std::string outputFileName = "fragmented_faces_" + inputPath.stem().string() + ".stp";
        std::string outputPath = (fs::path(dataDirPath) / outputFileName).string();

        try {
            BrepIO::exportBrep(outputPath, resultCompound);
            std::cout << "Exported fragmented faces to " << outputPath << std::endl;
        }
        catch (const std::exception& e) {
            std::cerr << "Error exporting to " << outputPath << ": " << e.what() << std::endl;
        }
    }

    std::cout << "\n--- All initial .stp and .step files processed ---" << std::endl;
}
