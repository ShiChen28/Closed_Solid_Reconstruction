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

// 用于gp_Pnt的比较器，以便在std::map中使用
struct PntCompare {
    bool operator()(const gp_Pnt& a, const gp_Pnt& b) const {
        if (a.X() != b.X()) return a.X() < b.X();
        if (a.Y() != b.Y()) return a.Y() < b.Y();
        return a.Z() < b.Z();
    }
};

bool ProcessStpAndCalculateWindingNumber(const std::string& stpFilePath, 
    double meshDeflection, 
    int numRandomPoints, 
    double windingNumberThreshold)
{
    std::filesystem::path inputPath(stpFilePath);
    if (!std::filesystem::exists(inputPath)) {
        std::cerr << "Error: Input file does not exist: " << stpFilePath << std::endl;
        return false;
    }

    std::string baseName = inputPath.stem().string(); // 文件名（不带扩展名）
    std::string parentPath = inputPath.parent_path().string(); // 父目录

    std::string objFilePath = (std::filesystem::path(parentPath) / (baseName + "_mesh.obj")).string();
    std::string plyFilePath = (std::filesystem::path(parentPath) / (baseName + "_points_winding.ply")).string();

    // 1. 使用OCC读取.stp文件
    TopoDS_Shape oc_shape;
    if (!BrepIO::importBrep(stpFilePath, oc_shape)) {
        return false;
    }

    // 2. 使用OCC对.stp文件进行网格化
    BRepMesh_IncrementalMesh(oc_shape, meshDeflection);
    std::cout << "OCC mesh generation completed with deflection: " << meshDeflection << std::endl;

    // 将OCC网格数据提取到libigl所需的Eigen矩阵中
    Eigen::MatrixXd V_occ; // 顶点
    Eigen::MatrixXi F_occ; // 面
    std::vector<Eigen::Vector3d> occ_vertices;
    std::vector<Eigen::Vector3i> occ_faces;
    std::map<gp_Pnt, int, PntCompare> vertex_map; // 用于去重和构建索引
    int current_vertex_id = 0;

    TopExp_Explorer faceExplorer(oc_shape, TopAbs_FACE);
    for (; faceExplorer.More(); faceExplorer.Next()) {
        const TopoDS_Face& face = TopoDS::Face(faceExplorer.Current());
        TopLoc_Location location;
        Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, location);

        if (triangulation.IsNull()) continue;

        const TColgp_Array1OfPnt& nodes = triangulation->Nodes();
        for (int i = nodes.Lower(); i <= nodes.Upper(); ++i) {
            const gp_Pnt& p = nodes.Value(i);
            Eigen::Vector3d v(p.X(), p.Y(), p.Z());
            if (vertex_map.find(p) == vertex_map.end()) {
                vertex_map[p] = current_vertex_id++;
                occ_vertices.push_back(v);
            }
        }

        for (int i = 1; i <= triangulation->NbTriangles(); ++i) {
            Poly_Triangle tri = triangulation->Triangle(i);
            int n1, n2, n3;
            tri.Get(n1, n2, n3);

            if (face.Orientation() == TopAbs_REVERSED) {
                std::swap(n1, n2);
            }

            occ_faces.push_back(Eigen::Vector3i(
                vertex_map[nodes.Value(n1)],
                vertex_map[nodes.Value(n2)],
                vertex_map[nodes.Value(n3)]
            ));
        }
    }

    if (occ_vertices.empty() || occ_faces.empty()) {
        std::cerr << "Error: No mesh data extracted from OCC shape for file: " << stpFilePath << std::endl;
        return false;
    }

    V_occ.resize(occ_vertices.size(), 3);
    for (size_t i = 0; i < occ_vertices.size(); ++i) {
        V_occ.row(i) = occ_vertices[i];
    }
    F_occ.resize(occ_faces.size(), 3);
    for (size_t i = 0; i < occ_faces.size(); ++i) {
        F_occ.row(i) = occ_faces[i];
    }
    std::cout << "Extracted " << V_occ.rows() << " vertices and " << F_occ.rows() << " faces from OCC mesh." << std::endl;

    // 3. 使用libigl导出.obj文件
    igl::writeOBJ(objFilePath, V_occ, F_occ);
    std::cout << "Exported mesh to OBJ file: " << objFilePath << std::endl;

    // 4. 然后再使用libigl读取进来 (确保libigl能正确处理)
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    if (!igl::readOBJ(objFilePath, V, F)) {
        std::cerr << "Error: Could not read OBJ file: " << objFilePath << std::endl;
        return false;
    }
    std::cout << "Read mesh from OBJ file using libigl. Vertices: " << V.rows() << ", Faces: " << F.rows() << std::endl;

    // 计算实际AABB
    Eigen::RowVector3d bbox_min = V.colwise().minCoeff();
    Eigen::RowVector3d bbox_max = V.colwise().maxCoeff();

    // 可以稍微扩展一点，保证完全覆盖
    Eigen::RowVector3d padding = 0.01 * (bbox_max - bbox_min);
    bbox_min -= padding;
    bbox_max += padding;

    std::cout << "Bounding Box Min: " << bbox_min << std::endl;
    std::cout << "Bounding Box Max: " << bbox_max << std::endl;

    // 生成随机点
    Eigen::MatrixXd P(numRandomPoints, 3);
    std::default_random_engine generator;
    std::uniform_real_distribution<double> dist_x(bbox_min(0), bbox_max(0));
    std::uniform_real_distribution<double> dist_y(bbox_min(1), bbox_max(1));
    std::uniform_real_distribution<double> dist_z(bbox_min(2), bbox_max(2));

    for (int i = 0; i < numRandomPoints; ++i) {
        P(i, 0) = dist_x(generator);
        P(i, 1) = dist_y(generator);
        P(i, 2) = dist_z(generator);
    }

    // 计算广义卷绕数
    Eigen::VectorXd W;
    AlgorithmTimer timer("igl::winding_number");
    timer.start();
    igl::winding_number(V, F, P, W);
    timer.stop();
    // 导出 PLY
    std::ofstream ofs(plyFilePath);
    if (!ofs.is_open()) {
        std::cerr << "Failed to open PLY file: " << plyFilePath << std::endl;
        return false;
    }

    // 写PLY头
    ofs << "ply\n";
    ofs << "format ascii 1.0\n";
    ofs << "element vertex " << P.rows() << "\n";
    ofs << "property float x\nproperty float y\nproperty float z\n";
    ofs << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    ofs << "end_header\n";

    // 写入点和颜色（0~1渐变）
    for (int i = 0; i < numRandomPoints; ++i) {
        const Eigen::RowVector3d& p = P.row(i);

        // 将W(i)映射到0~1
        double wn = std::clamp(std::abs(W(i)), 0.0, 1.0);
        unsigned char r = static_cast<unsigned char>(std::round((1.0 - wn) * 255.0));
        unsigned char g = static_cast<unsigned char>(std::round(wn * 255.0));
        unsigned char b = 0;

        ofs << p.x() << " " << p.y() << " " << p.z() << " "
            << static_cast<int>(r) << " "
            << static_cast<int>(g) << " "
            << static_cast<int>(b) << "\n";
    }

    ofs.close();
    std::cout << "Exported colored points to PLY file with gradient: " << plyFilePath << std::endl;


    std::cout << "Processing complete for: " << stpFilePath << std::endl;
    return true;
}
