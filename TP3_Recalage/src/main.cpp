#include "polyscope/polyscope.h"
#include <fstream>
#include "polyscope/point_cloud.h"
#include "nanoflann.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>


using namespace nanoflann;

struct GLMVectorAdaptor {
    const std::vector<glm::vec3>& pts;

    GLMVectorAdaptor(const std::vector<glm::vec3>& pts)
            : pts(pts) {}

    inline size_t kdtree_get_point_count() const {
        return pts.size();
    }

    inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
        return pts[idx][dim];
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const {
        return false;
    }
};

using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, GLMVectorAdaptor>,
        GLMVectorAdaptor,
        3
>;

std::vector<glm::vec3> readPointCloud(const std::string & path) {

    std::ifstream file(path);

    std::vector<glm::vec3> points;

    double x, y, z;
    while (file >> x >> y >> z) {
        points.emplace_back(x, y, z);
    }
    return points;
}

void readPointCloud(
        const std::string& path,
        std::vector<glm::vec3>& points,
        std::vector<glm::vec3>& normals)
{
    std::ifstream file(path);

    double x, y, z;
    double nx, ny, nz;

    while (file >> x >> y >> z >> nx >> ny >> nz)
    {
        points.emplace_back(x, y, z);
        normals.emplace_back(nx, ny, nz);
    }
}

void readPNFile(
    const std::string& filename,
    std::vector<glm::vec3>& positions,
    std::vector<glm::vec3>& normals)
{
    std::ifstream in(filename, std::ios::binary);

    if (!in) {
        std::cerr << filename << " is not a valid PN file.\n";
        return;
    }

    positions.clear();
    normals.clear();

    float data[6];

    while (in.read(reinterpret_cast<char*>(data), sizeof(data))) {
        positions.emplace_back(data[0], data[1], data[2]);
        normals.emplace_back(data[3], data[4], data[5]);
    }
}

void callback() {

    ImGui::PushItemWidth(100);
    ImGuiIO &io = ImGui::GetIO();
    if (ImGui::Button("cool geometry process")) {


    }

}

int main(int argc, char **argv) {

    // Options
    polyscope::options::autocenterStructures = true;
    polyscope::view::windowWidth = 1024;
    polyscope::view::windowHeight = 1024;

    // Initialize polyscope
    polyscope::init();


    /*

    // Build KD tree
    auto adaptor = GLMVectorAdaptor(points);
    KDTree tree(3,adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    tree.buildIndex();

    glm::vec3 query = points[0];

    const size_t k = 10;
    std::vector<size_t> indices(k);
    std::vector<float> distances(k);
    nanoflann::KNNResultSet<float> resultSet(k);
    resultSet.init(indices.data(), distances.data());

    tree.findNeighbors(
            resultSet,
            &query[0],
            nanoflann::SearchParameters()
    );
    // KD tree Query
    for (size_t j = 0; j < resultSet.size(); ++j)
    {
        size_t neighbor = indices[j];
        float dist2 = distances[j];

        std::cout << neighbor << "  " << dist2 << '\n';
    }
    */

    // Eigen
    Eigen::VectorXd vector;


    // read PN
    // Load point sets
    std::vector<glm::vec3> points;
    std::vector<glm::vec3> normals;

    readPNFile(std::string(TP3_Recalage_SOURCE_DIR) + "/data/pointsets/dino.pn", points, normals);

    auto ps = polyscope::registerPointCloud("input ps",points);
    ps->addVectorQuantity("normals", normals);
    ps->resetTransform();

    std::vector<glm::vec3> points2;
    std::vector<glm::vec3> normals2;

    readPNFile(std::string(TP3_Recalage_SOURCE_DIR) + "/data/pointsets/dino2.pn", points2, normals2);

    auto ps2 = polyscope::registerPointCloud("input ps2",points2);
    ps2->addVectorQuantity("normals2", normals);
    ps2->resetTransform();


    // Add the callback
    polyscope::state::userCallback = callback;

    // Show the gui
    polyscope::show();

    return 0;
}