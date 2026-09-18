#include "polyscope/polyscope.h"
#include <fstream>
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"
#include "nanoflann.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <array>


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

void acp(std::vector<glm::vec3> &points, Eigen::Vector3d& centroid, Eigen::Vector3d& eigenValues, Eigen::Matrix3d& eigenVectors) {
    // compute center
    centroid.setZero();
    for (const glm::vec3& p : points) {
        centroid += Eigen::Vector3d(p.x, p.y, p.z);
    }
    centroid /= static_cast<double>(points.size());

    // compute covariance matrix
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const glm::vec3& p : points) {
        Eigen::Vector3d q (p.x, p.y, p.z);
        q = q - centroid;
        covariance += q * q.transpose();
    }
    covariance /= static_cast<double>(points.size());

    // ACP decompose
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);

    eigenValues = solver.eigenvalues();
    eigenVectors = solver.eigenvectors();
}

void callback(std::vector<glm::vec3> & points1, std::vector<glm::vec3> points2) {

    ImGui::PushItemWidth(100);
    ImGuiIO &io = ImGui::GetIO();
    if (ImGui::Button("ACP")) {
        Eigen::Vector3d centroid1, centroid2;
        Eigen::Vector3d eigenValues1, eigenValues2;
        Eigen::Matrix3d eigenVectors1, eigenVectors2;

        acp(points1, centroid1, eigenValues1, eigenVectors1);
        acp(points2, centroid2, eigenValues2, eigenVectors2);

        glm::vec3 c1( static_cast<float>(centroid1.x()), static_cast<float>(centroid1.y()), static_cast<float>(centroid1.z()) );
        glm::vec3 c2( static_cast<float>(centroid2.x()), static_cast<float>(centroid2.y()), static_cast<float>(centroid2.z()) );

        // draw vectors
        std::array<glm::vec3, 3> points1Axes, points2Axes;

        for (int i = 0; i < points1Axes.size(); i++) {
            Eigen::Vector3d axe1 = std::sqrt(eigenValues1[i]) * eigenVectors1.col(i);
            Eigen::Vector3d axe2 = std::sqrt(eigenValues2[i]) * eigenVectors2.col(i);
            points1Axes[i] = glm::vec3(axe1.x(), axe1.y(), axe1.z());
            points2Axes[i] = glm::vec3(axe2.x(), axe2.y(), axe2.z());

            std::vector<glm::vec3> nodes1 = {
                c1 - points1Axes[i],
                c1 + points1Axes[i]
            };

            std::vector<glm::vec3> nodes2 = {
                c2 - points2Axes[i],
                c2 + points2Axes[i]
            };

            std::vector<std::array<size_t, 2>> edges = {
                {0, 1}
            };

            auto* axis1 = polyscope::registerCurveNetwork("points1 axe: " + std::to_string(i), nodes1, edges);
            axis1->setRadius(0.004);
            axis1->resetTransform();

            auto* axis2 = polyscope::registerCurveNetwork("points2 axe: " + std::to_string(i), nodes2, edges);
            axis2->setRadius(0.004);
            axis2->resetTransform();
        }
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
    ps->setPointRadius(0.0006);
    ps->resetTransform();

    std::vector<glm::vec3> points2;
    std::vector<glm::vec3> normals2;

    readPNFile(std::string(TP3_Recalage_SOURCE_DIR) + "/data/pointsets/dino2.pn", points2, normals2);

    auto ps2 = polyscope::registerPointCloud("input ps2",points2);
    ps2->addVectorQuantity("normals2", normals);
    ps2->setPointRadius(0.0006);
    ps2->resetTransform();


    // Add the callback
    polyscope::state::userCallback = [&points, &points2]() {
        callback(points, points2);
    };

    // Show the gui
    polyscope::show();

    return 0;
}