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

void acp(const std::vector<glm::vec3> &points, Eigen::Vector3d& centroid, Eigen::Vector3d& eigenValues, Eigen::Matrix3d& eigenVectors) {
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

void acpRecalage(std::vector<glm::vec3> &pointsCanditate, std::vector<glm::vec3> &pointsTarget, glm::mat3 &rotation, glm::vec3 &translation) {
    Eigen::Vector3d centroid1, centroid2;
    Eigen::Vector3d eigenValues1, eigenValues2;
    Eigen::Matrix3d eigenVectors1, eigenVectors2;

    acp(pointsCanditate, centroid1, eigenValues1, eigenVectors1);
    acp(pointsTarget, centroid2, eigenValues2, eigenVectors2);

    Eigen::Matrix3d R = eigenVectors2 * eigenVectors1.transpose();

    if (R.determinant() < 0.0) {
        eigenVectors2.col(2) *= -1.0;
        R = eigenVectors2 * eigenVectors1.transpose();
    }

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            rotation[col][row] = static_cast<float>(R(row, col));
        }
    }

    Eigen::Vector3d T = centroid2 - R * centroid1;

    translation.x = T.x();
    translation.y = T.y();
    translation.z = T.z();
}

void svdRecalage(std::vector<glm::vec3> &pointsCanditate, std::vector<glm::vec3> &pointsTarget, glm::mat3 &rotation, glm::vec3 &translation) {
    // compute center
    Eigen::Vector3d centroidCanditate = Eigen::Vector3d::Zero();
    Eigen::Vector3d centroidTarget = Eigen::Vector3d::Zero();

    for (const glm::vec3& p : pointsCanditate) {
        centroidCanditate += Eigen::Vector3d(p.x, p.y, p.z);
    }
    centroidCanditate /= static_cast<double>(pointsCanditate.size());

    for (const glm::vec3& p : pointsTarget) {
        centroidTarget += Eigen::Vector3d(p.x, p.y, p.z);
    }
    centroidTarget /= static_cast<double>(pointsTarget.size());

    // construct cross covariance matrix
    Eigen::Matrix3d crossCovariance = Eigen::Matrix3d::Zero();

    for (std::size_t i = 0; i < pointsCanditate.size(); i++) {
        glm::vec3 pointCanditate = pointsCanditate[i];
        glm::vec3 pointTarget = pointsTarget[i];
        Eigen::Vector3d p(pointCanditate.x, pointCanditate.y, pointCanditate.z);
        Eigen::Vector3d q(pointTarget.x, pointTarget.y, pointTarget.z);

        p = p - centroidCanditate;
        q = q - centroidTarget;

        crossCovariance += p * q.transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd( crossCovariance, Eigen::ComputeFullU | Eigen::ComputeFullV );
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Vector3d S = svd.singularValues();

    Eigen::Matrix3d R = V * U.transpose();

    Eigen::Vector3d T = centroidTarget - R * centroidCanditate;

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            rotation[col][row] = static_cast<float>(R(row, col));
        }
    }

    translation.x = T.x();
    translation.y = T.y();
    translation.z = T.z();
}

void callback(std::vector<glm::vec3> & points1, std::vector<glm::vec3> points2, polyscope::PointCloud* ps1, polyscope::PointCloud* ps2) {

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

    if (ImGui::Button("ACP Recalage")) {
        glm::mat3 rotation(1.0f);
        glm::vec3 translation(0.0f);

        acpRecalage(points1, points2, rotation, translation);

        for (auto &p : points1) {
            p = rotation * p + translation;
        }

        ps1->updatePointPositions(points1);
        ps2->updatePointPositions(points2);
    }
    if (ImGui::Button("SVD Recalage")) {
        glm::mat3 rotation(1.0f);
        glm::vec3 translation(0.0f);

        svdRecalage(points1, points2, rotation, translation);

        for (auto &p : points1) {
            p = rotation * p + translation;
        }

        ps1->updatePointPositions(points1);
        ps2->updatePointPositions(points2);
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
    // readPNFile(std::string(TP3_Recalage_SOURCE_DIR) + "/data/pointsets/dino_subsampled_extreme.pn", points, normals);

    auto ps = polyscope::registerPointCloud("input ps",points);
    ps->addVectorQuantity("normals", normals);
    ps->setPointRadius(0.001);
    ps->resetTransform();

    std::vector<glm::vec3> points2;
    std::vector<glm::vec3> normals2;

    readPNFile(std::string(TP3_Recalage_SOURCE_DIR) + "/data/pointsets/dino2.pn", points2, normals2);
    // readPNFile(std::string(TP3_Recalage_SOURCE_DIR) + "/data/pointsets/dino2_subsampled_extreme.pn", points2, normals2);

    auto ps2 = polyscope::registerPointCloud("input ps2",points2);
    ps2->addVectorQuantity("normals2", normals2);
    ps2->setPointRadius(0.001);
    ps2->resetTransform();


    // Add the callback
    polyscope::state::userCallback = [&points, &points2, &ps, &ps2]() {
        callback(points, points2, ps, ps2);
    };

    // Show the gui
    polyscope::show();

    return 0;
}
