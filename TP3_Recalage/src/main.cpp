#include "polyscope/polyscope.h"
#include <fstream>
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"
#include "nanoflann.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <limits>
#include <random>
#include <stdexcept>


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

struct RigidTransform {
    glm::mat3 rotation{1.0f};
    glm::vec3 translation{0.0f};
};

RigidTransform applyRandomRigidTransform(std::vector<glm::vec3>& points,
                                         std::vector<glm::vec3>* normals = nullptr) {
    if (points.empty()) {
        throw std::invalid_argument("Point cloud can not be empty");
    }
    if (normals != nullptr && !normals->empty() && normals->size() != points.size()) {
        throw std::invalid_argument("Points and normals must have the same size");
    }

    static std::mt19937 generator(std::random_device{}());
    std::normal_distribution<double> gaussian(0.0, 1.0);
    Eigen::Quaterniond quaternion;
    do {
        quaternion = Eigen::Quaterniond(gaussian(generator), gaussian(generator),
                                        gaussian(generator), gaussian(generator));
    } while (quaternion.norm() < 1e-12);
    quaternion.normalize();

    const Eigen::Matrix3d eigenRotation = quaternion.toRotationMatrix();
    RigidTransform transform;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            transform.rotation[col][row] = static_cast<float>(eigenRotation(row, col));
        }
    }

    glm::vec3 minPoint = points.front();
    glm::vec3 maxPoint = points.front();
    for (const auto& point : points) {
        minPoint = glm::min(minPoint, point);
        maxPoint = glm::max(maxPoint, point);
    }
    const float translationScale = 0.5f * glm::length(maxPoint - minPoint);
    std::uniform_real_distribution<float> uniform(-translationScale, translationScale);
    transform.translation = glm::vec3(uniform(generator), uniform(generator), uniform(generator));

    for (auto& point : points) {
        point = transform.rotation * point + transform.translation;
    }
    if (normals != nullptr) {
        for (auto& normal : *normals) {
            normal = transform.rotation * normal;
        }
    }
    return transform;
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

void acpRecalage(const std::vector<glm::vec3> &pointsCanditate, const std::vector<glm::vec3> &pointsTarget, glm::mat3 &rotation, glm::vec3 &translation) {
    if (pointsCanditate.empty() || pointsTarget.empty()) {
        throw std::invalid_argument("Point sets can not be empty");
    }

    Eigen::Vector3d centroid1, centroid2;
    Eigen::Vector3d eigenValues1, eigenValues2;
    Eigen::Matrix3d eigenVectors1, eigenVectors2;

    acp(pointsCanditate, centroid1, eigenValues1, eigenVectors1);
    acp(pointsTarget, centroid2, eigenValues2, eigenVectors2);

    auto adaptor = GLMVectorAdaptor(pointsTarget);
    KDTree tree(3, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    tree.buildIndex();

    // Each principal axis can point in either direction. Keep only proper rotations.
    const double baseDeterminant = (eigenVectors2 * eigenVectors1.transpose()).determinant();
    double bestSquaredError = std::numeric_limits<double>::infinity();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d T = Eigen::Vector3d::Zero();

    for (int signX : {-1, 1}) {
        for (int signY : {-1, 1}) {
            Eigen::Matrix3d signs = Eigen::Matrix3d::Identity();
            signs(0, 0) = signX;
            signs(1, 1) = signY;
            signs(2, 2) = (baseDeterminant < 0.0 ? -1 : 1) * signX * signY;

            const Eigen::Matrix3d candidateR = eigenVectors2 * signs * eigenVectors1.transpose();
            const Eigen::Vector3d candidateT = centroid2 - candidateR * centroid1;

            double squaredError = 0.0;
            for (const auto& point : pointsCanditate) {
                const Eigen::Vector3d p(point.x, point.y, point.z);
                const Eigen::Vector3d transformed = candidateR * p + candidateT;
                const glm::vec3 query = glm::vec3(transformed.x(), transformed.y(), transformed.z());

                size_t nearestIndex{};
                float squaredDistance{};
                nanoflann::KNNResultSet<float> resultSet(1);
                resultSet.init(&nearestIndex, &squaredDistance);
                tree.findNeighbors(resultSet, &query[0], nanoflann::SearchParameters());
                if (resultSet.size() != 1) {
                    throw std::runtime_error("Nearest neighbor search returned no point");
                }
                squaredError += squaredDistance;
            }

            if (squaredError < bestSquaredError) {
                bestSquaredError = squaredError;
                R = candidateR;
                T = candidateT;
            }
        }
    }

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            rotation[col][row] = static_cast<float>(R(row, col));
        }
    }

    translation.x = T.x();
    translation.y = T.y();
    translation.z = T.z();
}

void svdRecalage(std::vector<glm::vec3> &pointsCanditate, std::vector<glm::vec3> &pointsTarget, glm::mat3 &rotation, glm::vec3 &translation) {

    if (pointsCanditate.empty() || pointsTarget.empty()) {
        throw std::invalid_argument("Point sets cant not be empty");
    }

    if (pointsCanditate.size() !=  pointsTarget.size()) {
        throw std::invalid_argument("Point sets must be the same size");
    }

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

    Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
    if ((V * U.transpose()).determinant() < 0.0) {
        correction(2, 2) = -1.0;
    }
    Eigen::Matrix3d R = V * correction * U.transpose();

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

glm::vec3 projectHPSS(const glm::vec3& inputPoint,
                      const std::vector<glm::vec3>& pointsTarget,
                      const std::vector<glm::vec3>& normalsTarget,
                      const KDTree& tree) {
    constexpr size_t maxNeighbors = 20;
    constexpr int projectionIterations = 5;
    const size_t neighborCount = std::min(maxNeighbors, pointsTarget.size());
    glm::vec3 currentPoint = inputPoint;

    for (int iteration = 0; iteration < projectionIterations; ++iteration) {
        std::array<size_t, maxNeighbors> indices{};
        std::array<float, maxNeighbors> squaredDistances{};
        nanoflann::KNNResultSet<float> resultSet(neighborCount);
        resultSet.init(indices.data(), squaredDistances.data());
        tree.findNeighbors(resultSet, &currentPoint[0], nanoflann::SearchParameters());
        if (resultSet.size() != neighborCount) {
            throw std::runtime_error("HPSS neighbor search returned too few points");
        }

        const float radius = std::max(1e-6f, 1.5f * std::sqrt(squaredDistances[neighborCount - 1]));
        glm::vec3 weightedProjection(0.0f);
        float totalWeight = 0.0f;
        for (size_t i = 0; i < neighborCount; ++i) {
            const glm::vec3& normal = normalsTarget[indices[i]];
            const float normalSquaredLength = glm::dot(normal, normal);
            if (normalSquaredLength <= 1e-12f) {
                continue;
            }

            const float t = std::sqrt(squaredDistances[i]) / radius;
            const float oneMinusT = std::max(0.0f, 1.0f - t);
            const float weight = oneMinusT * oneMinusT * oneMinusT * oneMinusT * (1.0f + 4.0f * t);
            const glm::vec3 planeProjection = currentPoint
                - glm::dot(currentPoint - pointsTarget[indices[i]], normal) / normalSquaredLength * normal;
            weightedProjection += weight * planeProjection;
            totalWeight += weight;
        }
        if (totalWeight <= 1e-8f) {
            break;
        }
        currentPoint = weightedProjection / totalWeight;
    }
    return currentPoint;
}

void icpRecalage(const std::vector<glm::vec3> &pointsCandidate, const std::vector<glm::vec3> &pointsTarget, const std::vector<glm::vec3> &normalsTarget, glm::mat3 &rotation, glm::vec3 &translation) {

    if (pointsCandidate.empty() || pointsTarget.empty()) {
        throw std::invalid_argument("Point sets cant not be empty");
    }
    if (pointsTarget.size() != normalsTarget.size()) {
        throw std::invalid_argument("Target points and normals must have the same size");
    }

    glm::mat3 totalR(1.0f);
    glm::vec3 totalT = glm::vec3(0.0f);

    // apply ACP recalage to the working copy.
    acpRecalage(pointsCandidate, pointsTarget,totalR, totalT);
    std::vector<glm::vec3> pointsSource = pointsCandidate;
    for (auto& point : pointsSource) {
        point = totalR * point + totalT;
    }

    // build KD-Tree for pointsTarget
    auto adaptor = GLMVectorAdaptor(pointsTarget);
    KDTree tree(3,adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    tree.buildIndex();

    double lastRSMD = 0.0;

    // iterations
    for (std::size_t round = 0; round < 100; ++round) {

        // project each source point onto the HPSS surface of the target cloud.
        std::vector<glm::vec3> projectedTargets;
        projectedTargets.reserve(pointsSource.size());
        double RSMD = 0.0;
        for (const auto& point : pointsSource) {
            const glm::vec3 projected = projectHPSS(point, pointsTarget, normalsTarget, tree);
            projectedTargets.push_back(projected);
            const glm::vec3 difference = point - projected;
            RSMD += glm::dot(difference, difference);
        }
        RSMD = std::sqrt(RSMD / pointsSource.size());
        std::cout << "Round: " << round << ", RSMD: " << RSMD << std::endl;

        if (std::abs(RSMD - lastRSMD) < 1e-5) {
            std::cout << "Early stop..." << std::endl;
            break;
        }

        glm::mat3 deltaR(1.0f);
        glm::vec3 deltaT(0.0f);

        // svd recalage
        svdRecalage(pointsSource, projectedTargets, deltaR, deltaT);

        totalT = deltaR * totalT + deltaT;
        totalR = deltaR * totalR;

        // apply rotation and translation
        for (auto& point : pointsSource) {
            point = deltaR * point + deltaT;
        }

        lastRSMD = RSMD;

    }

    // update (return) result
    rotation = totalR;
    translation = totalT;
}

void callback(std::vector<glm::vec3> & points1, std::vector<glm::vec3>& points2,
              std::vector<glm::vec3>& normals2,
              polyscope::PointCloud* ps1, polyscope::PointCloud* ps2) {

    ImGui::PushItemWidth(100);
    ImGuiIO &io = ImGui::GetIO();

    if (ImGui::Button("Random transform target")) {
        const RigidTransform transform = applyRandomRigidTransform(points2, &normals2);
        ps2->updatePointPositions(points2);
        ps2->addVectorQuantity("normals2", normals2);
        std::cout << "Random rotation:" << std::endl;
        for (int row = 0; row < 3; ++row) {
            std::cout << transform.rotation[0][row] << " "
                      << transform.rotation[1][row] << " "
                      << transform.rotation[2][row] << std::endl;
        }
        std::cout << "Random translation: " << transform.translation.x << ", "
                  << transform.translation.y << ", " << transform.translation.z << std::endl;
    }

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

    if (ImGui::Button("ICP Recalage")) {
        glm::mat3 rotation(1.0f);
        glm::vec3 translation(0.0f);

        icpRecalage(points1, points2, normals2, rotation, translation);

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

    // Eigen
    Eigen::VectorXd vector;


    // read PN
    // Load point sets
    std::vector<glm::vec3> points;
    std::vector<glm::vec3> normals;

    // readPNFile(std::string(TP3_Recalage_SOURCE_DIR) + "/data/pointsets/dino.pn", points, normals);
    readPNFile(std::string(TP3_Recalage_SOURCE_DIR) + "/data/pointsets/dino_subsampled_extreme.pn", points, normals);
    // readPNFile(std::string(TP3_Recalage_SOURCE_DIR) + "/data/pointsets/african_statue2_subsampled_extreme.pn", points, normals);
    // readPNFile(std::string(TP3_Recalage_SOURCE_DIR) + "/data/pointsets/african_statue2.pn", points, normals);
    // readPNFile(std::string(TP3_Recalage_SOURCE_DIR) + "/data/pointsets/african_statue_partial_1.pn", points, normals);

    auto ps = polyscope::registerPointCloud("input ps",points);
    ps->addVectorQuantity("normals", normals);
    ps->setPointRadius(0.001);
    ps->resetTransform();

    std::vector<glm::vec3> points2;
    std::vector<glm::vec3> normals2;

    // readPNFile(std::string(TP3_Recalage_SOURCE_DIR) + "/data/pointsets/dino2.pn", points2, normals2);
    readPNFile(std::string(TP3_Recalage_SOURCE_DIR) + "/data/pointsets/dino2_subsampled_extreme.pn", points2, normals2);
    // readPNFile(std::string(TP3_Recalage_SOURCE_DIR) + "/data/pointsets/african_statue2_subsampled_extreme.pn", points2, normals2);
    // readPNFile(std::string(TP3_Recalage_SOURCE_DIR) + "/data/pointsets/african_statue_partial_1.pn", points2, normals2);
    // readPNFile(std::string(TP3_Recalage_SOURCE_DIR) + "/data/pointsets/african_statue.pn", points2, normals2);

    auto ps2 = polyscope::registerPointCloud("input ps2",points2);
    ps2->addVectorQuantity("normals2", normals2);
    ps2->setPointRadius(0.001);
    ps2->resetTransform();


    // Add the callback
    polyscope::state::userCallback = [&points, &points2, &normals2, &ps, &ps2]() {
        callback(points, points2, normals2, ps, ps2);
    };

    // Show the gui
    polyscope::show();

    return 0;
}
