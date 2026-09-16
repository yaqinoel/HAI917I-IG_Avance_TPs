#include <fstream>
#include <random>

#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "nanoflann.hpp"


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

std::vector<glm::vec3> generateRandomPointCloud(std::size_t pointNumber, float halfWidth) {
    if (pointNumber <= 0) {
        throw std::invalid_argument("Point number must be greater than zero");
    }
    std::mt19937 generator(42);
    std::uniform_real_distribution<float> distribution(-halfWidth, halfWidth);
    std::vector<glm::vec3> points(pointNumber);
    for (std::size_t i = 0; i < pointNumber; i++) {
        points[i].x = distribution(generator);
        points[i].y = distribution(generator);
        points[i].z = distribution(generator);
    }
    return points;
}

float computeWeight(float squaredDistance, float radius, int kernelType) {
    const float d = std::sqrt(squaredDistance);
    const float t = d / radius;

    switch (kernelType) {
        case 0: // Gaussien
            return std::exp(-t * t);
        case 1: // Wendland
            if (t >= 1.0f) {
                return 0.0f;
            }
            return std::pow(1.0f - t, 4.0f) * (1.0f + 4.0f * t);
        case 2: {
            constexpr float s = 2.0f;
            if (d < 1e-8f) {
                return std::numeric_limits<float>::max();
            }
            return std::pow(radius / d, s);
        }
        default:
            return 1.0f;
    }
}

void HPSS(glm::vec3 inputPoint,
    glm::vec3& outputPoint,
    glm::vec3& outputNormal,
    const std::vector<glm::vec3>& pointsSet,
    const std::vector<glm::vec3>& normalsSet,
    const KDTree& tree,
    const int kernerl_type,
    const float radius,
    const unsigned int nbIterations = 10,
    const unsigned int knn = 20
    ) {

    glm::vec3 currentPoint = inputPoint;
    glm::vec3 currentNormal(0.0f);

    // iteration
    for (int i = 0; i < nbIterations; ++i) {
        // 1) find knn points by KdTree
        std::vector<size_t> indices(knn);
        std::vector<float> distances(knn);
        nanoflann::KNNResultSet<float> resultSet(knn);
        resultSet.init(indices.data(), distances.data());
        tree.findNeighbors(resultSet, &currentPoint[0], nanoflann::SearchParameters());

        // 2) projet input point on the plan of the neighbor's plan
        const size_t neighborCount = resultSet.size();
        glm::vec3 avgPoint(0.0f);
        glm::vec3 avgNormal(0.0f);
        float totalWeight = 0.0f;

        std::vector<glm::vec3> projetedPoints(knn);
        std::vector<float> weights(knn);
        for (std::size_t i = 0; i < neighborCount; ++i) {
            const size_t neighborIndex = indices[i];

            const glm::vec3& p = pointsSet[neighborIndex];
            const glm::vec3& n = normalsSet[neighborIndex];

            projetedPoints[i] = currentPoint - glm::dot((currentPoint - p), n)* n;
            weights[i] = computeWeight(distances[i], radius, kernerl_type);

            avgPoint += weights[i] * projetedPoints[i];
            avgNormal += weights[i] * normalsSet[indices[i]];
            totalWeight += weights[i];
        }

        if (totalWeight <= 1e-8f) {
            break;
        }

        avgPoint /= totalWeight;
        avgNormal /= totalWeight;

        avgNormal = glm::normalize(avgNormal);

        currentNormal = avgNormal;
        currentPoint = avgPoint;
    }

    outputPoint = currentPoint;
    outputNormal = currentNormal;
}

void callback(const std::vector<glm::vec3>& points,
    const std::vector<glm::vec3>& normals,
    const KDTree& tree
) {
    ImGui::PushItemWidth(100);
    ImGuiIO &io = ImGui::GetIO();
    if (ImGui::Button("cool geometry process")) {
        constexpr unsigned int numPoints = 100;

        // 1) generate 100 random input points
        std::vector<glm::vec3> inputPoints = generateRandomPointCloud(numPoints, 4.0f);
        auto ps_input = polyscope::registerPointCloud("input points",inputPoints);
        ps_input->resetTransform();

        // 2) calculate output points and output normals by HPSS
        std::vector<glm::vec3> outputPoints(numPoints);
        std::vector<glm::vec3> outputNormals(numPoints);

        for (unsigned int i = 0; i < numPoints; i++) {
            HPSS(inputPoints[i], outputPoints[i], outputNormals[i], points, normals, tree, 0, 0.5f, 10, 20 );
        }

        // 3) draw projected points
        auto ps_output = polyscope::registerPointCloud("output points",outputPoints);
        ps_output->addVectorQuantity("output normals", outputNormals);
        ps_output->resetTransform();
    }

}



int main(int argc, char **argv) {

    // Options
    polyscope::options::autocenterStructures = true;
    polyscope::view::windowWidth = 1024;
    polyscope::view::windowHeight = 1024;

    // Initialize polyscope
    polyscope::init();


    std::vector<glm::vec3> points;
    std::vector<glm::vec3> normals;

    readPointCloud("../data/points/points_normals/bunny.xyz", points, normals);
    auto ps = polyscope::registerPointCloud("points set",points);
    ps->addVectorQuantity("normals", normals);
    ps->resetTransform();

    GLMVectorAdaptor pointsAdaptor(points);
    KDTree tree(3,pointsAdaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
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

    for (size_t j = 0; j < resultSet.size(); ++j)
    {
        size_t neighbor = indices[j];
        float dist2 = distances[j];

        std::cout << neighbor << "  " << dist2 << '\n';
    }
    // Add the callback
    polyscope::state::userCallback = [&points, &normals, &tree]() {
        callback(points, normals, tree);
    };

    // Show the gui
    polyscope::show();

    return 0;
}
