#include <fstream>
#include <random>

#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "nanoflann.hpp"
#include "grid.hpp"
#include "polyscope/surface_mesh.h"


using namespace nanoflann;

const std::array<glm::ivec3, 8> cornerOffsets = {{
    {0, 0, 0},
    {1, 0, 0},
    {0, 1, 0},
    {1, 1, 0},
    {0, 0, 1},
    {1, 0, 1},
    {0, 1, 1},
    {1, 1, 1}
}};

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

struct TriangleMesh {
    std::vector<glm::vec3> vertices;
    std::vector<std::array<size_t, 3>> triangles;
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
    const unsigned int knn = 20,
    glm::vec3* initialNormal = nullptr
    ) {

    glm::vec3 currentPoint = inputPoint;
    glm::vec3 currentNormal(0.0f);
    if (initialNormal != nullptr) {
        *initialNormal = glm::vec3(0.0f);
    }

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

        if (i == 0 && initialNormal != nullptr) {
            *initialNormal = avgNormal;
        }
        currentNormal = avgNormal;
        currentPoint = avgPoint;
    }

    outputPoint = currentPoint;
    outputNormal = currentNormal;
}

float implicitSDF(glm::vec3 x,
    const std::vector<glm::vec3>& pointsSet,
    const std::vector<glm::vec3>& normalsSet,
    const KDTree& tree,
    const int kernerl_type,
    const float radius,
    const unsigned int nbIterations = 10,
    const unsigned int knn = 20
) {
    glm::vec3 projetedPoint;
    glm::vec3 projectedNormal;
    glm::vec3 initialNormal;
    HPSS(x, projetedPoint, projectedNormal, pointsSet, normalsSet, tree,
         kernerl_type, radius, nbIterations, knn, &initialNormal);

    // Keep the normal estimated at x: the final normal may belong to a
    // different surface patch when a distant query moves during projection.
    return glm::dot(x - projetedPoint, initialNormal);
}

void addQuad(TriangleMesh& mesh, int a, int b, int c, int d, bool flip ) {
    if (a < 0 || b < 0 || c < 0 || d < 0) {
        return;
    }

    if (!flip) {
        mesh.triangles.push_back(
            std::array<size_t, 3>{
            static_cast<size_t>(a),
            static_cast<size_t>(b),
            static_cast<size_t>(c) });
        mesh.triangles.push_back(
            std::array<size_t, 3>{
            static_cast<size_t>(a),
            static_cast<size_t>(c),
            static_cast<size_t>(d) });
    } else {
        mesh.triangles.push_back(
            std::array<size_t, 3>{
            static_cast<size_t>(a),
            static_cast<size_t>(c),
            static_cast<size_t>(b) });
        mesh.triangles.push_back(
            std::array<size_t, 3>{
            static_cast<size_t>(a),
            static_cast<size_t>(d),
            static_cast<size_t>(c) });
    }
}

TriangleMesh DualContouring(Grid& grid,
    const std::vector<glm::vec3>& pointSet,
    const std::vector<glm::vec3>& normalsSet,
    const KDTree& tree
    ) {

    TriangleMesh mesh;
    // Keep iterative HPSS projection while evaluating the sign at each grid corner.
    constexpr int kernelType = 0; // Gaussian
    constexpr float radius = 0.5f;
    constexpr unsigned int sdfIterations = 10;
    constexpr unsigned int knn = 20;

    grid.cellVertexIds.assign(grid.cellResolution.x * grid.cellResolution.y * grid.cellResolution.z, -1);
    grid.sdfValues.assign((grid.cellResolution.x + 1) * (grid.cellResolution.y + 1) * (grid.cellResolution.z + 1), 0.0);

    // compute grid vertex SDF values
    for (int i = 0; i <= grid.cellResolution.x; ++i) {
        for (int j = 0; j <= grid.cellResolution.y; ++j) {
            for (int k = 0; k <= grid.cellResolution.z; ++k) {
                glm::vec3 point = grid.gridVertexPosition(i,j,k);
                float value = implicitSDF(point, pointSet, normalsSet, tree,
                                          kernelType, radius, sdfIterations, knn);
                std::size_t index = grid.gridVertexIndex(i,j,k);
                grid.sdfValues[index] = value;
            }
        }
    }

    for (int i = 0; i < grid.cellResolution.x; ++i) {
        for (int j = 0; j < grid.cellResolution.y; ++j) {
            for (int k = 0; k < grid.cellResolution.z; ++k) {
                bool hasNegative = false;
                bool hasPositive = false;

                for (const glm::ivec3& offset : cornerOffsets) {
                    int vi = i + offset.x;
                    int vj = j + offset.y;
                    int vk = k + offset.z;

                    std::size_t index = grid.gridVertexIndex(vi, vj, vk);
                    float value = grid.sdfValues[index];

                    if (value >= 0.0f) {
                        hasPositive = true;
                    } else {
                        hasNegative = true;
                    }
                }

                if (!hasNegative || !hasPositive) {
                    continue;
                }

                glm::vec3 cellCenter = grid.origin + grid.cellSize * (glm::vec3(i, j, k) + glm::vec3(0.5f));

                glm::vec3 cellVertex;
                glm::vec3 cellNormal;

                // TODO
                // use cell center as vertex
                // cellVertex = cellCenter;

                // TODO
                // projet cell center on MLS as vertex
                HPSS(cellCenter, cellVertex, cellNormal, pointSet, normalsSet, tree, kernelType, radius, 10, knn);

                const glm::vec3 cellMin = grid.gridVertexPosition(i,     j,     k);
                const glm::vec3 cellMax = grid.gridVertexPosition(i + 1, j + 1, k + 1);
                cellVertex = glm::clamp(cellVertex, cellMin, cellMax);

                const int vertexId = static_cast<int>(mesh.vertices.size());
                mesh.vertices.push_back(cellVertex);
                const std::size_t cellIndex = grid.cellVertexIndex(i, j, k);
                grid.cellVertexIds[cellIndex] = vertexId;
            }
        }
    }

    // Test draw vertex
    auto ps_output = polyscope::registerPointCloud("mesh vertex",mesh.vertices);
    ps_output->resetTransform();

    // create mesh by vertex
    const int nx = grid.cellResolution.x;
    const int ny = grid.cellResolution.y;
    const int nz = grid.cellResolution.z;

    // edge x
    for (int i = 0; i < nx; ++i) {
        for (int j = 1; j < ny; ++j) {
            for (int k = 1; k < nz; ++k) {
                std::size_t index0 = grid.gridVertexIndex(i, j, k);
                std::size_t index1 = grid.gridVertexIndex(i + 1, j, k);
                float value0 = grid.sdfValues[index0];
                float value1 = grid.sdfValues[index1];

                if ((value0 < 0.0f) == (value1 < 0.0f)) {
                    continue;
                }

                int cellVertexAIndex = grid.cellVertexIndex(i, j-1, k-1);
                int cellVertexBIndex = grid.cellVertexIndex(i, j-1, k);
                int cellVertexCIndex = grid.cellVertexIndex(i, j, k);
                int cellVertexDIndex = grid.cellVertexIndex(i, j, k-1);

                int vertexAId = grid.cellVertexIds[cellVertexAIndex];
                int vertexBId = grid.cellVertexIds[cellVertexBIndex];
                int vertexCId = grid.cellVertexIds[cellVertexCIndex];
                int vertexDId = grid.cellVertexIds[cellVertexDIndex];

                addQuad(mesh, vertexAId, vertexBId, vertexCId, vertexDId, value0 < 0.0f);
            }
        }
    }

    // edge y
    for (int i = 1; i < nx; ++i) {
        for (int j = 0; j < ny; ++j) {
            for (int k = 1; k < nz; ++k) {
                std::size_t index0 = grid.gridVertexIndex(i, j, k);
                std::size_t index1 = grid.gridVertexIndex(i, j + 1, k);
                float value0 = grid.sdfValues[index0];
                float value1 = grid.sdfValues[index1];

                if ((value0 < 0.0f) == (value1 < 0.0f)) {
                    continue;
                }

                int cellVertexAIndex = grid.cellVertexIndex(i-1, j, k-1);
                int cellVertexBIndex = grid.cellVertexIndex(i-1, j, k);
                int cellVertexCIndex = grid.cellVertexIndex(i, j, k);
                int cellVertexDIndex = grid.cellVertexIndex(i, j, k-1);

                int vertexAId = grid.cellVertexIds[cellVertexAIndex];
                int vertexBId = grid.cellVertexIds[cellVertexBIndex];
                int vertexCId = grid.cellVertexIds[cellVertexCIndex];
                int vertexDId = grid.cellVertexIds[cellVertexDIndex];

                addQuad(mesh, vertexAId, vertexBId, vertexCId, vertexDId, value0 >= 0.0f);
            }
        }
    }

    // edge z
    for (int i = 1; i < nx; ++i) {
        for (int j = 1; j < ny; ++j) {
            for (int k = 0; k < nz; ++k) {
                std::size_t index0 = grid.gridVertexIndex(i, j, k);
                std::size_t index1 = grid.gridVertexIndex(i, j, k + 1);
                float value0 = grid.sdfValues[index0];
                float value1 = grid.sdfValues[index1];

                if ((value0 < 0.0f) == (value1 < 0.0f)) {
                    continue;
                }

                int cellVertexAIndex = grid.cellVertexIndex(i-1, j-1, k);
                int cellVertexBIndex = grid.cellVertexIndex(i, j-1, k);
                int cellVertexCIndex = grid.cellVertexIndex(i, j, k);
                int cellVertexDIndex = grid.cellVertexIndex(i-1, j, k);

                int vertexAId = grid.cellVertexIds[cellVertexAIndex];
                int vertexBId = grid.cellVertexIds[cellVertexBIndex];
                int vertexCId = grid.cellVertexIds[cellVertexCIndex];
                int vertexDId = grid.cellVertexIds[cellVertexDIndex];

                addQuad(mesh, vertexAId, vertexBId, vertexCId, vertexDId, value0 >= 0.0f);
            }
        }
    }

    return mesh;
}


void callback(const std::vector<glm::vec3>& points,
    const std::vector<glm::vec3>& normals,
    const KDTree& tree
    ) {
    ImGui::PushItemWidth(100);
    ImGuiIO &io = ImGui::GetIO();
    if (ImGui::Button("HPSS")) {
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

    if (ImGui::Button("Dual Contouring")) {
        // create regular grid
        glm::ivec3 resolution(32,32,32);
        // glm::ivec3 resolution(64,64,64);
        // glm::ivec3 resolution(128,128,128);
        glm::vec3 gridSize(10.0, 10.0, 10.0);
        glm::vec3 cellSize(gridSize.x / resolution.x, gridSize.y / resolution.y, gridSize.z / resolution.z );

        glm::vec3 origin(-5.0, -5.0, -5.0);

        Grid grid(resolution, origin, cellSize);

        TriangleMesh mesh = DualContouring(grid, points, normals, tree);

        auto* psMesh = polyscope::registerSurfaceMesh( "dual contouring mesh", mesh.vertices, mesh.triangles );

        psMesh->setSmoothShade(true);
        psMesh->resetTransform();

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

    readPointCloud(
        std::string(TP2_Dual_Contouring_SOURCE_DIR) + "/data/points/points_normals/bunny.xyz",
        points,
        normals
    );
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
