# TP ICP


## 1  ACP
Calculer une ACP sur un nuage de points.
Affichez les axes principaux sur le centroid du nuage de point.

## 2  SVD
Faites un recalage entre 2 nuages de points par SVD (matrice de covariance croisée)

## 2  ICP

Testez sur dino et dino2. Que remarquez-vous ?

Testez sur dinosubsampledextreme et dino2subsampledextreme. Que remarquez-vous ?

Adaptez le code pour que l'étape de projection permette de projeter le point sur une surface HPSS. Discutez ce point.
Proposez des manières d'adapter w

### Recalage de pointssets identiques à une transformation rigide près
Utilisez african_statue2_subsampled_extreme.pn
En défissant une matrice de rotation aléatoire et une translation aléatoire pour la seconde instance
Qu'observez-vous ?

### Recalage de pointssets incomplets
Utilisez african_statue2.pn et african_statue_partial_1.pn
En défissant une matrice de rotation aléatoire et une translation aléatoire pour la seconde instance
Qu'observez-vous ?

### Recalage de pointssets inconsistents

Utilisez african_statue_partial_1.pn et african_statue.pn
En défissant une matrice de rotation aléatoire et une translation aléatoire pour la seconde instance

Dans ce contexte, expliquez le problème. Et proposez des solutions.


# Eigen
```cpp
Eigen::Vector3d v;     // 3D double
Eigen::Vector3f vf;    // 3D float
Eigen::VectorXd v;     // dynamic-size vector

Eigen::Vector3d v(1.0, 2.0, 3.0);
Eigen::Vector3d v;
v << 1.0, 2.0, 3.0;


Eigen::Matrix3d M;        // 3x3 double
Eigen::Matrix3f Mf;       // 3x3 float

Eigen::MatrixXd A;        // dynamic x dynamic
Eigen::MatrixXd A(10, 3); // 10x3

Eigen::Matrix<double, 3, 4> M34;

Eigen::Matrix3d M;

M << 1, 2, 3,
4, 5, 6,
7, 8, 9;
```
Transpose
```cpp
Eigen::Matrix3d Rt = R.transpose();
```
Squared distance:
```cpp
double d2 = (a - b).squaredNorm();
```
Matrix multiplication:
```cpp
A * B
```
Element-wise multiplication:
```cpp
A.array() * B.array()
```

Rows and columns:
```cpp
Eigen::MatrixXd A(N, 3);

Eigen::Vector3d p = A.row(i); // technically a row implicitely converted to column
Eigen::VectorXd x = A.col(0);

```
Eigenvalues / eigenvectors (For a symmetric matrix)
```cpp

Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(C);
Eigen::Vector3d values = solver.eigenvalues();
Eigen::Matrix3d vectors = solver.eigenvectors();
```
$ C v_i = \lambda_i v_i $

SVD  (General SVD):
```cpp
Eigen::JacobiSVD<Eigen::Matrix3d> svd(
A,
Eigen::ComputeFullU |
Eigen::ComputeFullV
);
Eigen::Matrix3d U = svd.matrixU();
Eigen::Matrix3d V = svd.matrixV();

Eigen::Vector3d S = svd.singularValues();


```
$ A = U \Sigma V^T $

Random rotation matrix

std::random_device rd;
std::mt19937 gen(rd());

std::normal_distribution<double> dist(0.0, 1.0);

Eigen::Quaterniond q(
dist(gen),
dist(gen),
dist(gen),
dist(gen)
);

q.normalize();

Eigen::Matrix3d R = q.toRotationMatrix();