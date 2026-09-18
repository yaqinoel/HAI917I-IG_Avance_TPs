# TP ICP

## 1 PCA（主成分分析）

对一个点云计算 PCA。

在点云的质心处显示主轴。

## 2 SVD（奇异值分解）

通过 SVD（交叉协方差矩阵）对两个点云进行配准。

## 3 ICP（迭代最近点）

使用 `dino` 和 `dino2` 进行测试。你观察到了什么？

使用 `dinosubsampledextreme` 和 `dino2subsampledextreme` 进行测试。你观察到了什么？

修改代码，使投影步骤能够将点投影到 HPSS 曲面上。讨论这种做法。

提出调整 `w` 的方法。

### 除刚体变换外相同的点集配准

使用 `african_statue2_subsampled_extreme.pn`。

为第二个实例定义一个随机旋转矩阵和一个随机平移。你观察到了什么？

### 不完整点集的配准

使用 `african_statue2.pn` 和 `african_statue_partial_1.pn`。

为第二个实例定义一个随机旋转矩阵和一个随机平移。你观察到了什么？

### 不一致点集的配准

使用 `african_statue_partial_1.pn` 和 `african_statue.pn`。

为第二个实例定义一个随机旋转矩阵和一个随机平移。

在这种情境下，解释问题所在，并提出解决方案。

# Eigen

```cpp
Eigen::Vector3d v;     // 三维 double 向量
Eigen::Vector3f vf;    // 三维 float 向量
Eigen::VectorXd v;     // 动态长度向量

Eigen::Vector3d v(1.0, 2.0, 3.0);
Eigen::Vector3d v;
v << 1.0, 2.0, 3.0;


Eigen::Matrix3d M;        // 3×3 double 矩阵
Eigen::Matrix3f Mf;       // 3×3 float 矩阵

Eigen::MatrixXd A;        // 动态 × 动态矩阵
Eigen::MatrixXd A(10, 3); // 10×3 矩阵

Eigen::Matrix<double, 3, 4> M34;

Eigen::Matrix3d M;

M << 1, 2, 3,
4, 5, 6,
7, 8, 9;
```

转置：

```cpp
Eigen::Matrix3d Rt = R.transpose();
```

平方距离：

```cpp
double d2 = (a - b).squaredNorm();
```

矩阵乘法：

```cpp
A * B
```

逐元素乘法：

```cpp
A.array() * B.array()
```

行与列：

```cpp
Eigen::MatrixXd A(N, 3);

Eigen::Vector3d p = A.row(i); // 严格说是从行隐式转换为列
Eigen::VectorXd x = A.col(0);
```

特征值 / 特征向量（适用于对称矩阵）：

```cpp
Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(C);
Eigen::Vector3d values = solver.eigenvalues();
Eigen::Matrix3d vectors = solver.eigenvectors();
```

\(C v_i = \lambda_i v_i\)

SVD（通用奇异值分解）：

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

\(A = U \Sigma V^T\)

随机旋转矩阵：

```cpp
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
```
