---

layout: post
title: dense matching
date: 2020-09-28
Author: jianping
categories: 
tags: [dense matching]
comments: false
---

# Contents
{:.no_toc}
* Will be replaced with the ToC, excluding the "Contents" header
{:toc}

# 1 Parameters used in Aerial triangulation

## 1.1 camera parameters

```c++
        // camera
        std::map<std::string, size_t> img2pos; // image key -> image id
        std::map<size_t, std::string> pos2img; // image id -> image key
        std::vector<std::string> cams_filenames; 
        std::vector<Eigen::Matrix3d> cams_rotation; // 摄影测量常用的规范, 实际对应 R_c_M
        std::vector<Eigen::Matrix3d> cams_intrinsic; // K
        std::vector<Eigen::MatrixXd> cams_projection; // camera model m = K [R|-Rt] X
        std::vector<Eigen::Vector3d> cams_translation; // 摄影测量常用的规范, 实际对应 r_c_cM
        std::vector<Eigen::Vector3d> cams_radial_dist; // 径向畸变
        std::vector<Eigen::Vector2d> cams_tangential_dist; // 切向畸变

```

![](https://pic.downk.cc/item/5f717164160a154a67084c09.jpg)

## 1.2 camera EOPS (used in inertial navigation)

```c++
        std::vector<Eigen::Vector3d> r_M_c;
        std::vector<Eigen::Quaterniond> q_M_c;
        Eigen::Vector3d offset;
```

## 1.3 feature observations

```c++
		// feature observation
        std::map<std::string, std::list<unsigned int> > featuresPerCam; // 当前相机key -> list<特征点id>
        std::map<std::string, unsigned int> feat_key2id; // featurekey -> 特征点id
        std::map<unsigned int, std::string> feat_id2key; // 特征点id -> featurekey 
        std::map<unsigned int, bool> feat_valid;
        std::map<unsigned int, Eigen::Vector3d> feat_pos3D; // 特征点id , 坐标 
        std::vector<std::list<std::pair<size_t, Eigen::Vector2d> > > feat_observations; // 特征点id： <相机key, <2d 观测>>
```

# 2 Triangulation

##  2.1 Initial value estimation

注意，这里假设了观测是去除畸变的。对于空三来说，不去除畸变（未知的时候），先求出来一个初始值，也够BA用了。

对于特征点 $X$ 的一个位于相机 $P_1$ 上的观测 $x_1=[u_1,v_1,1]$



$$
x_1 \times P_{1} \cdot X = 0
$$



$$
\begin{bmatrix}
0 & -1 & v_1 \\
1 & 0 &  -u1 \\
-v_1 & u1 & 0
\end{bmatrix} \cdot 

\begin{bmatrix}
P^{1}_{1} \cdot X \\
P^{2}_{1} \cdot X \\
P^{3}_{1} \cdot X 
\end{bmatrix}
=0
$$


取前两行(第三行可以由前两行得到)，可得：



$$
- P^{2}_{1} \cdot X + v_1 \cdot P^{3}_{1} \cdot X = 0
$$




$$
P^{1}_{1} \cdot X - u_1 \cdot P^{3}_{1} \cdot X = 0
$$


整理一下


$$
\begin{bmatrix}
-P^{2}_{1}+v_1P^{3}_{1} \\
-P^{1}_{1}+u_1P^{3}_{1}
\end{bmatrix} \cdot 
X = 0
$$







$$
\begin{bmatrix}
0 & -1 & v_1 \\
-1 & 0 & u_1
\end{bmatrix} \cdot P_{1} \cdot X =0
$$

如果有n个观测，那么可以构造2*n个方程， 利用svd， 解算AtA.

## 2.2 Bundle adjustment

已知 外方位，内方位，前方点初始值后，放到BA里重新计算前方点。

当然，如果完全不知道各种初始值，就退化成SfM问题了，还得增量SfM，在这就不详细说了。

![](https://pic.downk.cc/item/5f71d2b9160a154a672b9b6d.jpg)



# 3 DAISY描述子

## 3.1 parameters

![](https://pic.downk.cc/item/5f6dd29e160a154a67efc8c5.jpg)

15 -> rad 描述子的最大半径

3 -> rad_q_no 半径方向，分三层

8 -> th_q_no 一圈8个

8 -> hist_th_q_no 每个圈有8个梯度方向

```c++
void daisy::set_parameters( double rad, int rad_q_no, int th_q_no, int hist_th_q_no )
{
   m_rad = rad;                   // radius of the descriptor at the initial scale
   m_rad_q_no = rad_q_no;         // how many pieces shall I divide the radial range ?
   m_th_q_no = th_q_no;           // how many pieces shall I divide the angular range  ?
   m_hist_th_q_no = hist_th_q_no; // how many pieces shall I divide the grad_hist
   m_grid_point_number = m_rad_q_no * m_th_q_no + 1; // +1 is for center pixel
   m_descriptor_size = m_grid_point_number * m_hist_th_q_no;

   for( int i=0; i<360; i++ )
   {
      m_orientation_shift_table[i] = i/360.0 * m_hist_th_q_no;
   }
   m_layer_size = m_h*m_w;
   m_cube_size = m_layer_size*m_hist_th_q_no; //原始图片八个方向的梯度图

   compute_cube_sigmas();
   compute_grid_points();

}
```
![](https://pic.downk.cc/item/5f6dd3a9160a154a67f00bf9.jpg)

cube指的是G0

```c++

计算每一个bin的中心位置
void daisy::compute_grid_points()
{
   double r_step = m_rad / m_rad_q_no;
   double t_step = 2*PI/ m_th_q_no;

   if( m_grid_points )
      deallocate( m_grid_points, m_grid_point_number );

   m_grid_points = allocate<double>(m_grid_point_number, 2);
   for( int y=0; y<m_grid_point_number; y++ )
   {
      m_grid_points[y][0] = 0;
      m_grid_points[y][1] = 0;
   }

   for( int r=0; r<m_rad_q_no; r++ )
   {
      int region = r*m_th_q_no+1;
      for( int t=0; t<m_th_q_no; t++ )
      {
         double y, x;
         polar2cartesian( (r+1)*r_step, t*t_step, y, x );
         m_grid_points[region+t][0] = y;
         m_grid_points[region+t][1] = x;
      }
   }
}

```

## 3.2 base functions

### layered_gradient

提取梯度，8个方向

```c++

    float* gradient_layers = new float[h * w * 8];
    kutility::layered_gradient(img_data, h, w, 8, gradient_layers);

```

### compute_smoothed_gradient_layers

逐次计算多尺度的梯度图


### compute_histograms

注意，代码中计算了梯度图之后，每次卷积，相当于就是在计算histogram，这里的compute_histgram
实际上只是变换的存储位置。

### compute_histogram

计算一个像素的hist

### get_descriptor

```c++
   /// computes the descriptor at homography-warped grid. (y,x) is not the
   /// coordinates of this image but the coordinates of the original grid where
   /// the homography will be applied. Meaning that the grid is somewhere else
   /// and we warp this grid with H and compute the descriptor on this warped
   /// grid; returns null/false if centers falls outside the image; allocate
   /// 'descriptor' memory first. descriptor is normalized.

   inline bool get_descriptor(double y, double x, int orientation, double* H, float* descriptor );

```


```c++
   //使用到的单应变换
   void point_transform_via_homography( double* H, double x, double y, double &u, double &v )
   {
      double kxp = H[0]*x + H[1]*y + H[2];
      double kyp = H[3]*x + H[4]*y + H[5];
      double kp  = H[6]*x + H[7]*y + H[8];
      u = kxp / kp;
      v = kyp / kp;
   }

```

# 4 Patch Based Stereo

## 4.1 Undistort Images 

Multiview stereo 一般是将相片去畸变之后再处理的。 因为采用Patch-based，可以不去做stereo-rectify





## 4.2 Calculate Projection Matrix

旋转矩阵和投影矩阵的特性：

1. 旋转矩阵  $R^{M}_c$  和一个相机系下向量 $[0,0,1]^{T}$  (也就是相机主轴)，那么 $R^{M}_c \cdot  [0,0,1]^{T} $ 就是选择了$R^{M}_c$ 的第三列。 也就是说  $R^{M}_c$ 的第三列为世界坐标系下的相机主光轴。

2.  $R^{c}_M$  的第三行为 世界坐标系下的相机主光轴，x轴和y轴分别是第一行和第二行

3. 从投影矩阵得到主光轴

   投影矩阵如下：

   


$$
P=K[R^{c}_M|r^{c}_{cM}], K=\begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y &  c_y \\
0 & 0 & 1
\end{bmatrix}
$$


​					注意第三行，因为 $K$ 的第三行是$ [0,0,1] $ , 所以，$ P $ 的第三行的前三个元素就是世界坐标系下的相机主光轴

4. 从投影矩阵得到x轴和y轴

   

   

   
   $$
   R^{c}_M=\begin{bmatrix}
   R_1 \\
   R_2 \\
   R_3
   \end{bmatrix},P_{3\times4}=K\cdot [R^{c}_M|r^{c}_{cM}]=\begin{bmatrix}
   f_x & 0 & c_x \\
   0 & f_y &  c_y \\
   0 & 0 & 1
   \end{bmatrix} \cdot
   \begin{bmatrix}
   R_1 & t1 \\
   R_2 & t_2 \\
   R_3 & t_3
   \end{bmatrix}
   =
   \begin{bmatrix}
   f_xR_1 + c_x R_3 & f_x t_1 + c_x t_3\\
   f_yR_2 + c_y R_3 & f_y t_2 + c_y t_3\\
   R_3 & t_3
   \end{bmatrix}
   $$
   

   
   $$
   P_{1\times3}(0,0) \times P_{1\times3}(2,0) =
   f_x[R_3]_{\times}\cdot R_1^{T} + c_x[R_3]_{\times} \cdot R_3^{T}=f_x[R_3]_{\times}\cdot R_1^{T}
   $$
   

   

   也就是说，把第一行和第三行叉乘，可以得到y轴向量，注意要除以模长。得到了z轴和y轴，两者叉乘就得到了x轴。

   

5. 从投影矩阵得到相机中心

   这里验证结论，

   

   
   $$
   r^{M}_c = - P_{3\times3}(0,0)^{-1} \cdot P_{3\times1}(0,3)
   $$
   

   下面验证：

   

$$
P_{3\times3}(0,0)^{-1} = (K \cdot R^{c}_M)^{-1} = R^{M}_c \cdot K^{-1} = R^{M}_c \cdot \begin{bmatrix}
1/f_x & 0 & -c_x/f_x \\
0 & 1/f_x  & -c_y/f_y \\
0 & 0 & 1
\end{bmatrix}
$$


$$
- P_{3\times3}(0,0)^{-1} \cdot P_{3\times1}(0,3)=R^{M}_c \cdot \begin{bmatrix}
t_1\\
t_2 \\
t_3
\end{bmatrix}
= r^{M}_c
$$


 Projection file 保存如下格式：



```
CONTOUR // header
2563.3785486 2002.09943502 175.909287694 -2576.79680462 
1227.12336166 -309.431464381 2694.1963878 -1537.18385803 
0.965449284333 -0.244965622232 0.088885968683 -0.973456905582 // Pojection Matrix
```

## 4.3 Group File

```
SKE // header
7 1 
7 0
0 1 2 3 4 5 6 
```

## 4.4 Vis File

```
VISDATA // header
7 // 照片总数
0 4  3 4 2 1 // 照片ID 邻接个数 邻接照片ID1 邻接照片ID2 邻接照片ID3
1 6  6 3 2 5 4 0 
2 6  4 5 1 3 0 6 
3 6  0 1 4 2 5 6 
4 6  2 0 3 1 5 6 
5 5  2 6 1 3 4 
6 5  1 5 2 3 4 
```

## 4.5 Image loader

```c++
	Image::Cphoto photo;
	photo.init(name, mname, ename, cname, 3); // 图片名，mask名，edge名，camera名，尺度
	photo.alloc(); // alloc 之后才会加载图片，对应free

	std::vector<unsigned char>& imagedata0 = photo.getImage(0); // 加载不同尺度，存储方式为RGB，RGB
	std::vector<unsigned char>& imagedata1 = photo.getImage(1); 

	Image::CphotoSetS pss; // photo 的集合
	std::vector<int>images;
	for (int i = 0; i < 5; i++)
	{
		images.push_back(i);
	}
	int level = 3;
	int windowSize = 7;
	pss.init(images, prefix, level, windowSize, 1); // 初始化一个photo集合

```

## 4.6 Multi-thread

对于特征提取这种步骤，应该用多线程去并行，一个小例子

```c++
#include "tinycthread/tinycthread.h"
#include "tinycthread/rwmutex.h"

using namespace std;
mtx_t m_rwlock;
static int count_i = 0;

int thread_func(void * argc)
{
	int * threadId = (int*)argc;
	for (int i = 0; i < 1000; i++)
	{
		mtx_lock(&m_rwlock);
		count_i = count_i + 1;
		std::cout << "hello: " << count_i << " from thread "<< *threadId <<"\n";
		mtx_unlock(&m_rwlock);
	}	
	return 0;
}

int main(int argc, char** argv)
{
	int m_CPU = 6;
	mtx_init(&m_rwlock, mtx_plain | mtx_recursive);
	vector<thrd_t> threads(m_CPU);
	for (int i = 0; i < m_CPU; ++i)
		thrd_create(&threads[i], thread_func, (void*)(&i));
	for (int i = 0; i < m_CPU; ++i)
		thrd_join(threads[i], NULL);

	mtx_destroy(&m_rwlock);
}
```

## 4.7 Harris & DOG Feature detector



初始化PATCH需要先在图像上检测一些角点，或者差分金字塔（DOG）响应点。

```c++

	CdetectFeatures df;
	const int fcsize = 16;

	int detectLevel = 0; //detectLevel 越大，指的是降采样的图像上检测
	df.run(pss, 5, fcsize, detectLevel, 6);

```

LEVEL0：

![LEVEL0](https://pic.downk.cc/item/5f733c8b160a154a678b0e5d.jpg)

LEVEL1：

![](https://pic.downk.cc/item/5f733d91160a154a678b63a5.jpg)





## 4.8 Patch Initialization (seed generation)



### 4.8.1 initialMatch  



### 4.8.2 collectCandidates

```c++

void collectCandidates(const int index, const std::vector<int>& indexes,
                         const Cpoint& point, std::vector<Ppoint>& vcp); 


///  reference image id
///  target image ids
///  reference point
///  candidate points
```

由相机投影矩阵确定F：

```c++

template<class T>
void setF(const Image::Ccamera& lhs, const Image::Ccamera& rhs,
	  TMat3<T>& F, const int level = 0) {
  const TVec4<T>& p00 = lhs.m_projection[level][0];
  const TVec4<T>& p01 = lhs.m_projection[level][1];
  const TVec4<T>& p02 = lhs.m_projection[level][2];

  const TVec4<T>& p10 = rhs.m_projection[level][0];
  const TVec4<T>& p11 = rhs.m_projection[level][1];
  const TVec4<T>& p12 = rhs.m_projection[level][2];

  F[0][0] = det(TMat4<T>(p01, p02, p11, p12));
  F[0][1] = det(TMat4<T>(p01, p02, p12, p10));
  F[0][2] = det(TMat4<T>(p01, p02, p10, p11));

  F[1][0] = det(TMat4<T>(p02, p00, p11, p12));
  F[1][1] = det(TMat4<T>(p02, p00, p12, p10));
  F[1][2] = det(TMat4<T>(p02, p00, p10, p11));

  F[2][0] = det(TMat4<T>(p00, p01, p11, p12));
  F[2][1] = det(TMat4<T>(p00, p01, p12, p10));
  F[2][2] = det(TMat4<T>(p00, p01, p10, p11));
 };

```

推导下由 $P_1$ ,  $P_2$ ,  得到 $ F $ , 常见公式如下： 而上面的公式，暂时打个问号。

![](https://pic.downk.cc/item/5f7ac986160a154a679fdf85.jpg)

collectCandidates 完成了由参考影像上某一个特征点，寻找满足F矩阵关系的其他影像上的特征点。

```c++

// make sorted array of feature points in images, that satisfy the
// epipolar geometry coming from point in image
void Cseed::collectCandidates(const int index, const std::vector<int>& indexes,
                              const Cpoint& point, std::vector<Ppoint>& vcp) {
  const Vec3 p0(point.m_icoord[0], point.m_icoord[1], 1.0);
  for (int i = 0; i < (int)indexes.size(); ++i) {        
    const int indexid = indexes[i];
    
    vector<TVec2<int> > cells;
    collectCells(index, indexid, point, cells);
    Mat3 F;
    Image::setF(m_fm.m_pss.m_photos[index], m_fm.m_pss.m_photos[indexid],
                F, m_fm.m_level);
    
    for (int i = 0; i < (int)cells.size(); ++i) {
      const int x = cells[i][0];      const int y = cells[i][1];
      if (!canAdd(indexid, x, y))
	continue;
      const int index2 = y * m_fm.m_pos.m_gwidths[indexid] + x;

      vector<Ppoint>::iterator begin = m_ppoints[indexid][index2].begin();
      vector<Ppoint>::iterator end = m_ppoints[indexid][index2].end();
      while (begin != end) {
        Cpoint& rhs = **begin;
        // ? use type to reject candidates?
        if (point.m_type != rhs.m_type) {
          ++begin;
          continue;
        }
          
        const Vec3 p1(rhs.m_icoord[0], rhs.m_icoord[1], 1.0);
        if (m_fm.m_epThreshold <= Image::computeEPD(F, p0, p1)) {
          ++begin;          
          continue;
        }
        vcp.push_back(*begin);
        ++begin;
      }
    }
  }
  
  // set distances to m_response
  vector<Ppoint> vcptmp;
  for (int i = 0; i < (int)vcp.size(); ++i) {
    unproject(index, vcp[i]->m_itmp, point, *vcp[i], vcp[i]->m_coord);
    
    if (m_fm.m_pss.m_photos[index].m_projection[m_fm.m_level][2] *
        vcp[i]->m_coord <= 0.0)
      continue;

    if (m_fm.m_pss.getMask(vcp[i]->m_coord, m_fm.m_level) == 0 ||
        m_fm.insideBimages(vcp[i]->m_coord) == 0)
      continue;

    //??? from the closest
    vcp[i]->m_response =
      fabs(norm(vcp[i]->m_coord - m_fm.m_pss.m_photos[index].m_center) -
           norm(vcp[i]->m_coord - m_fm.m_pss.m_photos[vcp[i]->m_itmp].m_center));
    
    vcptmp.push_back(vcp[i]);
  }
  vcptmp.swap(vcp);
  sort(vcp.begin(), vcp.end());
}
```

collectCells 的作用是先把极线上的cell给取出来

 ```c++

void Cseed::collectCells(const int index0, const int index1,
                         const Cpoint& p0, std::vector<Vec2i>& cells) {
  Vec3 point(p0.m_icoord[0], p0.m_icoord[1], p0.m_icoord[2]);
#ifdef DEBUG
  if (p0.m_icoord[2] != 1.0f) {
    cerr << "Impossible in collectCells" << endl;    exit (1);
  }
#endif
  
  Mat3 F;
  Image::setF(m_fm.m_pss.m_photos[index0], m_fm.m_pss.m_photos[index1],
              F, m_fm.m_level);
  const int gwidth = m_fm.m_pos.m_gwidths[index1];
  const int gheight = m_fm.m_pos.m_gheights[index1];
  
  Vec3 line = transpose(F) * point;
  if (line[0] == 0.0 && line[1] == 0.0) {
    cerr << "Point right on top of the epipole?"
         << index0 << ' ' << index1 << endl;
    return;
  }
  // vertical
  if (fabs(line[0]) > fabs(line[1])) {
    for (int y = 0; y < gheight; ++y) {
      const float fy = (y + 0.5) * m_fm.m_csize - 0.5f;
      float fx = (- line[1] * fy - line[2]) / line[0];
      fx = max((float)(INT_MIN + 3.0f), std::min((float)(INT_MAX - 3.0f), fx));
      
      const int ix = ((int)floor(fx + 0.5f)) / m_fm.m_csize;
      if (0 <= ix && ix < gwidth)
        cells.push_back(TVec2<int>(ix, y));
      if (0 <= ix - 1 && ix - 1 < gwidth)
        cells.push_back(TVec2<int>(ix - 1, y));
      if (0 <= ix + 1 && ix + 1 < gwidth)
        cells.push_back(TVec2<int>(ix + 1, y));
    }
  }
  else {
    for (int x = 0; x < gwidth; ++x) {
      const float fx = (x + 0.5) * m_fm.m_csize - 0.5f;
      float fy = (- line[0] * fx - line[2]) / line[1];
      fy = max((float)(INT_MIN + 3.0f), std::min((float)(INT_MAX - 3.0f), fy));
      
      const int iy = ((int)floor(fy + 0.5f)) / m_fm.m_csize;
      if (0 <= iy && iy < gheight)
        cells.push_back(TVec2<int>(x, iy));
      if (0 <= iy - 1 && iy - 1 < gheight)
        cells.push_back(TVec2<int>(x, iy - 1));
      if (0 <= iy + 1 && iy + 1 < gheight)
        cells.push_back(TVec2<int>(x, iy + 1));
    }
  }
}
 ```







## 4.9 No-linear Optimization

对于带边界条件，代价函数导数无法确定的优化问题，需要特殊的方式求解，这个与LM+最小二乘不一致。

### 4.9.1 An example


$$
min f= (x_2)^{1/2} \\
s.t. x_2 \ge 0 \\
s.t. x_2 \ge (a_1 x_1+b_1)^{3}, a_1=2,b_1=0\\
s.t. x_2 \ge (a_2 x_1+b_2)^{3}, a_2=-1,b_2=1 \\
$$

 

优化问题的可行域如下：



![](https://pic.downk.cc/item/5f798ec2160a154a6752fd05.jpg)

定义代价函数和约束条件：

```c++

double myfunc(unsigned n, const double* x, double* grad, void* my_func_data)
{
    if (grad) {
        grad[0] = 0.0;
        grad[1] = 0.5 / sqrt(x[1]);
    }
    return sqrt(x[1]);
}

typedef struct {
    double a, b;
} my_constraint_data;

double myconstraint(unsigned n, const double* x, double* grad, void* data)
{
    my_constraint_data* d = (my_constraint_data*)data;
    double a = d->a, b = d->b;
    if (grad) {
        grad[0] = 3 * a * (a * x[0] + b) * (a * x[0] + b);
        grad[1] = -1.0;
    }
    return ((a * x[0] + b) * (a * x[0] + b) * (a * x[0] + b) - x[1]);
}

```

grad 也可以不管，对于不需要梯度的算法

对于约束条件，默认为fx <= 0, 因此做以下调整：


$$
s.t. x_2 \ge (a_1 x_1+b_1)^{3}, a_1=2,b_1=0\\
==> 0 \ge (a_1 x_1+b_1)^{3}-x_2
$$


```c++

int main() {
    double lb[2] = { -HUGE_VAL, 0 }; /* lower bounds */
    double ub[2] = { HUGE_VAL, HUGE_VAL }; /* lower bounds */
    nlopt_opt opt;

    opt = nlopt_create(NLOPT_LD_MMA, 2); /* algorithm and dimensionality */
    nlopt_set_lower_bounds(opt, lb);
    nlopt_set_upper_bounds(opt, ub);
    nlopt_set_min_objective(opt, myfunc, NULL);

    my_constraint_data data[2] = { {2,0}, {-1,1} };

    nlopt_add_inequality_constraint(opt, myconstraint, &data[0], 1e-8);
    nlopt_add_inequality_constraint(opt, myconstraint, &data[1], 1e-8);

    nlopt_set_xtol_rel(opt, 1e-4);

    double x[2] = { 1.234, 5.678 };  /* `*`some` `initial` `guess`*` */
    double minf; /* `*`the` `minimum` `objective` `value,` `upon` `return`*` */
    if (nlopt_optimize(opt, x, &minf) < 0) {
        printf("nlopt failed!\n");
    }
    else {
        printf("found minimum at f(%g,%g) = %0.10g\n", x[0], x[1], minf);
    }

    nlopt_destroy(opt);
}


```

### 4.9.2 Patch Optimizer



待优化参数的参数化， coord-> 3个参数，norm->3个参数，为了降低搜索空间（其实和李群李代数一个道理），把待解算参数空间缩小到3个（一个深度，航向，俯仰）



```c++

void Coptim::encode(const Vec4f& coord,  // 深度，注意是m_raysT是参考相机观测到patch的光线
		    double* const vect, const int id) const {
  vect[0] = (coord - m_centersT[id]) * m_raysT[id] / m_dscalesT[id];
}

void Coptim::encode(const Vec4f& coord, const Vec4f& normal,
		    double* const vect, const int id) const {
  encode(coord, vect, id);
  
  const int image = m_indexesT[id][0];
  const float fx = m_xaxes[image] * proj(normal); // projects from 4D to 3D, divide by last value
  const float fy = m_yaxes[image] * proj(normal);
  const float fz = m_zaxes[image] * proj(normal); // 计算法向量到相机三个轴的投影

  vect[2] = asin(max(-1.0f, min(1.0f, fy))); // 可以理解为计算经纬度
  const float cosb = cos(vect[2]);

  if (cosb == 0.0)
    vect[1] = 0.0;
  else {
    const float sina = fx / cosb;
    const float cosa = - fz / cosb;
    vect[1] = acos(max(-1.0f, min(1.0f, cosa)));
    if (sina < 0.0)
      vect[1] = - vect[1];
  }

  vect[1] = vect[1] / m_ascalesT[id];
  vect[2] = vect[2] / m_ascalesT[id];  
}

void Coptim::decode(Vec4f& coord, Vec4f& normal,
		    const double* const vect, const int id) const {
  decode(coord, vect, id);
  const int image = m_indexesT[id][0];

  const float angle1 = vect[1] * m_ascalesT[id];
  const float angle2 = vect[2] * m_ascalesT[id];

  const float fx = sin(angle1) * cos(angle2);
  const float fy = sin(angle2);
  const float fz = - cos(angle1) * cos(angle2);

  Vec3f ftmp = m_xaxes[image] * fx + m_yaxes[image] * fy + m_zaxes[image] * fz;
  normal = Vec4f(ftmp[0], ftmp[1], ftmp[2], 0.0f);
}

void Coptim::decode(Vec4f& coord, const double* const vect, const int id) const {
  coord = m_centersT[id] + m_dscalesT[id] * vect[0] * m_raysT[id];
}
```



```c++


//----------------------------------------------------------------------
// BFGS functions
//----------------------------------------------------------------------

double Coptim::my_f(unsigned n, const double *x, double *grad, void *my_func_data)
{
  double xs[3] = {x[0], x[1], x[2]};
  const int id = *((int*)my_func_data);

  const float angle1 = xs[1] * m_one->m_ascalesT[id];
  const float angle2 = xs[2] * m_one->m_ascalesT[id];

  double ret = 0.0;

  //?????
  const double bias = 0.0f;//2.0 - exp(- angle1 * angle1 / sigma2) - exp(- angle2 * angle2 / sigma2);
  
  Vec4f coord, normal;
  m_one->decode(coord, normal, xs, id);
  
  const int index = m_one->m_indexesT[id][0];
  Vec4f pxaxis, pyaxis;
  m_one->getPAxes(index, coord, normal, pxaxis, pyaxis);
  
  const int size = min(m_one->m_fm.m_tau, (int)m_one->m_indexesT[id].size());
  const int mininum = min(m_one->m_fm.m_minImageNumThreshold, size);

  for (int i = 0; i < size; ++i) {
    int flag;
    flag = m_one->grabTex(coord, pxaxis, pyaxis, normal, m_one->m_indexesT[id][i],
                          m_one->m_fm.m_wsize, m_one->m_texsT[id][i]);

    if (flag == 0)
      m_one->normalize(m_one->m_texsT[id][i]);
  }

  const int pairwise = 0;
  if (pairwise) {
    double ans = 0.0f;
    int denom = 0;
    for (int i = 0; i < size; ++i) {
      for (int j = i+1; j < size; ++j) {
        if (m_one->m_texsT[id][i].empty() || m_one->m_texsT[id][j].empty())
          continue;
        
        ans += robustincc(1.0 - m_one->dot(m_one->m_texsT[id][i], m_one->m_texsT[id][j]));
        denom++;
      }
    }
    if (denom <
        //m_one->m_fm.m_minImageNumThreshold *
        //(m_one->m_fm.m_minImageNumThreshold - 1) / 2)
        mininum * (mininum - 1) / 2)
      ret = 2.0f;
    else
      ret = ans / denom + bias;
  }
  else {
    if (m_one->m_texsT[id][0].empty())
      return 2.0;
      
    double ans = 0.0f;
    int denom = 0;
    for (int i = 1; i < size; ++i) {
      if (m_one->m_texsT[id][i].empty())
        continue;
      ans +=
        robustincc(1.0 - m_one->dot(m_one->m_texsT[id][0], m_one->m_texsT[id][i]));
      denom++;
    }
    //if (denom < m_one->m_fm.m_minImageNumThreshold - 1)
    if (denom < mininum - 1)
      ret = 2.0f;
    else
      ret = ans / denom + bias;
  }

  return ret;
}


```

得到patch的x和y轴，注意，x和y轴是带尺度的，相对于image[index], 单位pxaxis刚好投影后差一个像素



```c++

// get x and y axis to collect textures given reference image and normal
void Coptim::getPAxes(const int index, const Vec4f& coord, const Vec4f& normal,
		      Vec4f& pxaxis, Vec4f& pyaxis) const{  
  // yasu changed here for fpmvs
  const float pscale = getUnit(index, coord);

  Vec3f normal3(normal[0], normal[1], normal[2]);
  Vec3f yaxis3 = cross(normal3, m_xaxes[index]);
  unitize(yaxis3);
  Vec3f xaxis3 = cross(yaxis3, normal3);
  pxaxis[0] = xaxis3[0];  pxaxis[1] = xaxis3[1];  pxaxis[2] = xaxis3[2];  pxaxis[3] = 0.0;
  pyaxis[0] = yaxis3[0];  pyaxis[1] = yaxis3[1];  pyaxis[2] = yaxis3[2];  pyaxis[3] = 0.0;

  pxaxis *= pscale;
  pyaxis *= pscale;
  const float xdis = norm(m_fm.m_pss.project(index, coord + pxaxis, m_fm.m_level) -
                          m_fm.m_pss.project(index, coord, m_fm.m_level));
  const float ydis = norm(m_fm.m_pss.project(index, coord + pyaxis, m_fm.m_level) -
                          m_fm.m_pss.project(index, coord, m_fm.m_level));
  pxaxis /= xdis;
  pyaxis /= ydis;
}

```

获取patch纹理信息：

输入是patch 的coord，三个方向向量：pxaxis, pyaxis, pzaxis， 注意pxaxis, pyaxis是和当前相机的x，y轴对齐的

```c++

int Coptim::grabTex(const Vec4f& coord, const Vec4f& pxaxis, const Vec4f& pyaxis,
		    const Vec4f& pzaxis, const int index, const int size,
                    std::vector<float>& tex) const {
  tex.clear();

  Vec4f ray = m_fm.m_pss.m_photos[index].m_center - coord;
  unitize(ray);
  const float weight = max(0.0f, ray * pzaxis);

  //???????
  //if (weight < cos(m_fm.m_angleThreshold0))
  if (weight < cos(m_fm.m_angleThreshold1))
    return 1;

  const int margin = size / 2;

  Vec3f center = m_fm.m_pss.project(index, coord, m_fm.m_level);  
  Vec3f dx = m_fm.m_pss.project(index, coord + pxaxis, m_fm.m_level) - center;
  Vec3f dy = m_fm.m_pss.project(index, coord + pyaxis, m_fm.m_level) - center;
  
  const float ratio = (norm(dx) + norm(dy)) / 2.0f;
  //int leveldif = (int)floor(log(ratio) / log(2.0f) + 0.5f);
  int leveldif = (int)floor(log(ratio) / Log2 + 0.5f);

  // Upper limit is 2
  leveldif = max(-m_fm.m_level, min(2, leveldif));

  //const float scale = pow(2.0f, (float)leveldif);

  const float scale = MyPow2(leveldif);
  const int newlevel = m_fm.m_level + leveldif; // 这里的采样方式是先确定尺度，再把dx，dy，center归一化

  center /= scale;  dx /= scale;  dy /= scale;
  
  if (grabSafe(index, size, center, dx, dy, newlevel) == 0) // 检查投影的patch最大最小值是否在图像范围内
    return 1;
  
  Vec3f left = center - dx * margin - dy * margin;

  tex.resize(3 * size * size);
  float* texp = &tex[0] - 1;
  for (int y = 0; y < size; ++y) {
    Vec3f vftmp = left;
    left += dy;
    for (int x = 0; x < size; ++x) {
      Vec3f color = m_fm.m_pss.getColor(index, vftmp[0], vftmp[1], newlevel);
      *(++texp) = color[0];
      *(++texp) = color[1];
      *(++texp) = color[2];
      vftmp += dx;
    }
  }
  
  return 0;
}


```













