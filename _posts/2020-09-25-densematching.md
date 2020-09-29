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



## 4.2 Calculate Projection Matrix

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

```c++

	CdetectFeatures df;
	const int fcsize = 16;

	int detectLevel = 0; //detectLevel 越大，指的是降采样的图像上检测
	df.run(pss, 5, fcsize, detectLevel, 6);
```

LEVEL0：

![LEVEL0](https://pic.downk.cc/item/5f733c8b160a154a678b0e5d.jpg)

LEVEL1：















