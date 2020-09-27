---
layout: post
title: dense matching
date: 2020-09-14
Author: jianping
categories: 
tags: [dense matching]
comments: false
---

# Contents
{:.no_toc}
* Will be replaced with the ToC, excluding the "Contents" header
{:toc}

# DAISY

## 1.parameters

![](https://pic.downk.cc/item/5f6dd29e160a154a67efc8c5.jpg)

15 -> rad �����ӵ����뾶

3 -> rad_q_no �뾶���򣬷�����

8 -> th_q_no һȦ8��

8 -> hist_th_q_no ÿ��Ȧ��8���ݶȷ���

```
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
   m_cube_size = m_layer_size*m_hist_th_q_no; //ԭʼͼƬ�˸�������ݶ�ͼ

   compute_cube_sigmas();
   compute_grid_points();

}
```
![](https://pic.downk.cc/item/5f6dd3a9160a154a67f00bf9.jpg)

cubeָ����G0

```

����ÿһ��bin������λ��
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

## 2.base functions

### layered_gradient

��ȡ�ݶȣ�8������

```

    float* gradient_layers = new float[h * w * 8];
    kutility::layered_gradient(img_data, h, w, 8, gradient_layers);

```

### compute_smoothed_gradient_layers

��μ����߶ȵ��ݶ�ͼ


### compute_histograms

ע�⣬�����м������ݶ�ͼ֮��ÿ�ξ�����൱�ھ����ڼ���histogram�������compute_histgram
ʵ����ֻ�Ǳ任�Ĵ洢λ�á�

### compute_histogram

����һ�����ص�hist

### get_descriptor

```
   /// computes the descriptor at homography-warped grid. (y,x) is not the
   /// coordinates of this image but the coordinates of the original grid where
   /// the homography will be applied. Meaning that the grid is somewhere else
   /// and we warp this grid with H and compute the descriptor on this warped
   /// grid; returns null/false if centers falls outside the image; allocate
   /// 'descriptor' memory first. descriptor is normalized.

   inline bool get_descriptor(double y, double x, int orientation, double* H, float* descriptor );

```


```
   //ʹ�õ��ĵ�Ӧ�任
   void point_transform_via_homography( double* H, double x, double y, double &u, double &v )
   {
      double kxp = H[0]*x + H[1]*y + H[2];
      double kyp = H[3]*x + H[4]*y + H[5];
      double kp  = H[6]*x + H[7]*y + H[8];
      u = kxp / kp;
      v = kyp / kp;
   }

```