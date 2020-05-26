---
layout: post
title: Optical flow
date: 2020-05-25
Author: jianping
categories: 
tags: [cv, learning]
comments: false
---

# Contents
{:.no_toc}

* Will be replaced with the ToC, excluding the "Contents" header
{:toc}


# 1. 传统方法

## 1.1 Lucas/Kanade (稀疏光流)

参考：Lucas/Kanade Meets Horn/Schunck: Combining Local and Global Optic Flow Methods

LK　算法是最经典的稀疏光流法

主要有三个假设构成：

(1) 图像局部连续.图像可以描述为一个连续函数，因此，一张图片可以描述为函数　$ I(x, y, t) $
其中ｔ为时间

(2) 光度一致性.
 
(3) 连续图像仅仅存在微小运动．

根据假设　(1,2,3) 可以的到以下微分方程：

$$ I(x, y, t) = I(x + \delta x,y + \delta y,t+ \delta t) $$


$$ I(x + \delta x,y + \delta y,t+\delta t) = 
I(x, y, t) + 
\frac {\partial I} {\partial x} \delta x + 
\frac {\partial I} {\partial y} \delta y + 
\frac {\partial I} {\partial t} \delta t $$

对于任意一个像素点，可以得到以下方程

$$ \frac {\partial I} {\partial x} \delta x + 
\frac {\partial I} {\partial y} \delta y 
 = - \frac {\partial I} {\partial t} \delta t $$
 
 
其中待解参数为　$ \delta x $ 和　$ \delta y $　（也就是光流）. 对于一个像素点，只可以列一个方程，
解不了两个参数．因此还需要假设局部（一个窗口）内的光流是一致的．假设窗口内有ｎ个像素，其对应梯度为：
$ I_{x_n}　,  I_{y_n}$　，　观测到了ｎ个光度差 : $ I_{t_n}$．
可以列出以下方程

$$
{\left[\begin{array}{cc}
  I_{x_1} & I_{x_2} & I_{x_3} & ... & I_{x_n}
 \\\ I_{y_1} & I_{y_2} & I_{y_3} & ... & I_{y_n}
\end{array}\right]}^\top
{\left[\begin{array}{cc}
  \delta y 
 \\\ \delta y
\end{array}\right]}
 = 
  - {\left[\begin{array}{c}
  I_{t_1} & I_{t_2} & I_{t_3} & ... & I_{t_n}
\end{array}\right]}^\top
$$


利用最小二乘法，可以解

$$ {\left[\begin{array}{cc}
               \delta x 
              \\ \delta y
             \end{array}\right]}$$


## 1.2 Horn/Schunck (稠密光流)

参考：Lucas/Kanade Meets Horn/Schunck: Combining Local and Global Optic Flow Methods

Horn/Schunck　在ＬＫ的技术上增加了局部平滑，整体解算．
 
$$ E_{HS}(\delta x,\delta y) = 
    \int {
    ((\frac {\partial I} {\partial x} \delta x + 
    \frac {\partial I} {\partial y} \delta y +
    \frac {\partial I} {\partial t} \delta t)^{2}+
    \alpha ((\nabla x)^{2} +  (\nabla y)^{2})dxdy)}$$

上述能量函数可以通过欧拉拉格朗日法求解．$ \alpha $ 越大,越能得到一个平滑的光流场

ＨＳ的效果受到局部噪声影响很大．


## 1.3 融入稀疏特征匹配的光流估计

参考：Large Displacement Optical Flow: Descriptor Matching in Variational Motion Estimation

从1.1和1.2的方法分析，我们可以知道，光流估计是需要高密度采样的，以保证两图片间亮度一致，且可微分．虽然
可以通过coarse-to-fine　采样的模式来降低高密度采样的要求，但是，仍然无法保证高速运动光流的估计．
因此一些学者继续在能量函数上添加了一系列约束条件，例如，将特征匹配列入能量函数中．至于加入特征匹配的优势，
这里不过分讨论，可以看论文．逼近2010年也是一个久远的年代了．

按照该文章的思路，重新整理光流估计问题：
记　$ w(x) = [u(x),v(x)]^{\top}$ 为待估计光流．　$\Omega$ 为积分范围，即整张图像．


首先，光度一致性的代价，可以列为：

$$  E_{color}(w) = \int _{\Omega} {(I_2(x+w(x))-I_1(x))^{2}}dx　$$

注意这里的公式，并不是微分形式，如果假设$I_2(x)$ 和　$I_1(x)$　连续，对$w$ 求偏导数即可变换为lk
算法的解算公式。

经过长期的研究（本文发表于2010年TPAMI）,学者发现，仅仅靠灰度一致性实现光流估计，很容易由于光强变化
产生错误．因为ＬＫ算法是假设灰度不变的，这个约束条件在现实场景中还是很难满足．可以通过将原始图片进行
边缘提取，再将边缘信息作为跟踪的代价．

$$  E_{grad}(w) = \int _{\Omega} { ( \nabla I_2(x+w(x)) - \nabla I_1(x)}) dx $$

这里有一点值得思考的是，其实我们可以发现　$ \nabla I_1(x) $ 本身就是一种特征提取，或者卷积的过程．
当年CNN没有用起来的时候，人们通过设计特征的方式提取适用于光流的特征．

在就是，对于稠密光流估计来说，一般都要加入一个平滑约束项，即认为相邻的像素光流接近．这个自Horn/Schunck
就一直延续, 不再详细讨论．

$$  E_{smooth}(w) = \int _{\Omega} { (\nabla u(x))^2 + (\nabla v(x))^2}dx $$

在融入特征描述之前，以上三项就可以进行光流求解了．下面介绍特征匹配的约束．

$$  E_{match}(w) = \int _\Omega {} { \delta(x) \rho(x) (w(x)-w_1(x)) }dx $$

$  E_{match}(w) $ 的意义是，对于图像中的某个位置$x$，已经有了特征点的匹配向量$w_1(x)$,
那么该点的光流$w(x)$　应当与　$w_1(x)$　一致．$\delta(x)$ 就是判断是否有匹配（0 or 1）.
$\rho(x)$ 是匹配的权重（好坏）．

除了匹配向量的约束外，作者还加入了特征距离约束．注意这里优化的是$w1(x)$，而不是$w(x)$
作者在文中有讨论原因．我认为，可以理解为特征点匹配
的好坏是很难量化的，且特征点的提取，本身也不存在严格的一一对应（类似于摄影测量中的空三，即便进行了
特征点匹配，为了提高精度，还是需要利用灰度相关，精化特征点的位置）. 因此，这一项类似一个对已有特征
的精化，看到这里，感到很亲切，但是这也注定该方法是实时不了了．

$$  E_{desc}(w_1) = \int _\Omega {} { \delta(x)  (f_2(x+w_1(x))-f_1(x))^{2} }dx$$

总体来说，可以把上面的约束放在一起，如下：

$$  E(w) = E_{color}(w) + E_{grad}(w) + E_{smooth}(w) + E_{match}(w) + E_{desc}(w_1)$$

这个方程的求解，需要给定一个很好的初始值，否则很难收敛．初始值的计算方式，可以利用特征匹配来获取．

## 1.4 融入稠密特征匹配的光流估计

参考：　EpicFlow: Edge-Preserving Interpolation of　Correspondences for Optical Flow

### 1.4.1 PatchMatch

EpicFlow中用到了deepMatching,先忽略．这里先介绍
PatchMatch (generized patch match).同样可以实现稠密匹配．

参考：　The Generalized PatchMatch Correspondence Algorithm

PatchMatch 完成的任务：　为影像$I_1$ 中的patch $x$　找到 影像$I_2$　中的 match $f(x)$

PatchMatch 中衡量两个patch 的相似程度方式就是直接相减，求绝对值的和．

PatchMatch 搜索分为propagate 和　随即搜索两个部分．

Propagate: 这种搜索模式的基本思想是，如果patch$x$找到了$f(x)$,那么很有可能patch$x+1$
对应的是match $f(x)+1$. 因此patch$x$的初始值由 $f(x-1)+1$得到，并由此向图像右下角搜索．
当完成一次完整的自左上到右下搜索(Propagate)后，再从右下到左上搜索．

Random search: Propagate 每次搜索范围较小，还需要给当前最优值附近一个合理的范围增大．

通过Propagate　和　Random search 两步来回迭代，实现patchmatch.


### 1.4.2 DeepMatching


### 1.4.3 SED (Structured forests for fast edge detection)



### 1.4.4 EpicFlow

现在可以继续关注光流这个重点了．




