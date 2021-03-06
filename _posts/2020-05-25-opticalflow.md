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
$ I_{x_n}　,  I_{y_n}$, 观测到了ｎ个光度差 : $ I_{t_n}$．
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

$E_{match}(w)$ 的意义是，对于图像中的某个位置$x$，已经有了特征点的匹配向量$w_1(x)$,
那么该点的光流$w(x)$　应当与　$w_1(x)$一致．$\delta(x)$ 就是判断是否有匹配（0 or 1）.
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

参考：　DeepMatching: Hierarchical Deformable Dense Matching

优质文章，后面仔细阅读．MARK

核心思想：利用深度网络的架构，实现稠密的匹配，主要分为两个关键步骤：

1. Bottom-up correlation pyramid computation

    自底部向上的相关金字塔建立。

    先将原始图片$I$划分为N*N的小patch,每一个小patch计算与$I^{'}$的相关图，相关系数的计算方式，和SIFT描述子相似。第一步的N很小，之后逐渐变大。

    ![](https://pic.downk.cc/item/5ecdd75ec2a9a83be51f2e11.png)

    多层次相关系数计算的示意图如下
    ![](https://pic.downk.cc/item/5ecdd75ec2a9a83be51f2e18.png)

    在最底层相关系数图计算完成后，可以通过aggragation的方式，生成高层次的相关系数图

    ![](https://pic.downk.cc/item/5ecdd75ec2a9a83be51f2e1c.png)

    这样就完成了相关金字塔的建立。

2. Top-down correspondence extraction
   
    在建立好相关金字塔后，需要通过自顶而下的方式，搜索每一个匹配的对应。大致步骤如下图所示。
    ![](https://pic.downk.cc/item/5ecdd998c2a9a83be521c5b0.png)


### 1.4.3 SED (Structured forests for fast edge detection)

参考：Structured forests for fast edge detection
代码：opencv 已经实现

SED 实现了对图像的边缘检测，在后续内插时，起到了保边的作用。

### 1.4.4 EpicFlow

现在可以继续关注光流这个重点了.

Epicflow的大部分步骤已经在上面介绍了.首先是利用DeepMatching实现两张图片的匹配,
之后利用SED检测图像边缘. 再进行保边内插生成稠密光流,如下图所示:

![](https://pic.downk.cc/item/5ece0c33c2a9a83be563dc69.png)


保边内插的设计,是很值得借鉴的,一个例子如下图:

![](https://pic.downk.cc/item/5ece0d42c2a9a83be565458e.png)

(b)是利用SED检测的边缘,而作者假设一张图片的边缘也是运动边界,那么内插的时候,不可以跨越边界. 利用SED的结果生成一个voronoi图,再利用类似图割的模式,仅仅对一个块内部的像素进行内插.


# 2. 深度学习

暂时不对深度学习的方法做分类,只是简单的罗列具有代表性的方法.

## 2.1 FlowNet

参考: FlowNet: Learning Optical Flow with Convolutional Networks

框架如下:

![](https://pic.downk.cc/item/5ece21dfc2a9a83be582547a.png)

![](https://pic.downk.cc/item/5ece2208c2a9a83be582793b.png)

框架比较简单,这里直接分析FlowNetCorr:

第一部分:通过卷积分别提取$I_1$ 和 $I_2$的特征

第二部分:计算局部特征差异,例如,$I_1$ 的 特征图 $f_1$ 和 $I_2$ 的 特征图 $f_2$,
对于$f_1$上的一个像素$x_1$,对应到$f_2$上的一个像素$x_2$,那么二者的特征差异计算
如下:

$$c(x1,x2)= \sum_{o\in [-k,k]\times[-k,k]}{<f_1(x_1+o),f_2(x_2+o)>}$$

其次,比较重要的是,为了节省计算量,$x_1$只会寻找到一定范围内$x_1$. 图上的441 = (21*21).

第三部分:继续将比较后的特征进行卷积.
第四部分:反卷积,生成高分辨率的光流.

不足:however, contains large errors in smooth background regions and requires
variational refinement

## 2.2 FlowNet 2.0

参考: FlowNet 2.0: Evolution of optical flow estimation with deep networks


框架如下:

![](https://pic.downk.cc/item/5ece3c55c2a9a83be5a353dc.png)

基本是网络叠加,先放一下


## 2.3 PWC-Net: CNNs for Optical Flow Using Pyramid,Warping, and Cost Volume

### 2.3.1 Cost volume
参考: Fast Cost-Volume Filtering for Visual Correspondence and Beyond

代价体积是视觉匹配中常用的概念, 以立体匹配为例, 下图绿色为示例的像素.(b)是通过局部相关计算出的代价. 其中红色为局部最优解.可以看到,由于噪声的原因,仅仅依靠局部相关计算的值误差很大. (c-e)是采用不同代价体积滤波后得到的结果.

**虽然flownet里没有说Cost volume, 但是实际也用了cost volume的计算方式**

![](https://pic.downk.cc/item/5ecf130fc2a9a83be5a79a80.png)


### 2.3.2 MULTI-SCALE CONTEXT AGGREGATION BY DILATED CONVOLUTIONS
参考: MULTI-SCALE CONTEXT AGGREGATION BY DILATED CONVOLUTIONS

文中用这个网络增大感受野,补一下.



### 2.3.3 PWC-NET

pwc-net 的框架如下,左边是传统coarse-to-fine方式

![](https://pic.downk.cc/item/5ecf28e4c2a9a83be5bd609b.png)

1. Feature pyramid extractor.
   
   类似flow-net,都先将原始图像的特征利用多层cnn进行提取

2. Warping layer.
   
   这一步是区别于flow-net的改进. 类比于传统coarse-to-fine,用低分辨率的光流对$I_2$进行warp,再进一步计算该层次的光流.只不过,在框架中warp的是特征.warp的方式如下:
   $$ c_{w}^{l}(x)=c_{2}^{l}(x+up_2(w^{l+1})(x))$$
   这个warp公式在文章中写的不是很规范,只能大概判断是那个意思,具体实现需要看代码.

3. Cost volume layer.
   
   cost volume layer用来存储匹配代价,单个特征的差异计算,都是如下公式:
   $$ cv^{l}(x_1,x_2) = \frac {1} {N} (c_{1}^{l}(x_1))^{\top}c_{w}^{l}(x_2) $$
   与flownet一样,为了节约计算量,cost volume 的搜索范围是一定的.

4. Optical flow estimator.
   
   为了计算光溜,这一步除了要输入代价体积之外,还要输入特征层. 其实也比较容易理解,是为了在估计最优值的时候估计原始图像的信息.(例如之前的传统方法EpicFlow中,利用图像的边缘信息进行平滑操作.)

5. Context network.

   这一层类比于传统方法在估计完光流后,在进行一次后处理.作者认为:Thus we
   employ a sub-network, called the context network, to effectively
   enlarge the receptive field size of each output unit at
   the desired pyramid level.为啥可以增大感受野,这个暂时不能理解.

## 2.4 MaskFlownet
参考: MaskFlownet: Asymmetric Feature Matching with Learnable Occlusion Mask

maskflownest 在求解光流场的时候,考虑了occlusion的影响,下面这个示意图可以很好的说明问题.

![](https://pic.downk.cc/item/5ecf7b87c2a9a83be52262d6.png)

上面这张图的前景部分运动的比较快,在warp之后留下可一个未知区域.由于upsampled后的初始光流场$\Phi$不会很准确,那么就会在warp后的图像中形成鬼影.这个问题目前还没有任何一个框架提及过.

下面分析maskflownet中最主要的创新.

![](https://pic.downk.cc/item/5ecf7890c2a9a83be51eabcd.png)

  (a) 是flownet类似的框架,$\Phi$ 是光流场初始值,$F^{l}(I_2)$经过warp后与$F^{l}(I_1)$计算代价体积.代价计算方式如下:

  $$ c(F^{l}(I_1),\Phi \circ F^{l}(I_2)) $$

  (b) OFMM框架,$\Phi$ 是光流场初始值,$\theta$ 和 $\mu$ 是 mask. mask添加的操作如下:

  $$ c(F^{l}(I_1),\Phi \circ F^{l}(I_2) \otimes \theta \oplus \mu) $$

  $\theta$ 和 $\mu$ 的加入基本是不加入训练负担的,是因为如果某一块区域是occlusion的话,那么它乘$\theta$基本是不变的,主要起作用的是$\mu$, 这样训练的结果就是, $\theta$ 在occlusion区域为0, 即指示为occlusion区域.

  (c) AsymOFMM框架. 作者指出,如果利用$\Phi$ 生成warp的影像,再和$F^{l}(I_1)$做相关处理,不如直接利用deformCNN,将$\Phi$ 作为偏移量加入deformCNN中. 这样是合理的,因为warp的处理方式还是很难以预测的,这样处理更加优雅,也使得相关处理变成对称的.


## 2.5 PPAC-HD3

参考: Probabilistic Pixel-Adaptive Refinement Networks

github: https://github.com/visinf/ppac_refinement



# 3. PWC-NET 代码详细


在windows上运行预先训练的网络结果如下.


![](https://pic.downk.cc/item/5ed3374dc2a9a83be5dd7318.png)

![](https://pic.downk.cc/item/5ed3376bc2a9a83be5dda55e.png)


对网络中间结果显示:

Extractor:

```python

netOne = torch.nn.Sequential(
                torch.nn.Conv2d(in_channels=3, out_channels=16, kernel_size=3, stride=2, padding=1),
                torch.nn.LeakyReLU(inplace=False, negative_slope=0.1),
                torch.nn.Conv2d(in_channels=16, out_channels=16, kernel_size=3, stride=1, padding=1),
                torch.nn.LeakyReLU(inplace=False, negative_slope=0.1),
                torch.nn.Conv2d(in_channels=16, out_channels=16, kernel_size=3, stride=1, padding=1),
                torch.nn.LeakyReLU(inplace=False, negative_slope=0.1)
            ).cuda().eval();

netTwo = torch.nn.Sequential(
                torch.nn.Conv2d(in_channels=16, out_channels=32, kernel_size=3, stride=2, padding=1),
                torch.nn.LeakyReLU(inplace=False, negative_slope=0.1),
                torch.nn.Conv2d(in_channels=32, out_channels=32, kernel_size=3, stride=1, padding=1),
                torch.nn.LeakyReLU(inplace=False, negative_slope=0.1),
                torch.nn.Conv2d(in_channels=32, out_channels=32, kernel_size=3, stride=1, padding=1),
                torch.nn.LeakyReLU(inplace=False, negative_slope=0.1)
            ).cuda().eval();

netThr = torch.nn.Sequential(
                torch.nn.Conv2d(in_channels=32, out_channels=64, kernel_size=3, stride=2, padding=1),
                torch.nn.LeakyReLU(inplace=False, negative_slope=0.1),
                torch.nn.Conv2d(in_channels=64, out_channels=64, kernel_size=3, stride=1, padding=1),
                torch.nn.LeakyReLU(inplace=False, negative_slope=0.1),
                torch.nn.Conv2d(in_channels=64, out_channels=64, kernel_size=3, stride=1, padding=1),
                torch.nn.LeakyReLU(inplace=False, negative_slope=0.1)
            ).cuda().eval();

netFou = torch.nn.Sequential(
                torch.nn.Conv2d(in_channels=64, out_channels=96, kernel_size=3, stride=2, padding=1),
                torch.nn.LeakyReLU(inplace=False, negative_slope=0.1),
                torch.nn.Conv2d(in_channels=96, out_channels=96, kernel_size=3, stride=1, padding=1),
                torch.nn.LeakyReLU(inplace=False, negative_slope=0.1),
                torch.nn.Conv2d(in_channels=96, out_channels=96, kernel_size=3, stride=1, padding=1),
                torch.nn.LeakyReLU(inplace=False, negative_slope=0.1)
            ).cuda().eval();

netFiv = torch.nn.Sequential(
                torch.nn.Conv2d(in_channels=96, out_channels=128, kernel_size=3, stride=2, padding=1),
                torch.nn.LeakyReLU(inplace=False, negative_slope=0.1),
                torch.nn.Conv2d(in_channels=128, out_channels=128, kernel_size=3, stride=1, padding=1),
                torch.nn.LeakyReLU(inplace=False, negative_slope=0.1),
                torch.nn.Conv2d(in_channels=128, out_channels=128, kernel_size=3, stride=1, padding=1),
                torch.nn.LeakyReLU(inplace=False, negative_slope=0.1)
            ).cuda().eval();

netSix = torch.nn.Sequential(
                torch.nn.Conv2d(in_channels=128, out_channels=196, kernel_size=3, stride=2, padding=1),
                torch.nn.LeakyReLU(inplace=False, negative_slope=0.1),
                torch.nn.Conv2d(in_channels=196, out_channels=196, kernel_size=3, stride=1, padding=1),
                torch.nn.LeakyReLU(inplace=False, negative_slope=0.1),
                torch.nn.Conv2d(in_channels=196, out_channels=196, kernel_size=3, stride=1, padding=1),
                torch.nn.LeakyReLU(inplace=False, negative_slope=0.1)
            ).cuda().eval();

```

![](https://pic.downk.cc/item/5ed8cd74c2a9a83be5458dbb.jpg)

![](https://pic.downk.cc/item/5ed8ce24c2a9a83be5464e2e.jpg)

![](https://pic.downk.cc/item/5ed8ce8cc2a9a83be546d089.jpg)

