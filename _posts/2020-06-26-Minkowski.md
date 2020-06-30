---
layout: post
title: Minkowski 卷积网络
date: 2020-06-26
Author: jianping
categories: 
tags: [deep learning]
comments: false
---

# Contents
{:.no_toc}
* Will be replaced with the ToC, excluding the "Contents" header
{:toc}

# 1. 以二分类开始

二分类问题在计算机视觉中属于一个常见的问题. 在几何问题中的ransac方法,也可以抽象成二分类问题,即分割出inlier与outlier.

![](https://pic.downk.cc/item/5ef89bdf14195aa594d80111.jpg)

以这个图为例.蓝色是inlier,红色是outlier.为了提取出直线,传统方法就是ransac.我们用深度学习对方法来解决这个问题.

我们要通过网络来学习点分布的特征,给每一个点赋予inlier或outlier属性.
二维的离散点和图像不同,它离散的属性给点的卷积带来很大的麻烦.Minkowski Engine的大致原理就是将点云格网化,以空间格网组织点云,每一个格网赋予特征.后续在详细说明.


首先继承Dataset类,生成散点, 其中inlier是分布在一条线上的,其他点随机分布.

```python

class RandomLineDataset(Dataset):

    # Warning: read using mutable obects for default input arguments in python.
    def __init__(
        self,
        angle_range_rad=[-np.pi/4, np.pi/4],
        line_params=[
            -1,  # Start
            1,  # end
        ],
        is_linear_noise=True,
        dataset_size=100,
        num_samples=10000,
        quantization_size=0.005):

        self.angle_range_rad = angle_range_rad
        self.is_linear_noise = is_linear_noise
        self.line_params = line_params
        self.dataset_size = dataset_size
        self.rng = np.random.RandomState(0)

        self.num_samples = num_samples
        self.num_data = int(0.2 * num_samples)
        self.num_noise = num_samples - self.num_data

        self.quantization_size = quantization_size

    def __len__(self):
        return self.dataset_size

    def _uniform_to_angle(self, u):
        return (self.angle_range_rad[1] -
                self.angle_range_rad[0]) * u + self.angle_range_rad[0]

    def _sample_noise(self, num, noise_params):
        noise = noise_params[0] + self.rng.randn(num, 1) * noise_params[1]
        return noise

    def _sample_xs(self, num):
        """Return random numbers between line_params[0], line_params[1]"""
        return (self.line_params[1] - self.line_params[0]) * self.rng.rand(
            num, 1) + self.line_params[0]

    # 获取一个训练样本,由num_samples个点组成
    def __getitem__(self, i):
        # Regardless of the input index, return randomized data
        angle, intercept = np.tan(self._uniform_to_angle(
            self.rng.rand())), self.rng.rand()

        # Line as x = cos(theta) * t, y = sin(theta) * t + intercept and random t's
        # Drop some samples
        xs_data = self._sample_xs(self.num_data)
        ys_data = angle * xs_data + intercept + self._sample_noise(
            self.num_data, [0, 0.1])

        noise = 4 * (self.rng.rand(self.num_noise, 2) - 0.5)

        # Concatenate data
        input = np.vstack([np.hstack([xs_data, ys_data]), noise])
        feats = input
        labels = np.vstack(
            [np.ones((self.num_data, 1)),
             np.zeros((self.num_noise, 1))]).astype(np.int32)

        # Quantize the input
        discrete_coords, unique_feats, unique_labels = ME.utils.sparse_quantize(
            coords=input,
            feats=feats,
            labels=labels,
            quantization_size=self.quantization_size,
            ignore_label=-100)

        return discrete_coords, unique_feats, unique_labels

```

由于Minkowski Engine 中coords向量的第一个维度是batch id,因此,在提取训练样本的时候,需要重新定义collation_fn函数,给coords加上一个维度.

```python

def collation_fn(data_labels):
    coords, feats, labels = list(zip(*data_labels))
    coords_batch, feats_batch, labels_batch = [], [], []

    # Generate batched coordinates
    coords_batch = ME.utils.batched_coordinates(coords)

    # Concatenate all lists
    feats_batch = torch.from_numpy(np.concatenate(feats, 0)).float()
    labels_batch = torch.from_numpy(np.concatenate(labels, 0))

    return coords_batch, feats_batch, labels_batch

```

下面简要分析下网络的结构 Unet

前三个block是由MinkowskiConvolution组成的,目的是卷积提取点的高维度特征.
后三个部分是反向卷积,相当于内插特征.在内插的过程中,需要concate前面卷积得到的特征.也很好理解,是为了顾及特征进行内插.

```python

class UNet(ME.MinkowskiNetwork):

    def __init__(self, in_nchannel, out_nchannel, D):
        super(UNet, self).__init__(D)
        self.block1 = torch.nn.Sequential(
            ME.MinkowskiConvolution(
                in_channels=in_nchannel,
                out_channels=8,
                kernel_size=3,
                stride=1,
                dimension=D),
            ME.MinkowskiBatchNorm(8))

        self.block2 = torch.nn.Sequential(
            ME.MinkowskiConvolution(
                in_channels=8,
                out_channels=16,
                kernel_size=3,
                stride=2,
                dimension=D),
            ME.MinkowskiBatchNorm(16),
        )

        self.block3 = torch.nn.Sequential(
            ME.MinkowskiConvolution(
                in_channels=16,
                out_channels=32,
                kernel_size=3,
                stride=2,
                dimension=D),
            ME.MinkowskiBatchNorm(32))

        self.block3_tr = torch.nn.Sequential(
            ME.MinkowskiConvolutionTranspose(
                in_channels=32,
                out_channels=16,
                kernel_size=3,
                stride=2,
                dimension=D),
            ME.MinkowskiBatchNorm(16))

        self.block2_tr = torch.nn.Sequential(
            ME.MinkowskiConvolutionTranspose(
                in_channels=32,
                out_channels=16,
                kernel_size=3,
                stride=2,
                dimension=D),
            ME.MinkowskiBatchNorm(16))

        self.conv1_tr = ME.MinkowskiConvolution(
            in_channels=24,
            out_channels=out_nchannel,
            kernel_size=1,
            stride=1,
            dimension=D)

    def forward(self, x):
        out_s1 = self.block1(x)
        out = MF.relu(out_s1)

        out_s2 = self.block2(out)
        out = MF.relu(out_s2)

        out_s4 = self.block3(out)
        out = MF.relu(out_s4)

        out = MF.relu(self.block3_tr(out))
        out = ME.cat(out, out_s2)

        out = MF.relu(self.block2_tr(out))
        out = ME.cat(out, out_s1)

        return self.conv1_tr(out)


```


训练的过程和一般的网络大同小异,如下

```python

    criterion = torch.nn.CrossEntropyLoss(ignore_index=-100)

    accum_loss, accum_iter, tot_iter = 0, 0, 0

    for epoch in range(config.max_epochs):
        train_iter = iter(train_dataloader)

        # Training
        net.train()
        for i, data in enumerate(train_iter):
            coords, feats, labels = data

            example_mask = coords[:,0] ==0

            plot(feats[example_mask],labels[example_mask].squeeze())


            out = net(ME.SparseTensor(feats.float(), coords))
            optimizer.zero_grad()
            loss = criterion(out.F.squeeze(), labels.long())
            loss.backward()
            optimizer.step()

            accum_loss += loss.item()
            accum_iter += 1
            tot_iter += 1

            if tot_iter % 10 == 0 or tot_iter == 1:
                print(
                    f'Epoch: {epoch} iter: {tot_iter}, Loss: {accum_loss / accum_iter}'
                )
                accum_loss, accum_iter = 0, 0

    torch.save(obj=net.state_dict(), f="models/net.pth")

```

其中要使用CrossEntropyLoss作为代价(二分类).

做一个小结. Minkowski Engine 实现了一种随意维度,大小的点云卷积方式. 核心是使用voxel组织管理输入的高维度离散数据.下面正式学习下Minkowski Convolutional的原理.

# 2. Minkowski Convolutional Neural Networks

参考:4D Spatio-Temporal ConvNets: Minkowski Convolutional Neural Networks

## 2.1 Sparse Tensor

对于离散点集输入$C$和其对应的特征$F$如下:

$$ 
C=\begin{bmatrix}
    x_1 & y_1 & z_1 & t_1 & b_1 \\
        &&        ... \\
    x_N & y_N & z_N & t_N & b_N 
 \end{bmatrix}, 

F=\begin{bmatrix} 
f_1^{T} \\
...\\
f_N^{T}
\end{bmatrix}
$$

其中$x_N,  y_N, z_N$是空间坐标,$t_N$是时间, $b_N$是batch index.

## 2.2 Generalized Sparse Convolution

$$ x_u^{out} = \sum_{i\in V^D(K)}{W_i x_{u+i}^{in}}$$

其中 $W_i$是卷积核的权重.维数为$W \in R^{K^D \times N^{out} \times N^{in}}$

卷积核的形状如下图所示,可以根据需要去定义.


![](https://pic.downk.cc/item/5ef9946a14195aa5941baaac.jpg)

## 2.3 Sparse Tensor Quantization

一开始对Quantization这个概念有些误解.其实作者的意思就是对原始点云进行体素化. 体素化的目的是方便进行2.2提到的卷积. 体素中只保留一个特征点及其卷积后得到的特征.

## 2.4 ResNET18 & Unet

利用sparse tensor 这一方式,很容易将图像领域成熟的卷积网络转换到点云上,如下图所示.

![](https://pic.downk.cc/item/5ef9a8cd14195aa5942206df.jpg)

![](https://pic.downk.cc/item/5ef9a8f614195aa594221844.jpg)

# 3. Fully Convolutional Geometric Features

该网络主要由残差网络和跳跃连接组成的UNet构成,如下图

![](https://pic.downk.cc/item/5ef9ef1a14195aa594391d43.jpg)

## 3.1 Metric learning 度量学习

参考:Improved Deep Metric Learning with Multi-class N-pair Loss Objective

特征匹配的学习可以理解为提取特征映射,这个映射可以让相似的特征具有较近的距离,不同的特征距离较远.

![](https://pic.downk.cc/item/5efaa3d214195aa5946e55c5.jpg)

如何为训练过程提供正负样本是metric learning的关键.如上图所示,Triplet loss是最简单的方式,即提供一个正样本,提供一个负样本,利用正样本的距离小于负样本来进行训练.

而这种方式的弊端就是训练的过程只能看到一个负样本.(N+1)-tuplet loss 是再一次训练中提供很多负样本,增大训练可靠性.这样做的弊端是数据集合极大.

N-pair-mc loss则是把正样本对之间互相作为负样本,减少训练的数据量.

## 3.2 Hardest-contrastive and Hardest-triplet Losses

继续回到本文中,看下本文提出的Hardest-triplet Losses.

本文提出的Hardest-triplet Losses和上述方式的区别是,上面的方法把所有负样本的影响放在一起训练,而本文方法只是将"最难"的那一个负样本挑出来,进行训练.这样做的好处直观上的理解是,只要保证区分性就可以了,没必要保证和所有的负样本都具有较远的距离.似乎本文的创新点就是这个...

## 3.3 具体训练

对于kitti数据的处理:

For all LIDAR scans, we used the first scan that is taken at least 10m apart within each sequence to create a pair. We found the GPS “ground truth” to be very noisy and used the Iterative Clos- est Point algorithm (ICP) to refine the alignment.

对于样本扩容:

We applied data augmentation including random scaling ∈ [0.8, 1.2] to a pair, and different random rotation augmentation ∈ [0◦, 360◦) along an arbitrary 3D direction for both scans in a pair. We



## 4.Deep Global Registration

参考:Deep Global Registration

本文是Fully Convolutional Geometric Features的延续与运用.过了一下,其中最重要的部分是inlier和outlier二分类.就和开头的那个例子一样.

这里有一个点,给我映像很深刻.

开头二维的例子,是二维坐标$[x,y]$的二分类,inlier分布在一条直线上.而对于空间特征点匹配,是两组点$[(x_r,y_r,z_r),(x_t,y_t,z_t)]$ 是否满足转换关系$R,t$转换.可以直接把这两组点够成一个6维度向量,直接在6维空间内进行卷积,就可以了分割出inlier.仔细一想还是被作者的思维折服了.二分类的构架也是UNet,如下图

![](https://pic.downk.cc/item/5efae7b414195aa59482f4c8.jpg)


这里有一个点,个人觉得很值得思考.之前总听组内的小伙伴提端到端,例如配准,就是从输入开始,一层一层地叠加网络,一个网络把从特征点到最终r,t解算全干了.但是看了Choy的一系列工作,其实特征提取,outlier filer,r,t计算,都是很明确的可分离的目标,既然处理流程(特诊提取->匹配->rt)确定了,是否真的需要端到端呢?


