---

layout: post
title: deep image matching
date: 2020-10-28
Author: jianping
categories: 
tags: [deep learning]
comments: false
---

# Contents
{:.no_toc}
* Will be replaced with the ToC, excluding the "Contents" header
{:toc}

# 1 R2D2：Repeatable and Reliable Detector and Descriptor



## 1.1 Base Network



```python

class BaseNet (nn.Module):
    """ Takes a list of images as input, and returns for each image:
        - a pixelwise descriptor
        - a pixelwise confidence
    """
    
    # 这里和论文有一点区别，reliability 和 repeatability都可以理解为二分类问题，可以用两个band，一个表示+，一个表示-，在经过softmax。也可以用一个band，经过softplus，在归一化x / (1 + x)
    def softmax(self, ux):
        if ux.shape[1] == 1:
            x = F.softplus(ux)
            return x / (1 + x)  # for sure in [0,1], much less plateaus than softmax
        elif ux.shape[1] == 2:
            return F.softmax(ux, dim=1)[:,1:2]

    def normalize(self, x, ureliability, urepeatability):
        return dict(descriptors = F.normalize(x, p=2, dim=1),
                    repeatability = self.softmax( urepeatability ),
                    reliability = self.softmax( ureliability ))

    def forward_one(self, x):
        raise NotImplementedError()

    def forward(self, imgs, **kw):
        res = [self.forward_one(img) for img in imgs]
        # merge all dictionaries into one :reliability, descriptors, repeatability
        res = {k:[r[k] for r in res if k in r] for k in {k for r in res for k in r}}
        return dict(res, imgs=imgs, **kw)

```



```python

class PatchNet (BaseNet):
    """ Helper class to construct a fully-convolutional network that
        extract a l2-normalized patch descriptor.
    """
    def __init__(self, inchan=3, dilated=True, dilation=1, bn=True, bn_affine=False):
        BaseNet.__init__(self)
        self.inchan = inchan
        self.curchan = inchan
        self.dilated = dilated
        self.dilation = dilation
        self.bn = bn
        self.bn_affine = bn_affine
        self.ops = nn.ModuleList([])

    def _make_bn(self, outd):
        return nn.BatchNorm2d(outd, affine=self.bn_affine)

    def _add_conv(self, outd, k=3, stride=1, dilation=1, bn=True, relu=True):
        d = self.dilation * dilation
        if self.dilated: 
            conv_params = dict(padding=((k-1)*d)//2, dilation=d, stride=1)
            self.dilation *= stride
        else:
            conv_params = dict(padding=((k-1)*d)//2, dilation=d, stride=stride)
        self.ops.append( nn.Conv2d(self.curchan, outd, kernel_size=k, **conv_params) )
        if bn and self.bn: self.ops.append( self._make_bn(outd) )
        if relu: self.ops.append( nn.ReLU(inplace=True) )
        self.curchan = outd
    
    def forward_one(self, x):
        assert self.ops, "You need to add convolutions first"
        for n,op in enumerate(self.ops):
            x = op(x)
        return self.normalize(x)

# 特征提取backbone
class Quad_L2Net (PatchNet):
    """ Same than L2_Net, but replace the final 8x8 conv by 3 successive 2x2 convs.
    """
    def __init__(self, dim=128, mchan=4, relu22=False, **kw ):
        PatchNet.__init__(self, **kw)
        self._add_conv(  8*mchan)
        self._add_conv(  8*mchan)
        self._add_conv( 16*mchan, stride=2)
        self._add_conv( 16*mchan)
        self._add_conv( 32*mchan, stride=2)
        self._add_conv( 32*mchan)
        # replace last 8x8 convolution with 3 2x2 convolutions
        self._add_conv( 32*mchan, k=2, stride=2, relu=relu22)
        self._add_conv( 32*mchan, k=2, stride=2, relu=relu22)
        self._add_conv(dim, k=2, stride=2, bn=False, relu=False)
        self.out_dim = dim
        
class Quad_L2Net_ConfCFS (Quad_L2Net):
    """ Same than Quad_L2Net, with 2 confidence maps for repeatability and reliability.
    """
    def __init__(self, **kw ):
        Quad_L2Net.__init__(self, **kw)
        # reliability classifier
        self.clf = nn.Conv2d(self.out_dim, 2, kernel_size=1)
        # repeatability classifier: for some reasons it's a softplus, not a softmax!
        # Why? I guess it's a mistake that was left unnoticed in the code for a long time...
        self.sal = nn.Conv2d(self.out_dim, 1, kernel_size=1) 

    def forward_one(self, x):
        assert self.ops, "You need to add convolutions first"
        for op in self.ops:
            x = op(x)
        # compute the confidence maps
        ureliability = self.clf(x**2)
        urepeatability = self.sal(x**2)

        # print("urepeatability:", urepeatability.shape)
        return self.normalize(x, ureliability, urepeatability)
        
        
        

```

## 1.2 Sampler

根据GT的flow来计算image1 和 image2 的匹配

```python

class FullSampler(nn.Module):
    """ all pixels are selected
        - feats: keypoint descriptors
        - confs: reliability values
    """
    def __init__(self):
        nn.Module.__init__(self)
        self.mode = 'bilinear'
        self.padding = 'zeros'

    @staticmethod
    def _aflow_to_grid(aflow):
        H, W = aflow.shape[2:]
        grid = aflow.permute(0,2,3,1).clone()
        grid[:,:,:,0] *= 2/(W-1)
        grid[:,:,:,1] *= 2/(H-1)
        grid -= 1
        grid[torch.isnan(grid)] = 9e9 # invalids
        return grid
    
    # feats 是img2的特征
    # confs 是img2的confidence
    # aflow 是GT的光流
    def _warp(self, feats, confs, aflow):
        if isinstance(aflow, tuple): return aflow # result was precomputed
        feat1, feat2 = feats
        conf1, conf2 = confs if confs else (None,None)
    
        B, two, H, W = aflow.shape
        D = feat1.shape[1]
        assert feat1.shape == feat2.shape == (B, D, H, W) # D = 128, B = batch
        assert conf1.shape == conf2.shape == (B, 1, H, W) if confs else True

        # warp img2 to img1
        grid = self._aflow_to_grid(aflow)
        
        # ones2 的作用是，光流warp后，可能产生空洞
        ones2 = feat2.new_ones(feat2[:,0:1].shape)
        
        # grid_sample 逐一采样方式，grid是对应采样像素
        feat2to1 = F.grid_sample(feat2, grid, mode=self.mode, padding_mode=self.padding)
        mask2to1 = F.grid_sample(ones2, grid, mode='nearest', padding_mode='zeros')
        conf2to1 = F.grid_sample(conf2, grid, mode=self.mode, padding_mode=self.padding) \
                   if confs else None
        return feat2to1, mask2to1.byte(), conf2to1

    def _warp_positions(self, aflow):
        B, two, H, W = aflow.shape
        assert two == 2
        
        Y = torch.arange(H, device=aflow.device)
        X = torch.arange(W, device=aflow.device)
        XY = torch.stack(torch.meshgrid(Y,X)[::-1], dim=0)
        XY = XY[None].expand(B, 2, H, W).float()
        
        grid = self._aflow_to_grid(aflow)
        XY2 = F.grid_sample(XY, grid, mode='bilinear', padding_mode='zeros')
        return XY, XY2

class SubSampler (FullSampler):
    """ pixels are selected in an uniformly spaced grid
    """
    def __init__(self, border, subq, subd, perimage=False):
        FullSampler.__init__(self)
        assert subq % subd == 0, 'subq must be multiple of subd'
        self.sub_q = subq
        self.sub_d = subd
        self.border = border
        self.perimage = perimage

    def __repr__(self):
        return "SubSampler(border=%d, subq=%d, subd=%d, perimage=%d)" % (
            self.border, self.sub_q, self.sub_d, self.perimage)

    def __call__(self, feats, confs, aflow):
        feat1, conf1 = feats[0], (confs[0] if confs else None)
        # warp with optical flow in img1 coords
        # 因为warp，这个时候，img2 的特征已经转换到img1中了
        feat2, mask2, conf2 = self._warp(feats, confs, aflow)
        
        # subsample img1
        # slice 是python切片对象
        slq = slice(self.border, -self.border or None, self.sub_q)
        feat1 = feat1[:, :, slq, slq]
        conf1 = conf1[:, :, slq, slq] if confs else None
        # subsample img2
        sld = slice(self.border, -self.border or None, self.sub_d)
        feat2 = feat2[:, :, sld, sld]
        mask2 = mask2[:, :, sld, sld]
        conf2 = conf2[:, :, sld, sld] if confs else None
        
        B, D, Hq, Wq = feat1.shape
        B, D, Hd, Wd = feat2.shape
        
        # compute gt
        if self.perimage or self.sub_q != self.sub_d:
            # compute ground-truth by comparing pixel indices
            f = feats[0][0:1,0] if self.perimage else feats[0][:,0]
            idxs = torch.arange(f.numel(), dtype=torch.int64, device=feat1.device).view(f.shape)
            idxs1 = idxs[:, slq, slq].reshape(-1,Hq*Wq)
            idxs2 = idxs[:, sld, sld].reshape(-1,Hd*Wd)
            if self.perimage:
                gt = (idxs1[0].view(-1,1) == idxs2[0].view(1,-1))
                gt = gt[None,:,:].expand(B, Hq*Wq, Hd*Wd)
            else :
                gt = (idxs1.view(-1,1) == idxs2.view(1,-1)) 
        else:
            gt = torch.eye(feat1[:,0].numel(), dtype=torch.uint8, device=feat1.device) # always binary for AP loss
        
        # compute all images together
        queries  =  feat1.reshape(B,D,-1) # B x D x (Hq x Wq)
        database =  feat2.reshape(B,D,-1) # B x D x (Hd x Wd)
        if self.perimage:
            queries  =  queries.transpose(1,2) # B x (Hd x Wd) x D
            scores = torch.bmm(queries, database) # B x (Hq x Wq) x (Hd x Wd)
        else:
            queries  =  queries .transpose(1,2).reshape(-1,D) # (B x Hq x Wq) x D
            database =  database.transpose(1,0).reshape(D,-1) # D x (B x Hd x Wd)
            scores = torch.matmul(queries, database) # (B x Hq x Wq) x (B x Hd x Wd)

        # compute reliability
        qconf = (conf1 + conf2)/2 if confs else None

        assert gt.shape == scores.shape
        return scores, gt, mask2, qconf


```







