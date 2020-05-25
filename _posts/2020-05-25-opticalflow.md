---
layout: post
title: optical flow
date: 2020-05-25
Author: jianping
categories: 
tags: [cv, learning]
comments: false
---

## Optical flow

##传统方法

### Lucas/Kanade 

参考：Lucas/Kanade Meets Horn/Schunck: Combining Local and Global Optic Flow Methods

$ I(x, y, t) = I(x + \delta x,y + \delta y,t+ \delta t) $


$ I(x + \delta x,y + \delta y,t+\delta t) = 
I(x, y, t) + 
\frac {\partial I} {\partial x} \delta x + 
\frac {\partial I} {\partial y} \delta y + 
\frac {\partial I} {\partial t} \delta t $


$
{\left[\begin{array}{cc}
  I_{x_1} & I_{x_2} & I_{x_3} & ... & I_{x_n}
 \\\ I_{y_1} & I_{y_2} & I_{y_3} & ... & I_{y_n}
\end{array}\right]}^\top
{\left[\begin{array}{cc}
  \delta y 
 \\ \delta y
\end{array}\right]}
 = 
{\left[\begin{array}{c}
  I_{t_1} & I_{t_2} & I_{t_3} & ... & I_{t_n}
\end{array}\right]}^\top
$
