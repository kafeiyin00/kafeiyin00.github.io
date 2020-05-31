---
layout: post
title: 国产某型号MEMS IMU测试
date: 2020-05-01
Author: jianping
categories: 
tags: [IMU, hardware]
comments: false
---

### IMU 封装

与ADIS16488兼容

接口为 samtec CLM-112-02-L-D-A

### IMU 数据接口

1.SPI(200hz)

SPI配置：
```CPP

 	XSP_MASTER_OPTION| XSP_MANUAL_SSELECT_OPTION  | XSP_CLK_PHASE_1_OPTION | XSP_CLK_ACTIVE_LOW_OPTION

```
最大传输速率4m. 说明书中要求拉低到低速模式（256分频），实际上测试并没有这么严格的要求。为了传输速率，将主时钟线100mhz进行16分频即可。

2.时间同步：捕获上升延

DIO2: data ready


3.数据比例转换

X_GYRO_OUT: LSB=0.02deg/s
X_ACCL_OUT: LSB=0.8mg


```cpp
	// transform temperature to double
	imuFramePtr->temperature = 25 + temper_out * 0.00565;

	// transform gyro to double
	imuFramePtr->gyroX = (double)(65536 * x_gyro_out + x_gyro_low) * 0.0000152587890625 * 0.02;
	imuFramePtr->gyroY = (double)(65536 * y_gyro_out + y_gyro_low) * 0.0000152587890625 * 0.02;
	imuFramePtr->gyroZ = (double)(65536 * z_gyro_out + z_gyro_low) * 0.0000152587890625 * 0.02;


	// transform accl to double
	imuFramePtr->acclX = (double)(65536 * x_accl_out + x_accl_low) * 0.0000152587890625 * 0.8 * 0.001 * 9.80147;
	imuFramePtr->acclY = (double)(65536 * y_accl_out + y_accl_low) * 0.0000152587890625 * 0.8 * 0.001 * 9.80147;
	imuFramePtr->acclZ = (double)(65536 * z_accl_out + z_accl_low) * 0.0000152587890625 * 0.8 * 0.001 * 9.80147;

```




### IMU 陀螺 allan 方差测试

1. 静置2小时，采集数据

2. 将陀螺数据进行方差分析
```matlab
[avarx,taux] = allanvar(gyro_x * 3600,'octave',200);
loglog(taux,avarx);
```

结果如下


![allan var](https://pic.downk.cc/item/5ecb2ec8c2a9a83be5da6e0f.png)


陀螺稳定性在1deg/h以内
陀螺角度随机游走为0.3deg/sqrt(h)左右

### 结论

　　　　国产之光。



{% include gittalk.html %} 
