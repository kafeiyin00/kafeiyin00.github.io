---
layout: post
title: ubuntu_proxy
date: 2020-07-09
Author: jianping
categories: 
tags: [PROXY]
comments: false
---

#1 ����shadowsocks

#2 ʹ��genpac����pac�ļ�
```cpp
//���shadowsocks��socks5�Ĵ����ַ
//--gfwlist-local ��Ӧ https://raw.githubusercontent.com/gfwlist/gfwlist/master/gfwlist.txt�е�����

genpac --proxy="SOCKS5 127.0.0.1:1080" --gfwlist-proxy="SOCKS5 127.0.0.1:1080" -o autoproxy.pac --gfwlist-local="1.txt"

```

#3  consol������

```cpp
export http_proxy='http://localhost:1080'
export https_proxy='http://localhost:1080'
```

#4 ���

```cpp
curl -v www.google.com

```