---
layout: post
title: ubuntu_proxy
date: 2020-07-09
Author: jianping
categories: 
tags: [PROXY]
comments: false
---

#1 配置shadowsocks

#2 使用genpac生成pac文件
```cpp
//检查shadowsocks中socks5的代理地址
//--gfwlist-local 对应 https://raw.githubusercontent.com/gfwlist/gfwlist/master/gfwlist.txt中的内容

genpac --proxy="SOCKS5 127.0.0.1:1080" --gfwlist-proxy="SOCKS5 127.0.0.1:1080" -o autoproxy.pac --gfwlist-local="1.txt"

```

#3  consol中配置

```cpp
export http_proxy='http://localhost:1080'
export https_proxy='http://localhost:1080'
```

#4 检查

```cpp
curl -v www.google.com

```