---
layout: post
title: ubuntu_proxy
date: 2020-07-09
Author: jianping
categories: 
tags: [PROXY]
comments: false
---

#1 Config shadowsocks

#2 use genpac to generate pac file
```cpp
// check shadowsocks: proxy path of socks5 
// --gfwlist-local https://raw.githubusercontent.com/gfwlist/gfwlist/master/gfwlist.txt

genpac --proxy="SOCKS5 127.0.0.1:1080" --gfwlist-proxy="SOCKS5 127.0.0.1:1080" -o autoproxy.pac --gfwlist-local="1.txt"

```

#3  consol

```cpp
export http_proxy='http://localhost:1080'
export https_proxy='http://localhost:1080'
```

#4 check

```cpp
curl -v www.google.com

```
