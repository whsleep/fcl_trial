# VC2019+vcpkg构建`FCL`,`PCL`

参考：

[如何将vcpkg集成到vs并有效配置（保姆级（大概））-CSDN博客](https://blog.csdn.net/weixin_73959052/article/details/145927000)

[Visual Studio开源库集成器Vcpkg全教程--利用Vcpkg轻松集成开源第三方库_vcpkg vsstudio-CSDN博客](https://blog.csdn.net/cjmqas/article/details/79282847)

[Vcpkg C/C++库管理工具安装和使用教程（链接VS2019）_vcpkg 安装-CSDN博客](https://blog.csdn.net/qq_41023270/article/details/109274433)

## 下载`vcpkg`

[microsoft/vcpkg: C++ Library Manager for Windows, Linux, and MacOS](https://github.com/microsoft/vcpkg)

```shell
cd vcpkg
bootstrap-vcpkg.bat
```

注意：**网络不好可能不会成功**

检查是否安装成功

```shell
.\vcpkg.exe --version
```

## 安装fcl库

```shell
.\vcpkg.exe install fcl
```

查看是否安装成功

```shell
.\vcpkg.exe list
```

输出结果为：

```shell
ccd:x64-windows                                   2.1#4               Library for collision detection between two conv...
eigen3:x64-windows                                3.4.0#5             C++ template library for linear algebra: matrice...
fcl:x64-windows                                   0.7.0#4             A library for performing three types of proximit...
octomap:x64-windows                               1.10.0              An Efficient Probabilistic 3D Mapping Framework ...
vcpkg-cmake-config:x64-windows                    2024-05-23
vcpkg-cmake:x64-windows                           2024-04-23
```

`fcl`库及相关依赖已经安装成功

## 安装pcl库

```shell
.\vcpkg.exe install pcl
```

这里会出现报错

<font color="red">error: building liblzma:x64-windows failed with: BUILD_FAILED
See https://learn.microsoft.com/vcpkg/troubleshoot/build-failures?WT.mc_id=vcpkg_inproduct_cli for more information.</font>
Elapsed time to handle liblzma:x64-windows: 3.2 s
<font color="red">Please ensure you're using the latest port files with `git pull` and `vcpkg update`.
Then check for known issues at:
  https://github.com/microsoft/vcpkg/issues?q=is%3Aissue+is%3Aopen+in%3Atitle+liblzma
You can submit a new issue at:
  https://github.com/microsoft/vcpkg/issues/new?title=[liblzma]+Build+error+on+x64-windows&body=Copy%20issue%20body%20from%20D%3A%2FProgramData%2Fvcpkg%2Fvcpkg%2Finstalled%2Fvcpkg%2Fissue_body.md</font>

`liblzma`库安装失败

参考 [[liblzma\] Build error on x64-windows · Issue #44846 · microsoft/vcpkg](https://github.com/microsoft/vcpkg/issues/44846)

手动配置`vcpkg`的`ports`，进入目录`..\vcpkg\ports\liblzma`下

1. 将`vcpkg.json`中的`version`修改为`5.8.1`

2. 执行`.\vcpkg.exe install liblzma:x64-windows`命令，仍然会报错，会提示`hash`错误，并给出`actual`值，然后修改`portfile.cmake`中的`SHA512`为`actual`值即可；或者直接将下面使用下面这段代码覆盖`portfile.cmake`文件的`vcpkg_from_github`部分
```cmake
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO tukaani-project/xz
    REF "v${VERSION}"
    # SHA512 32c65500ccb49f598d88bca27cdd7bff35b505f16736ed341797eb308dc7fc9f4b01a9c8cacbecd6480701a2f8427777d476504eced663fc4f8b161f0e16adec
    SHA512 2f51fb316adb2962e0f2ef6ccc8b544cdc45087b9ad26dcd33f2025784be56578ab937c618e5826b2220b49b79b8581dcb8c6d43cd50ded7ad9de9fe61610f46
    HEAD_REF master
    PATCHES
        build-tools.patch
)
```

3. 再次执行`.\vcpkg.exe install liblzma:x64-windows`即可安装成功

安装成功后，再次尝试安装`pcl`

```shell
.\vcpkg.exe install pcl
```

注意：**安装比较耗时**

查看安装是否成功

```shell
.\vcpkg.exe list
```

主要输出为

```shel
ccd:x64-windows                                   2.1#4               Library for collision detection between two conv...
eigen3:x64-windows                                3.4.0#5             C++ template library for linear algebra: matrice...
fcl:x64-windows                                   0.7.0#4             A library for performing three types of proximit...
octomap:x64-windows                               1.10.0              An Efficient Probabilistic 3D Mapping Framework ...
pcl:x64-windows                                   1.15.0#2            Point Cloud Library (PCL) is open source library...
```

## 安装pcl[visualization]

```shell
.\vcpkg install pcl[visualization] --recurse
```

注意：**安装比较耗时**

## 全局集成

```shell
.\vcpkg.exe integrate install
```

将 vcpkg 安装的库路径（如头文件、库文件）**全局集成到 Visual Studio**，所有新建或现有的 C++ 项目均可直接引用这些库，无需手动配置路径。

注意：库的安装版本为`x64`，在`vs2019`执行时选择`x64`平台才能正常执行。

# 轨迹的碰撞检测

`visual studio 2019`新建工程，添加对应库，检测是否安装成功

```cpp
#include <iostream>
#include <vector>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>
#include <fcl/geometry/shape/sphere.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
```

运行后会报错 `pcl`依赖的`vtk`无法找到

## 解决

`vcpkg`只将第三方库的路径添加到

```shell
..\vcpkg\installed\x64-windows\include
```

每个第三方库又会单开文件夹，引用时需要加`vtk`库文件夹名称，才不会报错

手动添加过于麻烦，直接将`vtk`文件夹绝对路径添加到工程的附加包含目录即可

`右键工程`->`属性`->`C/C++`->`常规`->`附加包含目录`

将`vtk`的路径添加即可

```shell
..\vcpkg\installed\x64-windows\include\vtk
```

