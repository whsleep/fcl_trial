# FCL连续碰撞检测

新建`sphere`和`box`用于测试

```cpp
    auto sphere = std::make_shared<fcl::Sphere<double>>(1.0); // 半径为 1 的球体
    auto box = std::make_shared<fcl::Box<double>>(2.0, 2.0, 2.0); // 2x2x2 的立方体
```

设置球以 $(3.0,4.0,0.0)$ 为`box`旋转中心，旋转半径`r`为5.0

> 确定球的起始位置和终点位置，`FCL`库中使用`Transform3`的变换矩阵表示坐标信息
>
$$
\left[
\begin{matrix}
R &  t \\
0 & 1 \\
\end{matrix}
\right]
$$
> 例如，对于三维点 $p_0=(x_0,y_0,z_0)$
> 
$$
\begin{align}
\left[
\begin{matrix}
p_1 \\
1 \\
\end{matrix}
\right]
&=
\left[
\begin{matrix}
R &  t \\
0 & 1 \\
\end{matrix}
\right]
\left[
\begin{matrix}
p_0 \\
1 \\
\end{matrix}
\right]\\
p_1&=R\cdot p_0+t
\end{align}
$$
> 
>具体流程为 $p_0$ 先绕原点旋转 $R$ 再平移 $t$ ，这里的旋转和平移均是相对原点。

设置`box`的起点和终点状态为

$$
tf \\_box \\_beg =
\left[
\begin{matrix}
1 & 0 & 0 &  3+5 \\
0 & 1 & 0 &  4 \\
0 & 0 & 1 &  0 \\
0 & 0 & 0 &  1 \\
\end{matrix}
\right]\quad 
tf \\_box \\_beg =
\left[
\begin{matrix}
cos(\theta) & -sin(\theta) & 0 &  3+5\cdot cos(\theta) \\
sin(\theta) & cos(\theta) & 0 &  4+5\cdot sin(\theta) \\
0 & 0 & 1 &  0 \\
0 & 0 & 0 &  1 \\
\end{matrix}
\right]
$$

<img src="https://github.com/whsleep/fcl_trial/blob/main/assets/image-20250509103134227.png" alt="image-20250509103134227" style="zoom:60%;" />



青色为起始位置，黄色为终点位置。

## 连续碰撞请求配置

```cpp
fcl::ContinuousCollisionRequest<double> request;
```

打开`ContinuousCollisionRequest`结构体

```cpp
template <typename S>
struct FCL_EXPORT ContinuousCollisionRequest
{
  // 起点终点的插值点数量
  std::size_t num_max_iterations;
  /// @brief error in first contact time
  S toc_err;
  // 连续碰撞运动类型
  CCDMotionType ccd_motion_type;
  // GJK求解器类型
  GJKSolverType gjk_solver_type;
  // 连续碰撞求解器类型
  CCDSolverType ccd_solver_type;
  // 构造函数，设定默认值
  ContinuousCollisionRequest(std::size_t num_max_iterations_ = 10,
                             S toc_err_ = 0.0001,
                             CCDMotionType ccd_motion_type_ = CCDM_TRANS,
                             GJKSolverType gjk_solver_type_ = GST_LIBCCD,
                             CCDSolverType ccd_solver_type_ = CCDC_NAIVE);
  
};
```
| <img src="https://github.com/whsleep/fcl_trial/blob/main/assets/1747098889060-2.png" alt="7466919607362" height= "400"/> | <img src="https://github.com/whsleep/fcl_trial/blob/main/assets/7466920017173.png" alt="7466920017173"  height = "400" /> |
| ------------------------------------------------------------ | ------------------------------------------------------------ |

然后会在 `continuousCollide`函数中执行碰撞检测

```cpp
template <typename S>
FCL_EXPORT
S continuousCollide(
    const CollisionGeometry<S>* o1,
    const Transform3<S>& tf1_beg,
    const Transform3<S>& tf1_end,
    const CollisionGeometry<S>* o2,
    const Transform3<S>& tf2_beg,
    const Transform3<S>& tf2_end,
    const ContinuousCollisionRequest<S>& request,
    ContinuousCollisionResult<S>& result)
{
  MotionBasePtr<S> motion1 = getMotionBase(tf1_beg, tf1_end, request.ccd_motion_type);
  MotionBasePtr<S> motion2 = getMotionBase(tf2_beg, tf2_end, request.ccd_motion_type);

  return continuousCollide(o1, motion1.get(), o2, motion2.get(), request, result);
}
```

函数内部会根据`request.ccd_motion_type`类型生成插值的运动基类指针，具体插值细节再以下四个文件中

```cpp
#include "fcl/math/motion/translation_motion.h"
#include "fcl/math/motion/interp_motion.h"
#include "fcl/math/motion/screw_motion.h"
#include "fcl/math/motion/spline_motion.h"
```

这里我只关注螺旋运动`screw`插值细节，在`#include "fcl/math/motion/screw_motion.h"`文件内

```cpp
class FCL_EXPORT ScrewMotion : public MotionBase<S>
```

`ScrewMotion`派生类继承自 `MotionBase`运动基类

重点关注下面的构造函数

```cpp
/// @brief Construct motion from the initial transform and goal transform
template <typename S>
ScrewMotion<S>::ScrewMotion(
    const Transform3<S>& tf1_, const Transform3<S>& tf2_)
  : tf1(tf1_), tf2(tf2_), tf(tf1)
{
  computeScrewParameter();
}
/----------------computeScrewParameter()----------------------/
template <typename S>
void ScrewMotion<S>::computeScrewParameter()
{
  const AngleAxis<S> aa(tf2.linear() * tf1.linear().transpose());
    
  // 根据旋转矩阵提取旋转角度和旋转轴
  axis = aa.axis();
  angular_vel = aa.angle();

  if(angular_vel < 0)
  {
    angular_vel = -angular_vel;
    axis = -axis;
  }

  if(angular_vel < 1e-10)
  {
    angular_vel = 0;
    axis = tf2.translation() - tf1.translation();
    linear_vel = axis.norm();
    p = tf1.translation();
  }
  else
  {
    // 起点终点的平移增量
    Vector3<S> o = tf2.translation() - tf1.translation();
    // 确定参考旋转点
    p = (tf1.translation() + tf2.translation() + axis.cross(o) * (1.0 / tan(angular_vel / 2.0))) * 0.5;
    // 确定沿旋转轴的平移量
    linear_vel = o.dot(axis);
  }
}   
```

最终检测在函数 `continuousCollide`中运行，内容如下

```cpp
template <typename S>
FCL_EXPORT
S continuousCollide(
    const CollisionGeometry<S>* o1,
    const MotionBase<S>* motion1,
    const CollisionGeometry<S>* o2,
    const MotionBase<S>* motion2,
    const ContinuousCollisionRequest<S>& request,
    ContinuousCollisionResult<S>& result)
{
  switch(request.ccd_solver_type)
  {
  case CCDC_NAIVE:
    return continuousCollideNaive(o1, motion1,
                                  o2, motion2,
                                  request,
                                  result);
    break;
  case CCDC_CONSERVATIVE_ADVANCEMENT:
    return continuousCollideConservativeAdvancement(o1, motion1,
                                                    o2, motion2,
                                                    request,
                                                    result);
    break;
  case CCDC_RAY_SHOOTING:
    if(o1->getObjectType() == OT_GEOM && o2->getObjectType() == OT_GEOM && request.ccd_motion_type == CCDM_TRANS)
    {

    }
    else
      std::cerr << "Warning! Invalid continuous collision setting\n";
    break;
  case CCDC_POLYNOMIAL_SOLVER:
    if(o1->getObjectType() == OT_BVH && o2->getObjectType() == OT_BVH && request.ccd_motion_type == CCDM_TRANS)
    {
      return continuousCollideBVHPolynomial(o1, (const TranslationMotion<S>*)motion1,
                                            o2, (const TranslationMotion<S>*)motion2,
                                            request, result);
    }
    else
      std::cerr << "Warning! Invalid continuous collision checking\n";
    break;
  default:
    std::cerr << "Warning! Invalid continuous collision setting\n";
  }

  return -1;
}
```

`request.ccd_solver_type`默认参数为 `CCDC_NAIVE`，进而在`continuousCollideNaive`函数进行处理

```cpp
template <typename S>
FCL_EXPORT
S continuousCollideNaive(
    const CollisionGeometry<S>* o1,
    const MotionBase<S>* motion1,
    const CollisionGeometry<S>* o2,
    const MotionBase<S>* motion2,
    const ContinuousCollisionRequest<S>& request,
    ContinuousCollisionResult<S>& result)
{
  // 计算迭代次数，选取较小的迭代次数
  std::size_t n_iter = std::min(request.num_max_iterations, (std::size_t)ceil(1 / request.toc_err));
  Transform3<S> cur_tf1, cur_tf2;
  for(std::size_t i = 0; i < n_iter; ++i)
  {
    // 计算当前推进时间，最大为1
    S t = i / (S) (n_iter - 1);
    // 计算匀速旋转后的位置
    motion1->integrate(t);
    motion2->integrate(t);
	// 获取当前位置
    motion1->getCurrentTransform(cur_tf1);
    motion2->getCurrentTransform(cur_tf2);
	// 进行单次碰撞检测
    CollisionRequest<S> c_request;
    CollisionResult<S> c_result;

    if(collide(o1, cur_tf1, o2, cur_tf2, c_request, c_result))
    {
      // 返回是否碰撞及碰撞处的位置
      result.is_collide = true;
      result.time_of_contact = t;
      result.contact_tf1 = cur_tf1;
      result.contact_tf2 = cur_tf2;
      return t;
    }
  }

  result.is_collide = false;
  result.time_of_contact = S(1);
  return result.time_of_contact;
}
```

测试结果如下：

红色为`box`发生碰撞时的姿态。

<img src="https://github.com/whsleep/fcl_trial/blob/main/assets/image-20250512085204257.png" alt="image-20250512085204257" style="zoom:50%;" />

### 求解器类型测试

#### CCDM _LINEAR

使用`CCDM_LINEAR`运动类型进行插值，理论上不会发生碰撞，可以全部检测1000个插值点

##### `CCDC_NAIVE`

```cpp
    request.ccd_motion_type = fcl::CCDM_LINEAR;
    request.num_max_iterations = 1000; 
    request.ccd_solver_type = fcl::CCDC_NAIVE;

// 500us
```

##### `CCDC_CONSERVATIVE_ADVANCEMENT`

```cpp
    request.ccd_motion_type = fcl::CCDM_LINEAR;
    request.num_max_iterations = 1000; 
    request.ccd_solver_type = fcl::CCDC_CONSERVATIVE_ADVANCEMENT;

// 835us
```

#### CCDM_SCREW

使用`CCDM_SCREW`运动类型进行插值，仅检测起始状态到碰撞状态的所有插值点。

`CCDC_NAIVE`

```cpp
    request.ccd_motion_type = fcl::CCDM_SCREW;
    request.num_max_iterations = 1000; 
    request.ccd_solver_type = fcl::CCDC_NAIVE;

// 7252us
```

`CCDC_CONSERVATIVE_ADVANCEMENT`

```cpp
    request.ccd_motion_type = fcl::CCDM_SCREW;
    request.num_max_iterations = 1000; 
    request.ccd_solver_type = fcl::CCDC_CONSERVATIVE_ADVANCEMENT;

// 1290us
```

 `CCDC_RAY_SHOOTING`和`CCDC_POLYNOMIAL_SOLVER`只能用于平移运动`CCDM_TRANS`

```cpp
  case CCDC_RAY_SHOOTING:
    if(o1->getObjectType() == OT_GEOM && o2->getObjectType() == OT_GEOM && request.ccd_motion_type == CCDM_TRANS)
    {

    }
    else
      std::cerr << "Warning! Invalid continuous collision setting\n";
    break;
  case CCDC_POLYNOMIAL_SOLVER:
    if(o1->getObjectType() == OT_BVH && o2->getObjectType() == OT_BVH && request.ccd_motion_type == CCDM_TRANS)
    {
      return continuousCollideBVHPolynomial(o1, (const TranslationMotion<S>*)motion1,
                                            o2, (const TranslationMotion<S>*)motion2,
                                            request, result);
    }
    else
      std::cerr << "Warning! Invalid continuous collision checking\n";
    break;
```



