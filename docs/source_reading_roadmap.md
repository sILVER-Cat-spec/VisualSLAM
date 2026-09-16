# 源码阅读路线图：沿着六个问题走通 RGB-D SLAM

这份路线图的目标是让你能解释“下一步为什么会执行、数据从哪里来、谁真正改了状态”。一次只完成一站，不要求顺着目录读完所有类。先看本节给出的答案，再去源码里找证据，能回答末尾自检就停下。

依据 2026-09-12 当前 C++ 实现。链接定位到本工作区的具体行，代码变动后可用旁边的函数名重新搜索。这里的 loop 指逐帧处理循环；当前项目没有闭环检测线程或闭环优化流程。

## 今天只花 10 分钟：证明程序为什么能连续处理多帧

现在只开两个文件，其他链接暂时不点：

1. 打开 [run_dataset.cpp:44](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/apps/run_dataset.cpp:44)，看到 SlamSystem 在 for 循环外构造。
2. 往下看到 [主循环入口](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/apps/run_dataset.cpp:58)：每次 ReadFrame 后调用一次 slam.Process。
3. 跳到 [ProcessImpl](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/system/slam_system.cpp:20)，只找三个判断：没有 last_keyframe_、跟踪是否成功、是否插关键帧。

读完写下这三句话即可收工：

> 地图由循环外那个 System 持有，所以不会每帧消失。  
> 初始化成功会保存地图和参考关键帧，下一次 Process 就能用它们跟踪。  
> 建图与 BA 是某些成功帧里的同步步骤，不是另外一个后台循环。

如果今天只完成这里，已经抓住了项目的运行骨架。后面是逐次展开这三句话的路线，不是今天的额外作业。

## 第 1 站｜程序从启动到结束，SLAM 的生命周期究竟在哪里？

**先拿答案：**外层 main 循环负责逐帧推进；内层 ProcessImpl 负责处理一帧。算法组件在 System 构造时装配，第一张有效地图在初始化成功时才建立。初始化、跟踪、插帧、局部 BA 都在调用线程里顺序完成。

**这次只走下面四步，约 15 分钟。**

第一步，读 [main 启动部分](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/apps/run_dataset.cpp:14) 到逐帧循环前：加载配置和 manifest，准备输出，设置 OpenCV 线程/随机种子，再构造一个 SlamSystem。先把配置解析、日志格式、文件写出当作已知函数，不跳进去。

第二步，读 [SlamSystem 构造函数](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/system/slam_system.cpp:10)：找到 extractor_、tracker_、mapper_、空 map_ 和 frames_。此刻虽然有 Map 对象，但还没有初始关键帧；“对象已创建”不等于“视觉初始化已完成”。

第三步，返回 main，沿下面的真实调用顺序阅读。缩进表示调用或条件分支，箭头表示数据/控制继续向下：

~~~text
main
  ReadConfiguration + ReadManifest
  构造同一个 SlamSystem
  for 每条数据
    ReadFrame → FrameInput
    SlamSystem::Process → ProcessImpl
      Prepare → Extract → FrameFactory::Create → frame.SetFeatures
      没有 last_keyframe_
        InitializeRGBD
        成功：接管 Map，保存参考关键帧，发布初始位姿
        点不足：继续等待，无有效位姿
      已有 last_keyframe_
        Track → 成功后 RefineLocal
        最终失败：Lost，本帧无位姿，不建图
        最终成功：写 Frame → ShouldInsert
          需要插帧：LocalMapper::Process → 局部 BA → 同步 Frame
      返回 SlamResult
    记录本帧状态与成功轨迹的参考关系
  根据最终关键帧位姿导出轨迹、地图、点云
  离开 main，局部对象按生命周期释放
~~~

第四步，只比较这三种情况：

| 情况 | ProcessImpl 做什么 | main 接下来做什么 |
| --- | --- | --- |
| 初始化点不够 | 没有有效位姿，继续 Initializing | 正常记录并处理下一帧 |
| 已有地图但几何跟踪失败 | Lost，清除本帧位姿；地图仍在 | 状态码正常时继续下一帧 |
| 输入等错误导致 status 非正常 | 返回错误状态 | 当前 run_dataset 抛异常，结束本次运行 |

判断初始化分支的实际条件是 [if(!last_keyframe_)](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/system/slam_system.cpp:34)，不是一个 switch(state_)。Lost 后参考关键帧还在，所以后续仍尝试跟踪。[Reset](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/system/slam_system.cpp:78) 是调用方可主动使用的接口，当前 main 不会遇到 Lost 就自动 Reset。

还有一种“循环”是 Ceres 内部迭代：一次求解包含多轮数值更新，和处理多帧的 for 是两回事。

**停止条件：**能回答“初始化成功那一帧会接着调用 Track 吗？”  
答案：不会。它走初始化分支并返回；下一帧才走 else 里的 Track。

**下一站自然产生的问题：**Process 返回以后，刚得到的地图和参考究竟留在哪里？

## 第 2 站｜SLAM 的核心状态存在哪里？哪些值只属于本帧？

**先拿答案：**System 保存跨帧控制状态和地图所有权；Map 保存长期几何与观测关系；当前 Frame 保存本帧测量和估计；优化结果先保存在独立候选对象里。项目没有一个成员叫“当前全部 SLAM 状态”包办这些职责。

先打开 [SlamSystem 私有成员](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/include/vslam/system/slam_system.h:40)，只圈出四个名字：

~~~text
map_            拥有长期地图
last_keyframe_  指向当前参考关键帧的 ID
state_          当前运行状态标签
frames_         负责当前世代的帧 ID、sequence 和时间顺序
~~~

然后去 [Map 的三个持久容器](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/include/vslam/core/map.h:120)，只确认 keyframes_、points_、observations_。关键帧保存位姿和特征，点保存世界位置，观测连接两者。这一站先不读增删和异常回滚实现。

最后回到 [本帧的创建位置](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/system/slam_system.cpp:31)：

~~~cpp
result.frame=frames_->Create(...);
auto& frame=*result.frame;
~~~

上面 Create 的参数用省略号表示；第二句是源码原句。frame 是 result.frame 内对象的引用，不是另一份 Frame。后面对 frame.SetPose、frame.Associate 的写入，会体现在返回的结果对象中。Frame 内对应字段在 [frame.h:109](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/include/vslam/core/frame.h:109)。

| 你要找的状态 | 存放位置 | 谁负责让它生效 |
| --- | --- | --- |
| 本帧是否有位姿 | Frame 的 optional pose_；SlamResult.tcw | System 最终接受，mapping 插帧后可能同步修正 |
| 地图关键帧的位姿 | Map 拥有的 KeyFrame.tcw_ | 初始化/插帧建立；Map 几何提交更新 |
| 地图点世界位置 | Map 拥有的 MapPoint.position_w_ | 建点建立；Map 几何提交更新 |
| 本帧特征对应哪个点 | Frame.associations_ | System 写入跟踪接受的关联 |
| 长期观测关系 | Observation 与关键帧/点的索引 | Map 统一登记和删除 |
| 未接受的优化位姿/点 | PoseSolution 或 BASolution | 先返回调用方，不自动进入真实地图 |

tracker_ 还保存相机、配置和优化器，mapper_ 还保存插帧计数及新点观察期信息；但它们不各自拥有另一张主地图。

**停止条件：**能回答“在调试器里改 result.tcw，Map 的关键帧会一起变吗？”  
答案：不会。它是返回的位姿值，不是地图位姿的可写别名；这也说明为什么需要明确的写入和同步路径。

**下一站自然产生的问题：**初始化到底往这些位置写了什么，下一帧才能接得上？

## 第 3 站｜Initialization 的数据怎么进入 Tracking？

**先拿答案：**两者没有直接互相调用。初始化把点、关键帧和观测放入候选 Map；System 接管它并保存关键帧 ID；下一帧 System 把同一张 Map 和这个参考关键帧传给 Track。

只追一个具体特征，不要同时追整张地图。假设初始帧第 7 个特征有有效深度，后来匹配当前帧第 12 个特征。

**第一跳：像素怎样变成地图点。**读 [InitializeRGBD](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/initialization/rgbd_initializer.cpp:3)。这个文件很短，本轮可以读完整个函数。找到反投影、单位位姿和这几步：

~~~text
features[7] 的 uv、Z
  → camera.Unproject
  → 初始世界点 Pw（初始相机与世界重合）
  → map->AddMapPoint 返回 P
  → staged.Associate(7,P)
  → map->InsertKeyFrame(staged) 返回 K0
~~~

staged 是候选 Frame 副本；点数不足时在创建地图前返回，成功后才移动它更新输入 Frame。

**第二跳：关联怎样变成持久观测。**只读 [Map::InsertKeyFrame](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/core/map.cpp:53) 中“清空复制关联，再逐条调用 AddObservation”的部分。关键帧复制带来的 MapPoint ID 必须重新登记为观测边，不能只把关联数组复制过去就结束。此时只需知道 AddObservation 维护完整关系，回滚分支留给后续专题。

**第三跳：所有权怎样交接。**回到 [System 接管初始化结果](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/system/slam_system.cpp:38)：

~~~cpp
map_=std::move(initial.map); last_keyframe_=initial.keyframe;
~~~

这里用候选地图替换了构造时的空地图。局部变量 initial 离开作用域后，地图仍由 System 持有。

**第四跳：下一帧怎么找回这个点。**先看 [调用 Track 的实参](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/system/slam_system.cpp:43)，再跳 [匹配下标到地图点的转换](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/tracking/frame_tracker.cpp:42)：

~~~text
匹配结果：参考下标 7 ↔ 当前下标 12
  → ref.associations()[7] 得到 P
  → map.GetMapPoint(P).position_w() 得到 Pw
  → 当前 features[12].uv 得到像素
  → Pw ↔ uv，进入 PnP
~~~

所以衔接的关键不是“把初始两张图交给 tracking”，而是保留下来的参考关键帧、地图点和特征到点的关系。本项目 RGB-D 初始化本身只需一帧。

**停止条件：**能从“初始第 7 个特征”口述到“下一帧 PnP 的一个 3D–2D 对应”，并指出 System 中那条接管地图的语句。

**下一站自然产生的问题：**PnP 之后还有 optimization，它输出的位姿究竟在哪一步被当真？

## 第 4 站｜Optimization 怎样真正接入 Tracking？最终位姿是谁写下的？

**先拿答案：**FrameTracker 有一个 PoseOptimizer 成员。Track 和 RefineLocal 都通过内部 Optimize 调用它；优化器给候选位姿和内点标记，跟踪器判断成功，System 再把最终结果写进 Frame。

本轮按“调用点 → 接口数据 → 返回后写入”阅读，先不读雅可比和自动求导模板。

~~~text
SlamSystem::ProcessImpl
  → FrameTracker::Track
      描述子匹配 → PnP RANSAC 初值
      → FrameTracker::Optimize
          关联 ID → PoseMeasurement
          → PoseOptimizer::Solve
          ← PoseSolution：usable、tcw、inliers
          筛关联、检查跟踪所需内点数
      ← TrackingResult
  → 成功后 FrameTracker::RefineLocal
      投影局部点、补充关联
      → 同一个 Optimize → 同一个 PoseOptimizer::Solve
      ← 最终 TrackingResult
  → frame.SetPose + frame.Associate
  → 按需 mapping 后，result.tcw=frame.pose()
~~~

先看 [PnP 后调用 Optimize](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/tracking/frame_tracker.cpp:63)，然后读 [FrameTracker::Optimize](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/tracking/frame_tracker.cpp:14)。最值得停留的是：

~~~cpp
measurements.push_back({map.GetMapPoint(a.point).position_w(),k.uv,k.depth_z_m});
result.optimization=optimizer_.Solve(initial,measurements);
~~~

上面是源码中的两条语句，中间循环结构省略。它证明优化器收到的是“固定世界点、当前像素、可选深度”，不需要读图像、描述子或 Map。数据定义只看 [PoseMeasurement / PoseSolution](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/include/vslam/optimization/optimization_problem.h:14)。

再到 [PoseOptimizer::Solve](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/optimization/solvers.cpp:140)，本轮只找三件事：点被 SetParameterBlockConstant 固定；ceres::Solve 执行数值求解；通过检查才设置 usable 和候选 tcw。Reprojection、Cost、Huber 的数学细节先略过。

沿返回路径回到 Optimize，看到内点标记如何筛关联；再看 [System 真正写入 Frame](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/system/slam_system.cpp:53)。优化器通过不代表最终跟踪通过，跟踪器还要满足自己的内点门限；参考跟踪通过也不代表本帧通过，后面还有 RefineLocal。

**停止条件：**能回答“PoseOptimizer::Solve 返回 usable=true，当前 Frame.pose_ 已经改了吗？”  
答案：没有。求解器返回候选；当前正常跟踪路径的写入在 System 接受最终 TrackingResult 之后。

**可选断点：**Optimize 调用 Solve 前看 measurements.size()，返回后看 usable 和 inliers，再在 System 的 frame.SetPose 处确认真正接受。

**下一站自然产生的问题：**tracking 只改一帧，那 mapping 里的 optimization 又怎样改到整张地图？

## 第 5 站｜Optimization 怎样接入 Mapping？哪一行真正改变地图几何？

**先拿答案：**LocalMapper 先插帧建点，再把局部地图复制成 BAProblem；LocalBundleAdjuster 只计算候选副本。回到 LocalMapper 后，通过 Map::CommitGeometry 才改变地图关键帧和点的位置，之后还要同步当前 Frame。

先读 [ShouldInsert 与 mapper_.Process](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/system/slam_system.cpp:56)，确认 BA 并非每帧执行。再直接跳 [LocalMapper::Process](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/mapping/local_mapper.cpp:154)，这是本轮主文件：

~~~text
Map::InsertKeyFrame(frame)          把成功帧和跟踪关联持久化
SelectNeighbors + Extend           传播观测、创建新点
BuildProblem                       从地图复制出数值问题
LocalBundleAdjuster::Solve         候选联合优化
  usable 为真
    Map::CommitGeometry            接受的批次真正更新地图几何
      提交成功
        RemoveObservation          删除外点边
        RemoveMapPoint             清理不合格点
读取新关键帧的最终状态
  → frame.SetPose
  → 重建当前 frame 的关联
回到 System
  → 更新 last_keyframe_
  → result.tcw=frame.pose()
~~~

只有三个地方值得向下再跳：

1. [BuildProblem](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/mapping/local_mapper.cpp:115)：看它如何填 poses、points、measurements，以及 fixed、epoch、revision。点和位姿在这里从地图实体变成独立数值数组；具体排序和预算先不展开。
2. [LocalBundleAdjuster::Solve](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/optimization/solvers.cpp:176)：先看开头 result.geometry=input，末尾返回 usable，以及固定位姿不更新。它没有接收 Map，因此不会直接改真实地图。
3. [Map::CommitGeometry](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/core/map.cpp:230)：看版本/整批校验，然后找到 [关键帧与点的实际赋值](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/core/map.cpp:264)。这才是候选几何进入地图的位置。

下面两条源码就是这一轮的连接点：

~~~cpp
result.ba=ba_.Solve(BuildProblem(map,result.keyframe));
result.ba_committed=map.CommitGeometry(result.ba.geometry.epoch,result.ba.geometry.revision,poses,points).ok();
~~~

两句之间会检查 usable 并组装更新批次，不能省略这些条件直接理解成“求解后无条件写回”。

最后回到 [从 KeyFrame 同步当前 Frame](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/mapping/local_mapper.cpp:180)。为什么还要这一步？System 最后返回的是当前 Frame 的位姿；只改 Map 的 KeyFrame 而不更新 Frame，会返回旧值。

**停止条件：**不看文档也能补全：

> BuildProblem 从 ______ 取数，Solve 改 ______，CommitGeometry 改 ______，最后同步 ______。

答案：Map；独立候选几何；Map 里的关键帧与点；当前 Frame。

**额外自检：**BA 拒绝后，刚插入的关键帧和点还在吗？  
在。被拒绝的是候选几何回写及其后的清理，前面已完成的插帧建点不会整批撤销。

## 第 6 站｜丢失、恢复、Reset 和最终导出，能否用一条小故事串起来？

**先拿答案：**跟踪失败不自动清空地图；重试依靠保留的参考关键帧；Reset 才开启新世代；轨迹导出则用最终关键帧修正历史成功帧的位置。

这一轮先读测试中的“小故事”，比再看一轮类声明更容易记住：

打开 [TestSequence](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/tests/integration/rgbd_pipeline_test.cpp:53)。先把 Input 和 Features 当成测试数据生成器，只读每次 ProcessFeatures 与其后的 Check。按顺序能看到：

~~~text
空特征 → 等待初始化
已知几何的多帧 → 初始化、跟踪、插帧、至少一次 BA 提交
空特征 → Lost，无位姿，Map revision 不变
重新提供可匹配特征 → 恢复 Tracking
重复时间戳 → 错误状态，无位姿
Reset → epoch 增加 → 新一轮初始化
~~~

这是现有测试的输入和断言，不是本次新跑出来的实验结果。ProcessFeatures 与正常 Process 共用 ProcessImpl，但提供已知特征、跳过检测；适合读状态流，不等于验证了全部真实图像前端。也不要把其中每个预期姿态当作传入估计器的位姿初值，它主要用于生成合成测量和比较输出。

然后只补看两个源头：

- [System::Reset](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/system/slam_system.cpp:78)：Map Reset、重建 FrameFactory、清空 mapper 的辅助状态、清空 last_keyframe_、回到 Initializing。
- [记录成功轨迹](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/apps/run_dataset.cpp:62) 和 [最终轨迹导出](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/apps/run_dataset.cpp:86)：循环中保存相对参考关键帧的变换，结束后用该关键帧最终位姿重新组成当前位姿，取逆导出 Twc。失败帧不进入成功轨迹。

**停止条件：**能回答两个问题：

- Lost 和 Reset 的区别是什么？前者保留地图继续尝试；后者清空并进入新世代。
- 为什么已有每帧 result.tcw，还要结束后再计算一次轨迹？后续 BA 可能改变参考关键帧，需要把修正传播给之前成功的普通帧；这不是重新优化每个普通帧。

读到这里，你已经能从 main 讲到 Map 写入，再讲回最终文件。这时才值得进入局部算法细节。

## 走通以后，只选与你的实验最相关的一条支线

你原本关心真实 Mono 轨迹为何失真，以及 RGB-D 能否改善尺度问题。建议下一次沿“一个特征的 Z 到底在哪些地方起作用”追踪，而不是继续按目录翻。

**先拿答案：**深度不仅用于初始化。它还随特征进入 tracking 的位姿残差，作为持久观测进入 BA，并用于后续建点。只删掉某个深度残差，不能代表已经得到一个纯 Mono 对照组。

按下面的证据链逐个确认即可：

~~~text
Prepare 统一深度单位
  → FeatureExtractor::Extract 给 Keypoint 缓存可选 Z
  → InitializeRGBD：Unproject，建立初始米制点
  → FrameTracker::Optimize：Z 放进 PoseMeasurement
      → Cost：有 Z 为 3 维残差，无 Z 为 2 维
  → Map::AddObservation：保存该关键帧测量的 Z
      → BuildProblem：放进 BAMeasurement
      → LocalBundleAdjuster：深度参与联合优化
  → LocalMapper::Extend：也会使用深度创建新点
~~~

本轮只新增三个向下入口：[AddObservation 的测量来源](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/core/map.cpp:99)、[Reprojection 与 Cost 的深度分支](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/optimization/solvers.cpp:60)、[Extend 建点](//wsl$/Ubuntu-22.04/home/lai/project/ProjectSLAM/visual_slam_cpp/src/mapping/local_mapper.cpp:56)。前面已经看过的调用点复用即可。

这里不要求先推导 Ceres 自动求导。先能说出“深度从哪里进、在哪形成残差、在哪再次创建点”，再回 [QA 第 2 题的实验设计](QA.md) 判断对照组要控制哪些路径。当前 C++ 没有完整单目初始化，不能简单把深度图清空就认为运行了同等 Mono 管线。

## 阅读时卡住，只做这一个动作

从卡住的调用点返回上一层，用三行写下：

~~~text
调用前：我手上有什么具体对象/字段？
调用后：返回了什么，或谁的字段被修改？
继续往下：下一位使用者是哪一句代码？
~~~

例如：

~~~text
调用前：Map、当前 KeyFrame ID
调用后：BuildProblem 返回独立 poses/points/measurements
下一位使用者：ba_.Solve(...)
~~~

填得出来就先继续，不必为了一个模板写法中断整个调用链。填不出来才打开对应声明或实现，目标只是补上缺的一行。

本轮完成记录可以只留六个勾：

- [ ] 我能指出外层逐帧循环和内层单帧处理。
- [ ] 我知道长期地图、临时 Frame 和候选解分别在哪里。
- [ ] 我能追一个初始特征直到下一帧 PnP 的输入。
- [ ] 我能指出 tracking 接受并写入位姿的位置。
- [ ] 我能指出 BA 候选真正改到 Map 的位置。
- [ ] 我能解释 Lost、Reset 与最终轨迹修正。

先拿到第一个勾即可，不需要先读懂整份文档。

