# Location类功能解析

## 1. 类初始化（构造函数）
- 调用`loadParameters()`加载传感器参数
  - 雷达到IMU距离（lidarToIMUDist_，默认1.87m）
  - 前后轮轴到IMU的三维距离（默认0.0m）
- 初始化点云存储`cloud.reset(new pcl::PointCloud<pcl::PointXYZ>)`
- 创建ROS订阅者
  - `/INS/ASENSING_INS`（惯性导航数据）→ 回调`doINSMsg`
  - `/cone_position`（锥筒位置）→ 回调`doConeMsg`
- 创建ROS发布者
  - 车辆状态：`/Carstate`
  - 锥筒地图：`/coneMap`、`/globalMapOnly`
  - 可视化标记：`/carBody`、`/whole`等

## 2. 参数加载（loadParameters）
- 从ROS参数服务器读取配置参数
- 关键参数列表：
  - 传感器间距：lidarToIMUDist_、frontToIMUdistanceX_等
  - 订阅话题名：subTopic_（默认/location）
- 读取失败时使用默认值并输出警告

## 3. INS消息处理（doINSMsg）
- 解析惯性导航数据
  - 速度：east_velocity、north_velocity、ground_velocity
  - 姿态：azimuth（方位角）、角速度、加速度
- 首次接收处理
  - 初始化基准经纬度（first_lat、first_lon）
  - 计算初始速度（V线速度、W角速度、A加速度）
  - 设置isfirstINSreceived=true
- 非首次接收处理
  - 计算方位角变化量diff
  - 转换角度为弧度并处理边界（±π）
  - 调用GeoDetic_TO_ENU()转换坐标
- 辅助操作
  - 计算方向向量：calcVehicleDirection()
  - 可视化：visCar()、visWhole()、visTrack()
  - 发布车辆状态消息

## 4. 坐标转换（GeoDetic_TO_ENU）
- 经纬度转ECEF坐标
  - 基于参考椭球参数（a=6378137、b=6356752.3142）
  - 计算站点与原点的ECEF坐标
- ECEF转ENU坐标
  - 东向：-sin_lon0*xd + cos_lon0*yd
  - 北向：(-cos_lon0*xd - sin_lon0*yd)*sin_lat0 + cos_lat0*zd
- 更新车辆位置
  - 计算前后轮位置（front_wheel、rear_wheel）
  - 通过tf2变换更新Carstate
  - 发布车辆状态并保存坐标

## 5. 锥筒消息处理（doConeMsg）
- 数据校验
  - 未接收INS数据时警告返回
  - 锥筒数据为空时警告返回
- 坐标转换
  - 从雷达坐标系转换到全局坐标系
  - 基于车辆位置和雷达偏移量
- 地图更新逻辑
  - 首次接收：分配新ID，添加到点云和地图
  - 非首次接收：
    - KD树搜索最近邻锥筒
    - 距离≤2.5m：更新现有锥筒位置（平均滤波）
    - 距离>2.5m：分配新ID，添加新锥筒
- 发布更新
  - 发布地图消息和点云消息

## 6. 可视化功能
- visCar()：发布车身轮廓（线框）到/carBody
- visWhole()：发布四个车轮（圆柱体）到/whole
- visCone()：发布锥筒（圆柱体）到/coneMarker
- visTrack()：发布车辆轨迹（线）到/trackMarker

## 7. 数据保存功能
- saveGPS()：保存经纬度到/testData/gps.txt
- saveCarstate()：保存车辆ENU坐标到/testData/carstate.txt
- savePoint()：保存锥筒坐标及ID到/testData/point.txt
- 其他：角度变化、速度、加速度等数据保存