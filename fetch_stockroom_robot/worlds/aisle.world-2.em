<?xml version="1.0" ?>
<sdf version="1.4">
<world name="aisle">
<gui>
  <!-- 设置观测视角，这样就不用在每次启动仿真时都手动调整视角了 -->
  <camera name="camera">
    <pose>3 -2 3.5 0.0 .85 2.4</pose>
    <view_controller>orbit</view_controller>
  </camera>
</gui>
<scene>
  <ambient>0.65 0.65 0.65 1</ambient>
  <background>0.78 0.82 0.88 1</background>
</scene>
<light type="directional" name="sun">
  <cast_shadows>true</cast_shadows>
  <pose>0 0 10 0 0 0</pose>
  <diffuse>1.0 1.0 1.0 1</diffuse>
  <specular>0.35 0.35 0.35 1</specular>
  <attenuation>
    <range>1000</range>
    <constant>0.9</constant>
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
  </attenuation>
  <direction>-0.45 0.2 -0.87</direction>
</light>
<model name="ground_plane">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
    </collision>
    <visual name="visual">
      <cast_shadows>false</cast_shadows>
      <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
      <material>
        <ambient>0.72 0.72 0.72 1</ambient>
        <diffuse>0.72 0.72 0.72 1</diffuse>
        <specular>0.08 0.08 0.08 1</specular>
      </material>
    </visual>
  </link>
</model>

<!-- 生成左右对称的两列存储仓，每列６个存储仓 -->
@{from numpy import arange}@
@{bin_count = 0}
@[for side in ['left','right']]
  @[if side == 'left']
    @{y = -1.5}
    @{yaw = 3.1415}
  @[else]
    @{y = 1.5}
    @{yaw = 0}
  @[end if]
  @[for x in arange(-1.5, 1.5, 0.5)]
  <!-- 存储仓 -->
    <include>
      <name>bin_@(bin_count)</name>  <!-- 给每个隔间构造一个唯一的模型名称 -->
      <pose>@(x) @(y) 0.5 0 0 @(yaw)</pose>
      <!-- 位置变量x和y决定了每个隔间所在的位置，通过改变x的步长，可以改变隔间的间距和分布 -->
      <uri>model://storage_bin</uri>
      <!-- 引用前面构造好的单个存储仓模型，需要设置好GAZEBO_MODEL_PATH环境变量 -->
    </include>
    
    <!-- 存储仓对应的标签 -->
     <!-- 构造一个扁box, 用来给存储仓贴上ALVAR图 -->
    <model name="bin_@(bin_count)_tag">
      <static>true</static>
      <pose>@(x) @(y*1.125) 0.63 0 0 @(yaw)</pose>
      <!-- ALVAR码贴在存储仓的背板上 -->
      <link name="link">
        <visual name="visual">
          <geometry><box><size>0.2 0.01 0.2</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>
        </visual>
      </link>
    </model>
    @{bin_count += 1}
  @[end for]
@[end for]

</world>
</sdf>