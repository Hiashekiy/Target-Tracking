---


---

<h1 id="target-tracking-system-目标跟踪系统"><span class="prefix"></span><span class="content">Target-Tracking System 目标跟踪系统</span><span class="suffix"></span></h1>
<p>本项目建立了一个完善的<strong>无人机目标跟踪系统</strong>，包括未知环境下<strong>障碍地图</strong>的建立、无人机<strong>跟踪轨迹</strong>的生成以及<strong>跟踪目标的识别与定位</strong>，使用Px4作为无人机的底层控制，实现从算法到硬件控制的全部跟踪流程，并且最终在<strong>XTDrone平台</strong>上进行了全面的仿真验证。</p>
<h1 id="演示视频"><span class="prefix"></span><span class="content">演示视频</span><span class="suffix"></span></h1>
<p><a href="https://youtu.be/Kgru534RLyE"> XTdrone仿真验证演示</a></p>
<p align="center"> 
<img width="640" height="360" src="">
</p>
# 运行
### 1、依赖项
1、XTDrone平台（XTDrone官方配置文件：https://www.yuque.com/xtdrone/manual_cn/basic_config）
2、Anaconda
3、Yolov8
<h3 id="2、环境配置"><span class="prefix"></span><span class="content">2、环境配置</span><span class="suffix"></span></h3>
<pre><code>sudo apt-get install libarmadillo-dev ros-melodic-nlopt
</code></pre>
<p>使用到的python库有：</p>
<pre><code>matplotlib == 2.2.5
numpy == 1.16.6
rospy == 1.14.13
tf == 1.12.1
pyquaternion == 0.9.9
</code></pre>
<p>上面是测试通过的python库版本，如果你运行其他版本出错，建议更换成上述对应版本</p>
<h3 id="3、下载程序"><span class="prefix"></span><span class="content">3、下载程序</span><span class="suffix"></span></h3>
<pre><code>cd catkin_ws/src
git clone https://github.com/Hiashekiy/Target-Tracking.git
cd .. &amp;&amp; catkin_make
</code></pre>
<h3 id="4、赋予python文件执行权限"><span class="prefix"></span><span class="content">4、赋予python文件执行权限</span><span class="suffix"></span></h3>
<pre><code>cd ~/catkin_ws/src/Target-Tracking/target_tracking/scripts
sudo chmod -R 775 .
cd ~/catkin_ws/src/Target-Tracking/yolov8_ros/yolov8_ros/scripts
sudo chmod +x *.py
</code></pre>
<h3 id="5、将无人机模型复制到px4模型目录下"><span class="prefix"></span><span class="content">5、将无人机模型复制到PX4模型目录下</span><span class="suffix"></span></h3>
<pre><code>cd ~/catkin_ws/src/Target-Tracking/target_tracking/models
cp -r iris_stereo_camera ~/PX4_Firmware/Tools/sitl_gazebo/models
</code></pre>
<h3 id="6、运行"><span class="prefix"></span><span class="content">6、运行</span><span class="suffix"></span></h3>
<pre><code>roslaunch target_tracking outdoor.launch
roslaunch target_tracking target_tracking.launch
</code></pre>
<h3 id="7、启动yolov8"><span class="prefix"></span><span class="content">7、启动yolov8</span><span class="suffix"></span></h3>
<pre><code>conda activate yolov8  # 取决于你的yolov8环境名字
roslaunch yolov8_ros yolov8_launch_iris.launch
</code></pre>
<h3 id="8、控制目标运动"><span class="prefix"></span><span class="content">8、控制目标运动</span><span class="suffix"></span></h3>
<pre><code>cd ~/catkin_ws/src/Target-Tracking/target_tracking/scripts/xtdrone
python actor_control.py 0 120 0 2
</code></pre>
<p>python脚本后四个数分别是actor编号、期望x坐标、期望y坐标和行进速度</p>
<h1 id="总结"><span class="prefix"></span><span class="content">总结</span><span class="suffix"></span></h1>
<p>本项目主要是使用fast planner和yolov8来实现目标识别与追踪，可以实现目标复杂轨迹运动下的跟踪。但目前仍然存在很多问题，按照最开始的设想，是希望使用目标检测算法和目标跟踪算法相互配合来完成目标定位的，但本人对目标跟踪算法方面研究较少，所以最终只是使用了连续的目标检测来代替目标跟踪算法。其次，缺少合适的目标轨迹预测算法。本项目的无人机跟踪是基于目标的当前位置来对无人机未来的行为进行决策，存在一定的滞后性，遇到高速运动的目标时容易出现目标丢失的情况。未来工作希望能使用合适的预测技术来对目标的轨迹进行预测，以减少滞后性。如果你对此项目感兴趣的话，可以通过邮箱zizheng127@gmail.com联系我了解细节。</p>

