1,系统运行环境：ubuntu14.04，gcc，g++默认的4.8；其他环境下的依赖库问题请用户自行解决
  适用用户：用户需要有一定的linux下C++的开发经验。
2,安装libusb
 sudo apt-get install libusb-1.0-0-dev
 安装Qt4工具
 sudo apt-get install qt4-dev-tools qt4-doc qt4-qtconfig qt4-demos qt4-designer
 参考网址： 
 http://blog.csdn.net/u014304293/article/details/39322707
 进入cyusb_linux_1.0.4目录
 make clean;make
 chmod 777 ./install.sh
 sudo ./install.sh

2,运行环境依赖库
  1)opencv2.4.9
  参考网址：http://blog.csdn.net/surgewong/article/details/39078251
  2)pangolin
  pangolin-master创建build目录
  然后cmake ..
  sudo make install
  3)eigen3.3 需要cmake3.0以上版本
  参考网址：http://eigen.tuxfamily.org/index.php?title=Main_Page（官网）
  下载eigen3.3 编译安装。

3,编译步骤
  1)请在目录中CMakeList.txt中配置各个依赖库的路径
  2)编译命令
    mkdir build
    cd build
    cmake ..
    make
4,文件说明
  Sensor_Binocular-HPBCV1.0.0   			sensor的驱动，demo为测试程序
  Sensor_calibration-HPBCV1.0.0_for_user		运行demo生成标定文件，我们的标定文件事先写在sensor的EEPROM中，运行算法SDK时要先将标定文件configStereo.yaml生成。 

 HPslam_test 						为hpslam测试demo程序。

商务合作请联系：
远形时空 ：李昌檀 
邮箱：lict@hyperci.com

技术支持
远形时空 ：赵珂 
邮箱：zhaoke@hyperci.com
  

