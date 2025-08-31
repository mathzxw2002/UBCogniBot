
1. install ubuntu 22.04

update softwares by Ubuntu Software Updates

2. install gcc 12

(MUST do this!!!)
sudo apt install -y build-essential gcc-12 g++-12

sudo ln -s /user/bin/gcc-12 /usr/bin/gcc

sudo ln -s /user/bin/g++-12 /usr/bin/g++


3. install nvidia gpu driver 550+ and cuda 12.4

3.1 disable  nouveau:
sudo vim /etc/modprobe.d/blacklist-nouveau.conf

add (at the end of /etc/modprobe.d/blacklist-nouveau.conf):
blacklist nouveau
options nouveau modeset=0

3.2 update config and reboot

sudo update-initramfs -u
sudo reboot


4. install cuda 12.4

(change to tty mode)
<img width="1106" height="733" alt="image" src="https://github.com/user-attachments/assets/c2f491a3-bb9b-459e-9735-3de014d33722" />




wget https://developer.download.nvidia.com/compute/cuda/12.4.0/local_installers/cuda_12.4.0_550.54.14_linux.run
chmod +x cuda_12.4.0_550.54.14_linux.run
sudo sh cuda_12.4.0_550.54.14_linux.run

vim ~/.bashrc

export PATH=/usr/local/cuda-12.4/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.4/lib64:$LD_LIBRARY_PATH

source ~/.bashrc




https://blog.csdn.net/ZH13114130815/article/details/141021282



4. 


# Bug

./isaac-sim.selector.sh 
[Info] [carb] Logging to file: /home/sany/.nvidia-omniverse/logs/Kit/Isaac-Sim_App_Selector/4.5/kit_20250831_092041.log
[0.646s] [ext: omni.kit.async_engine-0.0.1] startup
[0.755s] [ext: omni.client.lib-1.0.0] startup
[0.822s] [ext: omni.stats-1.0.1] startup
[0.824s] [ext: omni.client-1.2.2] startup
[0.833s] [ext: omni.gpu_foundation.shadercache.vulkan-1.0.0] startup
[0.850s] [ext: omni.assets.plugins-0.0.0] startup
[0.852s] [ext: omni.gpu_foundation-0.0.0] startup
[0.866s] [ext: carb.windowing.plugins-1.0.0] startup
[0.945s] [ext: omni.kit.renderer.init-0.0.0] startup
X Error of failed request:  GLXBadFBConfig
  Major opcode of failed request:  152 (GLX)
  Minor opcode of failed request:  0 ()
  Serial number of failed request:  236
  Current serial number in output stream:  236


  # 安装 OpenGL 测试工具
sudo apt install -y mesa-utils
# 测试 OpenGL 渲染（需在图形界面环境执行）
glxinfo | grep "OpenGL renderer"


正常输出应包含 NVIDIA 字样（如 NVIDIA GeForce RTX 3090/PCIe/SSE2）；
若输出 llvmpipe（软件渲染），说明未使用 NVIDIA 硬件加速，需修复驱动。


# 生成NVIDIA Xorg配置文件
sudo nvidia-xconfig

sudo nvidia-xconfig

WARNING: Unable to locate/open X configuration file.

Package xorg-server was not found in the pkg-config search path.
Perhaps you should add the directory containing `xorg-server.pc'
to the PKG_CONFIG_PATH environment variable
No package 'xorg-server' found
New X configuration file written to '/etc/X11/xorg.conf'

sudo add-apt-repository universe
sudo apt update -y

sudo apt install -y xorg-dev




