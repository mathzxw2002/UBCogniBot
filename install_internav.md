
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
sudo nvidia-xconfig \

sudo nvidia-xconfig

WARNING: Unable to locate/open X configuration file.

Package xorg-server was not found in the pkg-config search path. \
Perhaps you should add the directory containing `xorg-server.pc' \
to the PKG_CONFIG_PATH environment variable \
No package 'xorg-server' found \
New X configuration file written to '/etc/X11/xorg.conf' \

sudo add-apt-repository universe \
sudo apt update -y

sudo apt install -y xorg-dev \

切换为hdmi接口，不能用VGA



<img width="1980" height="288" alt="90ecb934bfe5a91b24f133fc441ef8a7" src="https://github.com/user-attachments/assets/1f58248b-2a98-4cbd-b75c-aa2fa2d911f4" />



这个提示是关于 IOMMU（输入输出内存管理单元） 的警告：
核心意思
如果你的系统是物理机（直接安装的 Linux，不是虚拟机）：Linux 下的 CUDA 和显卡驱动，不支持 “开启 IOMMU 的 PCIe 点对点内存复制”，继续开着可能导致图像出错、程序崩溃。
如果是带 GPU 直通的虚拟机（比如用 vGPU 把显卡给虚拟机用）：反而需要开启 IOMMU。
怎么处理
先点提示框右下角的「OK」关掉它。
确认自己的系统类型：
物理机：重启电脑，进 BIOS 设置（按主板提示的键，比如Del/F2），找到 Intel VT-d（Intel 平台）或 AMD-Vi（AMD 平台）的选项，把它禁用，保存后重启系统。
虚拟机（且用了 GPU 直通）：保持 IOMMU 开启即可，这个警告可以忽略。



不同品牌和型号的主板进入 BIOS 以及禁用 Intel VT-d 或 AMD-Vi 的操作可能会有所不同，但一般可以按照以下步骤进行：
进入 BIOS 设置界面：重启计算机，在启动时按下特定的键以进入 BIOS（或 UEFI）设置界面。常见的按键有 F2、Del、Esc 或 F10 等，具体按键可以查阅主板手册或在开机时留意屏幕上的提示。
查找虚拟化相关选项：在 BIOS 界面中，找到与虚拟化相关的选项。通常可以在 “Advanced”（高级）、“CPU Configuration”（CPU 配置）、“Chipset”（芯片组）等菜单下找到。
禁用虚拟化支持：找到 “Intel Virtual Technology”（英特尔虚拟化技术）、“VT-x”、“AMD-V” 或 “SVM Mode”（安全虚拟机模式）等选项，将其设置为 “Disabled”（禁用）。
保存并重启计算机：按下 F10 键保存更改并退出 BIOS 界面，计算机会重启以应用新的设置。
以下是一些常见主板品牌的具体操作示例：
华硕主板：进入 BIOS 页面后，找到 “Advanced”--“CPU Configuration”--“Intel Virtual Technology” 选项，将其设置为 “Disabled”。
微星主板：进入 BIOS 页面后，找到 “OC” 或 “Overcloking”--“CPU 特征”--“SVM Mode” 或 “Intel 虚拟化技术” 选项，将其设置为 “Disabled”。
技嘉主板：进入 BIOS 页面后，找到 “BIOS Features”--“Intel Virtual Technology” 和 “VT-d” 选项，将它们都设置为 “Disabled”。
AMD 平台：如果使用 AMD 的 CPU，进入 BIOS 页面后，找到 “M.I.T”--“Advanced Frequency Settings”--“Advanced CPU Core Settings”--“SVM” 选项，将其设置为 “Disabled”。



