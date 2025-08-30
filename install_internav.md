
1. install ubuntu 22.04

2. install gcc 12

(MUST do this!!!)
sudo apt install -y build-essential

sudo apt install gcc-12

sudo ln -s /user/bin/gcc-12 /usr/bin/gcc

sudo apt install -y g++-12

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
