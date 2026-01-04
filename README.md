
# ROS 2 Humble for **WINDOWS** Installation
To run ROS 2 Humble on Windows 10 or 11, you must configure the **Windows Subsystem for Linux (WSL2)**. This provides a high-performance Linux kernel environment within Windows, specifically supporting **Ubuntu 22.04 LTS (Jammy Jellyfish)**.


## 💻 1. System Requirements & Windows Pre-setup

Before installing, ensure your hardware and software meet these specifications:

* **Hardware Requirements** 
* **CPU**: 64-bit CPU (x64 or ARM64).

  * **Virtualization**: Must be enabled in BIOS/UEFI.

  * **Intel**: Enable **Intel VT-x** and **Intel VT-d** if available. (Mostly Enabled)
                 
    *   To check if Virtualization is enabled/disabled on your desktop/laptop, simply press Ctrl + Shift + Esc keys to open the Task Manager. 
         Click on the Performance tab and under CPU, you will find information about Virtualization on your desktop/laptop.

* **AMD**: Enable **SVM Mode** and **AMD-Vi**.


* **Dependencies** (OPTIONAL):
  * [Microsoft Visual C++ Redistributable](https://learn.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist?view=msvc-170).
  * [.NET Framework 4.8+](https://support.microsoft.com/en-us/topic/microsoft-net-framework-4-8-offline-installer-for-windows-9d23f658-3b97-68ab-d013-aa3c3e7495e0).





---
## 2. WLS2 Installation and (Ubuntu 22.04) for windows

### ⚙️ Step 2.1: Configure Windows Features

You must activate the underlying Windows components that allow Linux to run alongside Windows.

1. Open the **Start Menu**, search for **"Turn Windows features on or off"**, and open it.


2. Select and enable the following:
* ✅ **Windows Subsystem for Linux** 


* ✅ **Virtual Machine Platform** 


* ✅ **Windows Hypervisor Platform** 




3. Click **OK** and **Restart** your computer to finalize these system changes.



---

### 🐧 Step 2.2: Ubuntu 22.04 Deployment

Once the system has restarted, deploy the Ubuntu environment using PowerShell.

1. Search for **Windows PowerShell**, right-click, and select **Run as Administrator**.


2. Install the base WSL engine:
```powershell
wsl --install

```



*(If WSL is already installed, skip this step)* .


3. Install the specific Ubuntu distribution required for ROS 2 Humble:
```powershell
wsl --install ubuntu-22.04

```
Wait till it download and installization (almost 35Min) 

4. **Finalize Setup:** An Ubuntu terminal will open. Follow the prompts to set a **Username** (e.g., `dineshkolhedkk`) and a secure **Password** (e.g., `xyz`) it is invisible.



---

### ✅ Step 2.3: Verify WSL2 Protocol

It is essential to confirm that your Ubuntu instance is running on the **Version 2** protocol to ensure compatibility with ROS 2 networking.

1. In your **PowerShell (Administrator)** window, run:
```powershell
wsl -l -v

```


2. **Validation:** Ensure the **VERSION** column displays **2** for `Ubuntu-22.04`.


* *Troubleshooting:* If it shows version 1, update it by running: `wsl --set-version Ubuntu-22.04 2`.
  

### ✅ Step 2.4: Open and Run ubuntu-22.04(bash) on windows
* Open PowerShell or Command Prompt
* Then launch your Ubuntu environment
```powershell
wsl -d Ubuntu-22.04

```

* Or just search "Ubuntu" in the Start Menu



## 🐧 3. ROS 2 Humble Installation (Ubuntu 22.04) Rember you always need to run this in terminal of ubuntu refer step 2.4 to launch 
[recomment to install from web or you can also install from below steps](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

Step 3.1: Set Locale 

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

```

Step 3.2: Setup source

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe

```
```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

```
```bash
sudo apt update
```
```bash
sudo apt upgrade
```

Step 3.3: Install ROS 2 Desktop 

```bash
sudo apt install ros-humble-desktop

```

```bash
sudo apt install ros-dev-tools

```
this both steps almost take 45min
---

## ⚙️ 4. Environment Configuration

You must load the ROS 2 setup script to use its commands.
```bash
source /opt/ros/humble/setup.bash
```
Automatic Setup (Recommended) 

Add the setup command to your `.bashrc` file to make it automatic in every terminal:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

```

* The `>>` operator appends the command to the end of the file.


* 
`source ~/.bashrc` applies the changes immediately without restarting.



Verification 

```bash
echo $ROS_DISTRO
#Output should be: humble 

```

---

*Congrulation You have sucessfully installed ROS2  for windows

## 🚀 5. Testing ROS 2 Communication (Optional)

A. Talker and Listener Demo 

This demonstrates nodes communicating via a **Topic** called `/chatter`.

1. **Terminal 1 (Talker)**: `ros2 run demo_nodes_cpp talker` 


2. **Terminal 2 (Listener)**: `ros2 run demo_nodes_cpp listener` 



B. Turtlesim (GUI Simulator) 

1. **Terminal 1 (Simulator)**: `ros2 run turtlesim turtlesim_node` 


2. **Terminal 2 (Controller)**: `ros2 run turtlesim turtle_teleop_key` 


* 
*Use the arrow keys to move the turtle*.





