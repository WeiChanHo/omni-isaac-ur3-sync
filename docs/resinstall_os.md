# Ubuntu 24.04 重灌與開發環境安裝

本文件記錄工作站從零開始安裝 Ubuntu、NVIDIA Driver、ROS 2、UR ROS 2 Driver 與 Isaac Sim 的流程。

## 已知環境資訊

| 項目 | 確認資訊 |
| --- | --- |
| OS | Ubuntu 24.04 |
| Architecture | x86_64 |
| GPU | NVIDIA Quadro RTX 6000 |
| VRAM | 24 GB |
| NVIDIA Driver | 580.173.02 |
| `nvidia-smi` CUDA compatibility | 13.0 |
| ROS 2 | Jazzy |
| UR ROS 2 Driver | Binary installation |
| UR package | `ros-jazzy-ur` |
| Robot | Universal Robots UR3 |
| Robot generation | CB3 |
| PolyScope | 3.15.4.106291 |
| Robot IP | `192.168.56.101` |
| PC Ethernet IP | `192.168.56.1` |
| Calibration file | `/home/spatiallabs/my_robot_calibration.yaml` |
| Isaac Sim | 6.0.1 |
| Isaac Sim platform | Linux x86_64 |
| Isaac Sim path | `~/isaacsim` |
| Isaac Sim ROS status | 已成功看到 `rclpy loaded` |

## 1. 更新系統與安裝 NVIDIA Driver

更新套件索引並確認 GPU 狀態：

```bash
sudo apt-get update
nvidia-smi
```

安裝 NVIDIA utilities：

```bash
sudo apt install nvidia-utils-580
```

接著開啟 **Software & Updates > Additional Drivers**，確認使用 `nvidia-driver-580`。安裝完成並重新啟動後，再執行 `nvidia-smi` 確認 Driver 已正常載入。

可透過 `nvidia-settings` 將 GPU 設為 Performance Mode：

```bash
nvidia-settings
```

## 2. 設定有線網路

將工作站的 Ethernet IPv4 位址設為：

```text
192.168.56.1
```

UR3 控制器的 IP 位址為 `192.168.56.101`。設定完成後，確認兩端位於相同子網路，並測試連線。

## 3. 安裝 ROS 2 Jazzy

依照官方 **ROS 2 Jazzy Ubuntu (deb packages)** 文件完成安裝，
[ROS 2 Jazzy Ubuntu (deb packages)_system-setup](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html#system-setup)
[ROS 2 Jazzy Ubuntu (deb packages)_install-ros-2](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html#install-ros-2)

之後執行：

```bash
source /opt/ros/jazzy/setup.bash
```

## 4. 安裝 UR ROS 2 Driver

先安裝 Git 與 Jazzy 版 UR 套件：

```bash
sudo apt install git -y
sudo apt install ros-jazzy-ur
```

安裝與設定時請參考：

- [Universal Robots ROS 2 Driver（Jazzy）](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/jazzy)
- [UR Client Library 安裝文件](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/installation.html)

相關套件包括：

- `ur_client_library`
- `ur_description`
- `ur_robot_driver`

## 5. 安裝 Isaac Sim 6.0.1

1. 下載 Linux x86_64 版本的 Isaac Sim 6.0.1。
2. 將 ZIP 解壓縮至家目錄。
3. 將解壓縮後的資料夾命名為 `isaacsim`。
4. 進入資料夾並執行安裝後處理：

   ```bash
   cd ~/isaacsim
   ./post_install.sh
   ```

5. 啟動 Isaac Sim：

   ```bash
   ./isaac-sim.sh
   ```

## 6. 確認 Isaac Sim ROS 2 Extension

在 Isaac Sim 中開啟 **Window > Extensions**，搜尋 ROS 2，確認相關 Extension 已安裝並啟用。

啟動後若日誌出現以下訊息，表示 `rclpy` 已成功載入：

```text
rclpy loaded
```

> **下一步：** 先以 UR Mock Hardware 完成整合測試，再連接實體手臂。
