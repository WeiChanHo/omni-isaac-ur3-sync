# Ubuntu 開發環境

本文件記錄目前 Ubuntu 工作站上的 Isaac Sim、Python、ROS 2 與 GPU 環境。資訊於 2026-08-24 透過本機指令確認。

## 系統資訊

| 項目 | 值 |
| --- | --- |
| OS | Ubuntu 24.04.4 LTS |
| Architecture | x86_64 |
| GPU PCI device | NVIDIA TU102GL（Quadro RTX 6000/8000） |
| NVIDIA Driver package | **nvidia-driver-580** 580.173.02 |
| NVIDIA utilities | **nvidia-utils-580** 580.173.02 |

> `lspci` 可辨識 NVIDIA GPU，且 Driver 套件已安裝；但本次查詢時 `nvidia-smi` 無法與 NVIDIA Driver 通訊，因此 GPU Runtime、VRAM 與 CUDA 執行狀態尚未完成即時驗證。

## Isaac Sim

目前使用解壓縮至家目錄的 Isaac Sim 獨立安裝版，不是 Conda 或 pip 安裝。

| 項目 | 值 |
| --- | --- |
| 安裝路徑 | `/home/spatiallabs/isaacsim` |
| 啟動指令 | `/home/spatiallabs/isaacsim/isaac-sim.sh` |
| Python 啟動器 | `/home/spatiallabs/isaacsim/python.sh` |
| Isaac Sim version | 6.0.1 |
| 完整 build version | `6.0.1-rc.7+release.42383.32955d8d.glx86_64` |
| 內建 Python version | 3.12.13 |

可使用以下指令重新確認版本：

~~~bash
sed -n '1p' /home/spatiallabs/isaacsim/VERSION
/home/spatiallabs/isaacsim/python.sh --version
~~~

## PyTorch 與 CUDA

Isaac Sim 內建 Python 目前無法匯入 `torch`，表示此 Python 環境沒有可用的 PyTorch 套件。因此下列資訊目前無法由 PyTorch 查詢：

| 項目 | 狀態 |
| --- | --- |
| PyTorch version | 未安裝 |
| CUDA available | 無法透過 PyTorch 驗證 |
| PyTorch CUDA version | 不適用 |
| GPU Runtime | `nvidia-smi` 目前無法驗證 |

若之後安裝 PyTorch，可使用以下指令確認：

~~~bash
/home/spatiallabs/isaacsim/python.sh -c \
  "import torch; print('PyTorch:', torch.__version__); print('CUDA available:', torch.cuda.is_available()); print('CUDA:', torch.version.cuda); print('GPU:', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'N/A')"
~~~

## ROS 2 與 Universal Robots 套件

| 項目 | 值 |
| --- | --- |
| ROS distribution | Jazzy |
| ROS 安裝路徑 | `/opt/ros/jazzy` |
| `ur_robot_driver` | 已安裝 |
| `ur_moveit_config` | 已安裝 |
| `ur_description` | 已安裝 |

確認指令：

~~~bash
source /opt/ros/jazzy/setup.bash
echo "$ROS_DISTRO"
ros2 pkg prefix ur_robot_driver
ros2 pkg prefix ur_moveit_config
~~~
