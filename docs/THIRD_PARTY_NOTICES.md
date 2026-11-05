# 第三方依赖、来源与许可记录

本工程将外部库作为依赖安装，没有把legacy中的SDK副本复制进新版运行包。以下记录针对当前核对版本；实际环境版本见 `artifacts/environment.json`，补充依赖见 `requirements-dev.txt`。本文不替代第三方随包许可证，也不为历史自研文件补造授权。

## 设备依赖

| 项目/固定版本 | 来源 | 核对的许可信息 | 使用方式 |
|---|---|---|---|
| dynamixel-sdk 4.1.0 | [PyPI发行页](https://pypi.org/project/dynamixel-sdk/4.1.0/)、[官方源码](https://github.com/ROBOTIS-GIT/DynamixelSDK) | Apache-2.0；安装wheel含 `dist-info/licenses/LICENSE.txt`，源码头也有声明 | 导入SDK；自己的限时port/严格响应子类不重写SDK协议 |
| pigpio 1.78 | [PyPI发行页](https://pypi.org/project/pigpio/1.78/)、[官方源码](https://github.com/joan2937/pigpio) | 发布元数据为 `unlicense.org`，按该发行包及上游许可声明核对 | 导入Python客户端；不分发daemon二进制，不复制库源码 |
| pyserial 3.5 | [PyPI发行页](https://pypi.org/project/pyserial/3.5/)、[官方许可正文](https://github.com/pyserial/pyserial/blob/master/LICENSE.txt) | BSD，官方正文为三条条件；分发时保留适用版权、条件及免责 | 导入串口库；可能由Ubuntu系统包提供同版本 |

ROBOTIS许可证原文见 [官方LICENSE](https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/master/LICENSE)。升级任何固定依赖前需核对实际发行内容与API，重跑适配器测试；pigpio的受限socket初始化使用固定实现内部结构，尤须审查。

## 系统与开发依赖

ROS 2 Jazzy、rclpy、rosidl、ament/colcon以及消息包来自系统ROS环境；Python、pytest、PyYAML、NumPy、Matplotlib和其它工具按安装环境记录版本，格式/静态工具ruff版本由requirements固定。每个包保留其自身版权和许可证，不能把“ROS”统称为一个覆盖所有包的许可证。

本项目不把系统库源码、字体或二进制重新打包进工程。若后续发布包含第三方内容的容器、wheel、安装镜像或二进制，发布者需依据实际包含物收集对应许可证和声明；不能仅以本表代替分发清单。开发环境准备和运行版本的可复现信息由环境记录与工具输出提供。

## Legacy边界

`legacy/SteeringGearControl/DynamixelSDK/LICENSE`及若干SDK源码头有Apache-2.0声明；这不代表其它legacy自研代码都采用相同许可。

`legacy/Magnetically controlled guide wire thruster/LICENSE`在本次检查时包含项目介绍/平台模板内容，而非明确的开源授权正文。不能仅凭文件名把这些代码重新标为Apache、MIT或其它许可证。新版项目代码按领域契约重写，legacy用于设备线索与缺陷追溯，不进入运行时导入路径；保留原目录，不修改历史许可。

本项目新自研包暂用 `LicenseRef-Project-Unspecified`标识“项目授权尚未明确”，不等于授予一项开源许可。许可证确定应由项目权利人完成；不能为通过打包检查而虚构授权。

## 验证许可证与安装版本

以下命令只读取本地元数据，不连接设备。从工程根执行：

```bash
.venv/bin/python - <<'PY'
from importlib.metadata import metadata, version
for name in ('dynamixel-sdk', 'pigpio', 'pyserial'):
    info = metadata(name)
    print(name, version(name), info.get('License'))
PY
```

若版本、来源或许可证字段与本记录不符，先保存实际元数据并核对对应发行包；不得静默沿用旧声明。
