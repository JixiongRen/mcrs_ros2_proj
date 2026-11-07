# 开发、构建与安装验证

本项目沿用当前 Pi 4B／ARM64／Ubuntu 24.04／Python 3.12／ROS 2 Jazzy，不升级操作系统。Python 核心安装为 `gca`；所有运行与验证都应通过新工作空间，避免扫描 legacy 中的示例和顶层硬件脚本。

## 准备环境

```bash
cd /home/pi/mcrs_ros2_proj
bash tools/bootstrap_dev.sh
```

脚本检查 `/opt/ros/jazzy`，创建允许读取系统 ROS Python 包的项目 `.venv`，安装 [固定依赖](../requirements-dev.txt)。当前系统没有 ensurepip，因此引导脚本使用 PyPI 版本元数据及 SHA256 校验的 pip wheel；不会向系统 Python 安装 SDK。Python、架构与依赖实际版本写入 `artifacts/environment.json`。

Ubuntu／ROS 提供 pytest、PyYAML、numpy、matplotlib、colcon 和 rclpy；缺失时脚本明确失败。没有管理员权限时，先让环境维护者补齐相应系统包，再执行本项目脚本。仅安装 pigpio 客户端不会启动 pigpiod，也不会启用 GPIO。

## 日常构建

```bash
cd /home/pi/mcrs_ros2_proj
source tools/common.sh
cd ros2_ws
"$GCA_PYTHON" -c 'from colcon_core.command import main; raise SystemExit(main())' build \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DPython3_EXECUTABLE=/home/pi/mcrs_ros2_proj/.venv/bin/python \
  -DPYTHON_EXECUTABLE=/home/pi/mcrs_ros2_proj/.venv/bin/python
cd ..
gca_source_install
```

使用项目 Python 调用 colcon，确保 Python entry point 与 SDK 处在同一环境。`common.sh` 设置 `CMAKE_BUILD_PARALLEL_LEVEL=1`、`MAKEFLAGS=-j1`，并限制默认 ROS 发现到本机。Pi 内存有限，五包使用串行构建；不要让多个 agent 同时写同一个 build／install。

普通安装会复制 Python 文件。修改源码后须重新 build 才能验证新代码；不要用手工 PYTHONPATH 将源码插在 install 前面掩盖安装缺陷。开发中也可选 symlink-install，但最终验证始终使用独立普通安装。

## 统一验证

```bash
bash tools/verify_software.sh
```

依次执行 Ruff、五包 build、安装环境下的全部新测试、colcon test／test-result、独立目录普通安装、工作空间外资源检查，以及真实 ROS 进程仿真。每一步记录命令、工作目录、退出码、日志和用时，第一处失败使验证返回非零。JUnit 中必需软件测试的 skipped 不能当作通过。

查看 `artifacts/verification/latest/report.json`；`pytest.xml` 是逐用例证据。干净安装保存在 `artifacts/clean-install/`，运行时只 source 官方 underlay 与本次 `local_setup.bash`，清除继承的 overlay 环境。CLI、launch 和 YAML 都从 install 查询。

定位局部错误时可以执行：

```bash
source tools/common.sh
gca_source_install
"$GCA_PYTHON" -m pytest -q ros2_ws/src/gca_core/test
"$GCA_PYTHON" -m pytest -q tests/test_ros_integration.py
"$GCA_PYTHON" -m ruff check ros2_ws/src tools tests
```

只有综合验证通过后才更新完成状态。不要用 compileall 或 import 替代 ROS 服务与 Action 联调，也不要删除失败断言换取绿色结果。

## 扩展与注释规则

领域接口用不可变类型表达单位、有效性、epoch 和期限。算法模块不能操作 GPIO／串口或 publish。新增设备实现 `domain/ports.py` 的契约，并测试真实适配代码的调用顺序、坏回执、截止时间、部分初始化与关闭；模拟后端测试不能替代这层。

控制状态只由本地控制线程写。每条总线只能有一个 I/O 所有者。跨线程只发送不可变需求或快照；不要把等待设备、用户输入或磁盘操作放进状态锁。每个队列／缓存／重试／关闭均须有容量或期限。

注释应说明为什么这样做：实际 dt、抗饱和残差、脉冲一次结算、取消终态、原始反馈年龄和停止证据。新增参数要同步 schema、硬件模板、配置文档和回归。设备型号支持需要官方控制表与明确能力检查，不能只增加一个字符串别名。

## 调试边界

默认只运行 simulation。不得 import legacy 的 main/test 脚本做“快速检查”。HIL 测试另按 [验收说明](HIL_ACCEPTANCE.md) 的资料、配置、仪器和授权执行。清理只作用于本次创建的进程组，不查杀占串口／端口的其他进程。

代码本身不代替网络认证。应用 token 管理业务控制权，原生 CancelGoal 的访问权限依赖受控主机／DDS Security。跨机部署需另行提供并验证安全策略，不能只改 ROS_DOMAIN_ID 后宣称已授权隔离。

## 生成文件的容量管理

控制记录默认4个文件、每个最多2 MB，队列满或写入失败可见计数。仿真每场景有墙钟期限与10万条记录硬上限，默认同一输出目录保存最新结果。`verify` 只轮转带本工具生成标记的普通安装目录，保留此前最近两份及当前一份；未标记目录不删除。不要把手工文件放入标记的构建输出目录。验证锁禁止两个验证进程同时写工作空间。

## Git版本管理

工程根目录是本地Git仓库，默认分支为 `main`。新版源码、测试、工具、配置模板、文档和legacy源码参考快照纳入版本管理；`.venv`、旧`.env`环境、缓存、ROS构建/安装/日志目录及 `artifacts/`由根 `.gitignore` 排除。新克隆需要重新准备环境和运行验证，历史报告不会随源码克隆。

legacy中的五个嵌套仓库按普通源码文件快照保存，不使用submodule。它们在本机已有的`.git`目录保持原位，不属于根仓库的提交内容；根忽略规则防止以后把这些目录误加成gitlink。已纳入的旧文件修改仍会被根仓库追踪，这些参考目录中的新增文件默认忽略；确需更新参考快照时应单独核对，不对整个目录执行强制添加。第三方原有许可随源码保留，详见 [许可说明](THIRD_PARTY_NOTICES.md)。

日常工作从项目根目录开始：

```bash
git status
git switch -c feature/your-change
# 完成修改与相关验证后，按实际改动选择路径。
git add ros2_ws/src tests tools docs
git diff --cached
git commit -m "Describe the change"
```

当前远程 `origin` 为 `https://github.com/JixiongRen/mcrs_ros2_proj.git`，`main` 跟踪 `origin/main`。提交经核对的源码、配置和文档；本地 `artifacts/` 中的实体标定与实机记录继续保留，不随源码推送。
