# Goal prompt：实现手柄控制GCA及可控性模拟验证

以下正文用于启动下一轮工程实现。当前保存本文件不代表该goal已经执行。

---

你正在 `/home/pi/mcrs_ros2_proj` 工作。请使用多个agent讨论、分工实现并交叉审查，依据 `docs/TELEOP_IMPLEMENTATION_PLAN.md` 和 `docs/TELEOP_DESIGN.md`，实际完成Xbox式游戏手柄控制GCA轴向推进与绕轴旋转的功能模块，以及可复现的可控性模拟验证。交付可构建、可安装、可运行的代码、工具、中文操作文档及真实报告；不要以再次写方案、空节点、演示伪代码或绕过正式运行链路的测试代替实现。

## 一、不可改变的用户交互

- 左摇杆上下LY：推进/后退速度；左摇杆左右LX：正向/反向旋转速度。
- LT：按住才允许GCA双轴运动，松开立即请求STOP；LT不等于ARM，不用作速度倍率。
- B：优先软件STOP；B按住不运动，松开不自动恢复。
- RT、RB、右摇杆及其按下RS预留给磁场控制，不产生GCA运动、ARM、参考或模式切换请求。LB与其它未分配键保持未分配。
- LY/LX独立死区和幅度映射，斜向允许双轴同时运动；首期固定axis_mask=3。旋转参考失效时锁定整个双轴手柄模式，不自动降级单轴。
- 启动、重连、停止、输入过期、会话/参考变化后，均须LT释放且左杆回中，然后新一次LT按下；不自动ARM、设零、RESET或恢复旧速度。
- 普通摇杆回中可受控减速，不能追赶历史位移或补发积压脉冲。LT/B保持既有立即输出撤销与abort语义，不默认改为等待波形排空。

## 二、先读取事实并约定接口

先读适用AGENTS.md，以及当前的TELEOP两份设计文档、ARCHITECTURE、ROS_INTERFACES、USER_GUIDE、CONFIGURATION_AND_CALIBRATION、SIMULATION_AND_TESTING、HIL_ACCEPTANCE、IMPLEMENTATION_STATUS；核对实际源代码和安装API。保留用户已有未提交修改与嵌套legacy历史，不导入或运行legacy硬件脚本。

规划时只读确认的设备为USB `20bc:506d`、`BEITONG AXIULUO3SNS BFM DONGLE`，gamepad曾为js0/event5、另有Mouse/event4；js0当前用户可读，event5不可读。JSIOCG得8轴16键，但实体LY/LX/LT/B映射、极性、范围与无线失联行为尚未验证。以上只是定位线索，必须重新发现并核对，不能硬编码设备号或假定微软Xbox映射。

首期使用native joydev，只读/nonblocking/poll和Python标准库，不启动SDL震动、不改全局输入权限，不同时维护第二套evdev/SDL控制栈。未来后端可以通过明确端口替换。设备选择必须校验身份/能力；多个候选拒绝歧义选择。

多agent分工至少包含输入/映射、核心控制/模拟、ROS/session/集成，由主agent先固定输入质量、STOP屏障、有限边沿时序及文件写入范围，再并行实现。由未主写模块的agent复审停止竞争、无线健康语义、真实适配器时序及独立plant真实性；不能用agent口头结论代替测试。所有colcon构建由一个执行者串行运行。

## 三、本轮授权边界

允许读取实际手柄、采集按键、执行输入设备资格检查，以及用实体手柄或合成输入驱动simulation。默认不连接真实GCA串口、不设置GPIO、不启动pigpiod、不输出电机/磁场或震动，不更改系统/ROS发行版。只读手柄允许用户按向导操作，但不能伪造用户已按键、已拔出或已关机。

不要因缺实机映射或无线资格停下全部软件开发。完成合成输入、真实ROS进程和模拟执行器的自动验证，交付可运行的只读向导；需要用户操作而未执行的实体项目如实标NOT_RUN。软件不能用未经确认的物理参数作为hardware默认值。

## 四、先修现有Jog缺口

方案JF01–JF07必须落实为失败回归与修复，包括：

1. 在唯一ControlRuntime内增加外部编码器速度PI，手柄输入是速度意图。速度前馈、有效新采样的真实dt、独立增益量纲、最终限幅/边界、抗饱和和量化残差一次结算完整实现；重复快照不重复积分。不得把旧定位PID增益直接当速度增益。回中后不能追赶虚拟位置或产生积分爬行。
2. 修复旋转每段从confirmed角度重新舍入导致细动丢失：实现连续受限相位、小数余量、planned/reserved/confirmed分离。按确认和预约端点检查限位；换向经过零速，不能继续旧方向的未提交积压。
3. 首次进入、长空闲后再入和确认停止后显式重建速度/时间初态，过长dt不得放大首次加速预算或触发密集追赶。
4. 增加有限逐边沿时序：PulseSegment可在末尾增加可选不可变edge_offsets_us，默认None兼容旧调用；只升级Jog旋转规划、SimPulseEngine与pigpio有限波形实现。profile按真实边沿时刻执行，不能忽略后回退为均匀脉冲。验证边沿数、整型时间量化、最小高低电平、DIR建立/保持、总时长与完整期限；用独立相位预期检查首次/反向的离散速度与加速计划。既有有限段、代次、计数和未知中止语义保留。
5. 所有输入期限贯穿到I/O的速度需求和有限波形整段；控制线程冻结时也不能用now+TTL延长原输入的有效期。
6. LT/B中止计数未知时如实使旋转估计/参考失效；提供同一owner下的显式停止、DISARM、必要时RESET、用户参考确认、ARM和新LT周期恢复流程。

已有探索位于 `artifacts/teleop_planning/core_experimental.json` 和 `core_experiment.py`，只作定位和回归线索，不能作为新增正式功能已通过的证据。

## 五、真实输入与合成输入共用正式模块

新增完整ament_python包gca_teleop，提供有类型不可变的InputFrame/InputHealth/Intent，严格配置schema、单一采集所有者、映射、操作状态机、session owner、异步派发、管理代理、节点、CLI、probe及测试。算法/输入端口不依赖rclpy，不复制控制核心。

joydev处理初始化、8字节事件解析、短读、错误、时间回绕、积压、重同步、断连和重开epoch；单次事件数/字节数/时间、等待与关闭均有上限。JS_EVENT_INIT只建立缓存基线，不能当作LT新按下；运行后重新INIT必须先撤销运动。不能把旧队列事件到达时刻当作其发生时刻。

只读probe/标定向导不启动GCA、不领取控制权：逐项采集左右杆、LT、B和预留输入，记录原始编号、内核码、符号、中心/释放值、范围、设备身份与版本，输出profile和原始记录。未知字段不得自动填为已验证。配置变更仅在停止并DISARM后应用，刷新epoch/许可并重新门控。

实现两种明确模式：

- event_age默认：GCA相关的真实有效事件才更新左侧运动年龄；右侧变化、死区噪声、重复值、B释放、GUI、reader心跳不能续旧速度。静态长保持会到期停车并要求释放LT/回中/重新按下，必须在界面及文档明确这一限制。
- qualified_state实现但默认锁定：只有已记录并匹配当前设备/模式/驱动/映射的输入资格测试，证明持LT及非零左杆时无线失联会在约定D内中立或断连，才允许保持经确认的驱动状态。原事件时间不改写，观察时间另记，标明依赖资格契约。缺记录、无法核对身份/版本、解析/健康异常或资格失效时拒绝该模式。不得用一个verified标志替代原始证据；资格记录也不等于当前无线在线证明。

实体资格当前UNKNOWN，只允许保守simulation。硬件使能须同时满足输入资格与原GCA硬件验收条件。资格测试提供工程证据，不声明检测任意无线故障或证明机械停止距离。

每次重新打开、连接epoch、身份或profile变化时重新核验资格。两种模式只在停止确认且DISARM后切换，撤销旧许可和待发工作并重新门控；资格失败不允许悄悄切换模式继续运动。qualified_state的新观察只能来自实际reader推进，不能由发布定时器、许可回执或缓存访问刷新。D只是输入端响应界限，总停车预算还须加入后续调度、命令期限和执行器响应。

scripted端口提供固定种子、可注入时钟与带时间的合成帧，走与joydev相同的映射/门控/session/ROS路径，只允许连接simulation。不得由真实输入断连自动切到脚本源；合成样本不能覆盖真实源失效、修改实体资格或证明实物按键已验证。

## 六、ROS质量契约与既有CLI兼容

新增ManualInputQuality并接入JogCommand，至少明确source kind/ID、input epoch、事件序号/时间及来源、reader观察序号/时间、固定valid_until、同机clock_domain_id、link evidence/qualification ID；程序Jog还需固定episode_end。新增TeleopState作只读状态，包含门控原因、质量、速度、停止及参考状态，不发布凭证。

首期仅同Pi同Linux boot的单调时钟可比较，boot ID取系统实际值；拒绝异机/未来/失效时间，不将ROS时间跳变作为续命依据。event_age、qualified_state、SCRIPT_SAMPLE、PROGRAMMED_JOG各自的期限来源明确，不混淆原事件与本地观察。

coordinator、设备、core和I/O都取输入截止、一次性nonce、租约、输出TTL的最小值。继续使用现有双层get_motion_permit，不因为新增质量字段取消权限。deadman=true缺质量必须拒绝；deadman=false维持受限停车路径。

现有CLI jog(duration)保留调用方式，但一次调用建立固定episode_end，后续帧不能逐帧延长。同步迁移所有正式调用者和测试；不能把未经确认来源默认为PROGRAMMED_JOG。

## 七、唯一会话与管理代理

一个gca_teleop owner持有MANUAL源会话和全局CommandContext序号，token只在内存。管理GUI/CLI不得再次claim或共享token文件。新增本机Unix socket代理：0700目录、0600 socket、SO_PEERCRED同UID、有界帧/连接/超时/请求ID去重、严格操作白名单、预期owner实例/input epoch/会话和版本检查。configure/activate/claim/reference/ARM/RESET/STOP/DISARM/release/status都由同owner转入正式系统接口。

普通launch不自动claim、设参考或ARM。未来磁场输入可复用该owner，本轮不实现磁场电流控制，不占用右侧预留输入，不增加无认证的远程ARM服务。

异步派发为一个在途permit加一个容量1最新输入槽。输入事件、reader观察和命令序号分离；普通新样本不无限取消仍有效工作，续租序号变化不导致许可永久饥饿。过期/B/LT释放/epoch变化的回执必须丢弃，不重新给旧样本TTL。不用阻塞GcaClient.jog循环逐次处理实体输入。

## 八、STOP屏障必须端到端实现

以已认证STOP的context.sequence=S和(session_id,authorization_generation)形成单调有界屏障，覆盖所有停车入口：

- 协调器与设备立即清latest、撤销seq≤S未用nonce，签发/慢ACK/DDS接收均检查屏障。
- core.stop携带context并在普通队列之外撤销输出；core出队再次拒绝旧Jog；I/O需求带停止serial，使已生成旧需求失效。
- owner只有完成新的LT门控周期后才发seq>S。旧续租、重发、迟到消息或STOP回READY都不能恢复运动。
- 客户端、协调器、设备各有独立stop通道/回调组，不能等慢permit/ARM/renew。ARM事务在每次await后和最终enable前核对stop_epoch，B后迟到ARM不得使能。
- 先校验会话/身份再更新屏障，过期旧owner不能影响新owner。停止受理、执行器停止确认、角参考有效性分别报告。

特别修复ARM重新授权的代次交接：当前协调器先rotate_generation再等待租约/ARM响应，owner仍持旧generation时STOP可能被拒绝。仅在同一当前session/source capability内保留有界的前代次STOP别名，作用于交接涉及的执行代次并取消ARM；不能用于renew、ARM或reference，新owner接管立即失效。别名覆盖ARM响应在途，保留至owner明确确认新代次；交接超时撤销ARM并停车，不得只删除别名留下已使能设备。设备端新旧执行许可交接也须有受认证的独立停车能力，不因executor_token暂空或租约回执未到而等待ARM完成。确定性覆盖代次提升后、租约回执前、enable入队、ARM响应在途及交接超时。

## 九、开展可控性模拟和完整回归

完整覆盖方案JT01–JT20，并建立JF→修复源码→测试→实际报告追踪。至少三层验证：纯输入状态机/算法的可控时钟测试；真实ROS进程/DDS的全路径测试；用户可配合时的只读实体手柄→模拟执行器试验。既有独立plant只能消费实际接受的输出与已模拟发出的边沿，不能读取目标/摇杆后直接赋值为位置。

必须测试单轴/斜向/反向/细动/死区/右侧隔离，LT滞回/B优先/重连门控，速度滑移/噪声/延迟/饱和，首次和长空闲后启动，连续相位/有限边沿时序/预约限位，输入/nonce/租约/输出期限，程序episode不可延长，reader与控制线程分别冻结，全部阶段STOP竞争、慢ARM不复活、未知中止和显式恢复，慢盘/慢订阅/有界队列、退出和资源清理。

使用版本化合成profile预先固定指标，至少包含：

- 目标±2 mm/s、滑移0/10%/25%、无噪声且未饱和时，最迟2s后在0.5s稳态窗口测独立plant平均速度绝对误差≤max(5%×abs(v_set), 2个编码器计数/窗口时间)。噪声余量另先声明，不能事后放宽。
- 回中并确认稳定后1s内额外位移≤两个编码器计数，无位置追赶/隐式HOLD。
- 旋转0.1deg/s持续10秒的累计边沿与受限速度独立积分相差≤1脉冲（完整结算、无丢弃预算），不能永远不动；常速跨段平均率亦按5%或量化下限评价。
- 用独立相位计算验证有限边沿时刻、速度/离散加速计划、DIR和高低电平。报告段间空档，不把段内平均率当全程速度。
- STOP闩锁及时撤销，seq≤屏障不再输出；实际ROS记录各段延迟、P50/P95/最大值及事先规定的测试超时，不声明实机硬实时上界。
- 固定种子运行至少30分钟模型时间，检查重复性、漂移、相位余量、状态和日志容量。

竞态用屏障/事件/受控调度安排，不靠偶然sleep；数字预期来自契约、独立数学和独立plant，不能复制实现充当oracle。所有必须软件用例实际运行且无跳过，失败先定位修复，不能删断言或改为仅测mock。原有200项基线只作参考数量，当前完整软件集须重新构建测试，不能直接引用历史PASS。

## 十、工具、文档与完成定义

必须实际交付并验证：

- 安装后的gca_teleop_node、gca_teleop_probe、gca_teleop_cli与teleop_simulation.launch.py，普通launch保持无运动；脚本套件通过明确管理意图操作模拟设备。
- `bash tools/bootstrap_dev.sh`、`bash tools/verify_software.sh`、新增 `bash tools/run_teleop_suite.sh --output artifacts/teleop/latest`。verify纳入新包、全部新测试、旧回归、普通安装及手柄模拟套件；从源码树外加载本次install验证，不靠手工PYTHONPATH掩盖安装问题。
- 模型时间与真实ROS墙钟分开的JSON/JUnit、原始CSV/JSONL和PNG/SVG，包含源文件/配置指纹、设备/profile/资格、输入与质量、速度/编码器/plant真值、脉冲各计数和机械真值、完整停止时间线、命令与退出码。输出有容量管理，失败也清理本次资源并返回非零。
- 中文文档解释设备识别、只读映射向导、两种新鲜度模式及静态保持限制、同owner管理操作、手柄→simulation、停止/参考恢复、报告读取和常见故障。更新ARCHITECTURE、ROS_INTERFACES、USER_GUIDE、配置、HIL与IMPLEMENTATION_STATUS，区分已实现、模拟已测、实体已测及NOT_RUN。

只有模块通过安装入口实际接通、JF修复落地、JT及旧必测软件回归通过、全路径仿真与普通安装通过、文档可照做且报告完整，才报告本轮无执行器硬件实现完成。用户尚未操作的手柄映射/无线资格可保留NOT_RUN并提供完整向导；不能宣称实际手柄长保持或无线掉线行为已验收。真实GCA电气、脉冲与停止距离仍按H01–H03单独验证。

授权的软件工作持续推进，无需逐步确认；需要真实用户按键/拔插/关机的环节明确询问并继续独立自动验证。不得把超时当成用户操作或批准。最终答复给出实现入口、当前手柄可用模式、修复清单、实际测试/报告，以及尚待用户实物确认的项目。
