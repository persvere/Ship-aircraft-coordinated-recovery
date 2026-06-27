**该项目为我的无人机在受风浪干扰下的自主回收方法的仿真验证平台**  
**该项目包含以下几个主要部分**  
*/海浪模拟/*  
//1  
wave_t/Assets/Shaders/FFTOcean_compute.compute ：生成波浪的能量谱，并通过FFT（快速傅里叶变换）计算出顶点偏移数据。  
//2  
wave_t/Assets/Shaders/FFTOcean_Scrips.cs ：这段代码通过调用 FFTOcean_compute.compute执行快速傅里叶变换（FFT），将基于 JONSWAP 物理模型生成的频域波浪谱数据转换为空域的位移与法线纹理，以供水面网格进行实时渲染。  
//3  
wave_t/Assets/Shaders/FTTOcean_shaders.shader ：这段代码是一个基于物理渲染（PBR）的顶点着色器，它通过采样预计算的位移和法线纹理，并集成曲面细分、阴影、泡沫和雾效等技术，来渲染一个具有动态波浪、光照和散射效果的逼真海洋表面。  
//4  
wave_t/Assets/Shaders/FFTOcean_Buoyancy.cs ：这段代码是基于模拟海浪的浮力生成模块  
**这段代码参考了https://github.com/ChenHanMK1/FFT-Ocean-Code，的代码，并根据我的需求做了一定的修改**  
*/无人机飞行物理引擎/*（完成）  
//1  
wave_t/Assets/Models/物理逻辑和算法/DronePhysic.cs ：这段代码是一个基于真实物理开发的无人机物理引擎，将角动量效应，地面效应，下洗流，电机控制延迟，以及空气阻力，反扭矩纳入考虑。他会读取电机控制指令，根据实际情况计算真实推力，再根据这些推力和其余干扰以及力矩更新无人机实时状态  
//2  
wave_t/Assets/Models/物理逻辑和算法/智能体控制相关/DroneController.cs ：这段代码是物理系统更新的总调度器，他使用FIXupdate去固定时长调用物理更新引擎，并能按实际情况去调用搭载推力指令分配算法的无人机飞行控制器  
*/PID控制器/*（完成）  
//1  
wave_t/Assets/Models/物理逻辑和算法/PID_control/PID_control.cs ：这段代码实现了一个无人机飞行的串级PID控制器，它通过位置环和速度环的协同工作，计算出目标加速度和偏航指令，以驱动无人机精确地飞向指定目标点。  
//2  
wave_t/Assets/Models/物理逻辑和算法/flly_control/Droneflightcontrol.cs ：这段代码是一个无人机飞行控制器，它采用串级PID控制算法，根据//1中得到的目标加速度和偏航指令计算出所需的姿态和力矩，并最终通过扭矩分配器生成各电机的推力指令。  
//3  
wave_t/Assets/Models/物理逻辑和算法/flly_control/TorqueAllocator.cs ：这段代码实现了无人机扭矩分配器，它通过//2中预计算矩阵和迭代求解带约束的线性方程组，将期望的三轴力矩和总推力精确地分配给四个电机。  
*/玩家遥控控制/*（完成）  
//1  
wave_t/Assets/Models/物理逻辑和算法/flly_control/InputAccelerationDebugger.cs ：这段代码是遥控器控制的实现，玩家通过wsad控制无人机局部坐标的加速度，j和k控制无人机的升降。设计这个函数初衷是验证TorqueAllocator.cs和Droneflightcontrol.cs是否能基于DronePhysic.cs正常运行  
**实时画面预览如下**  
<img width="1465" height="847" alt="image" src="https://github.com/user-attachments/assets/49e9ada7-4a4b-4163-85b0-2e3db064bb61" />
*/基于ml-agent的强化学习的控制方案/*(废弃)  
wave_t/Assets/Models/物理逻辑和算法/智能体控制相关  
**这部分内容由于当时控制效果表现不佳目前已经被废弃，由于他依赖于ml-agent的包，一起上传会导致项目文件过大，我将其移除了，为了防止编译错误我将相关文件改成了.txt的文本形式，如今回头看我认为该方案失败的主要原因可能在于我的训练方法存在问题，当时我的训练方法是通过设计一个精妙的奖励函数完成一次训练达到所需要求。**  
**尽管引入了cnn记忆单元仍然表现不佳。我认为处理这样的长序列的复杂任务阶段性训练非常重要。我应该先训练他的存活能力，在保证足够的存活时间不会炸机后，在此基础上训练其避障能力。避障能力训练完成后再在此基础上训练其向目标点移动的能力，逐次修改奖励函数，但中间会存在一个问题就是奖励函数在这些任务间的过度**  
**我认为可以设计一个存储单元，他存储一个任务完成率的数据，根据这个数据去修改上一层任务的奖励函数值，仿真层级切换奖励函数的突变**  
*/基于世界模型的MPC控制范式/*（开发中）  
wave_t/Assets/Models/物理逻辑和算法/MPC/StepPredict.cs ：当前的世界模型，他会根据当前的电机控制指令预测未来电机状态  
**这是一个世界模型的槽位，可以在这个槽位插入任何世界模型，如果时间充裕，我希望尝试用基于transformer的神经网络去从数据中拟合一个世界模型**  
wave_t/Assets/Models/物理逻辑和算法/MPC/FiniteDifferenceSolver.cs :他会根据世界模型的结果进行有限差分法，生成离散状态转移的雅各比矩阵  
**该套算法目前处在开发中，整体的算法流程图如下**  
<img width="1953" height="994" alt="image" src="https://github.com/user-attachments/assets/ec40e5ab-7339-46f1-9d5e-e85cf7f1ba56" />  
其中红色框代表每一轮运算需要读取的输入，虚线框代表循环结构，黑色框代表需要开发的算法模块，蓝色框代表运算过程会产生的中间临时存储数据，绿色框代表最终输出  
**无人机旋翼不会在画面转动是因为模型是扫描模型，难以将旋翼分割为子物体，且该项目主要是为学术目的架设，故而并未对这个画面做优化**
