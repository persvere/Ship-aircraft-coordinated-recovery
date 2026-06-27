using UnityEngine;

//**这个类是单步预测类，他依赖于真实物理类（DronePhysic)中的方法。。
//**他的目的是做单步预测
namespace MPC
{
public class StepPredict
{
//每次调用需要的变量
    //**1状态量
        //位置
        public Vector3 position;
        //速度
        public Vector3 velocity;
        //四元数
        public Quaternion attitude;
        //角速度
        public Vector3 angularVelocity;
    //**2控制量

        //电机推力控制量
        public Vector4 rotorThrust;
//物理模型需要的参数
    //**1环境参数
        //空气密度
        public float airDensity = 1.225f;
        //离散时间间隔
        public float deltaTime;
    //**2无人机参数
        //*1旋翼相关参数
            //无人机旋翼旋转方向
            public int[] rotordir = new int[4] {1,-1,1,-1};
            //四个旋翼在机体坐标系下的位置
            public Vector3[] rotorLocalPositions = new Vector3[4]
            {
            new Vector3(0.2f, 0.05f, 0.2f),   // 前右
            new Vector3(-0.2f, 0.05f, 0.2f),  // 前左
            new Vector3(-0.2f, 0.05f, -0.2f), // 后左
            new Vector3(0.2f, 0.05f, -0.2f)   // 后右
            };
            //单旋翼电机最大推力
            public float maxThrust = 10f;
            //旋翼角动量系数
            public float rotorInertia=0.01f;
            //反扭距系数（描述旋翼产生的推力和扭矩之间的关系）
            public float Anti_torquecoefficient = 0.1f;
            //旋翼半径
            public float rotorRadius = 0.1f;
        //*2空气阻力相关
            //前向空气阻力系数
            public float forwardDragCoeff=0.8f;
            //侧向空气阻力系数
            public float sideDragCoeff=1.0f;
            //垂向空气阻力系数
            public float upDragCoeff=1.2f;
            //机体前向面积
            public float forwardreferenceArea=0.01f;
            //机体侧向面积
            public float sidereferenceArea=0.01f;
            //机体纵向面积
            public float upreferenceArea=0.01f;
        //*3机体相关系数
            //无人机质量
            public float droneMass = 1.0f;
            //无人机惯性矩
            public Vector3 inertia = new Vector3(0.01f,0.01f,0.02f);
        //*4电机有关系数
            //电机时间常数
            public float motorTimeConstant = 0.05f;
        //*5环境相关系数
            //重力加速度
            Vector3 g = new Vector3(0,-9.81f,0);
//计算过程会使用的中间变量
    //电机实际产生的推力
    public float[] actualThrust = new float[4];
    //空气阻力 - 
    public Vector3 AnisotropicDrag;
    //重力
    public Vector3 gravity;
    //电机推力矩 -
    Vector3 thrustTorque;
    //空气阻力矩 -
    Vector3 airdrugTorque;
    //陀螺力矩 -
    Vector3 gyroscopicTorque;
    //旋翼反扭矩 -
    Vector3 rotorReactionTorque;
    //单个电机的相对于机体原点的世界坐标偏离
    Vector3 rotor_relativePos;
    //单个电机的离地高度
    float rotorHeight;
    
//计算结果存储变量
    //下一时刻状态量
        //位置
        public Vector3 new_Position;
        //速度
        public Vector3 new_Velocity;
        //欧拉角
        public Quaternion new_attitude;
        //角速度
        public Vector3 new_AngularVelocity;
//计算过程中的辅助函数
    /**************************************************************************************************/
    //**1单个电机推力计算（根据当前输入状态，和之前设置好的配置项得出单个电机推力的模块）
            private float[] CalculateCompleteThrust()
        {   //临时变量
                float realThrust;
                float[] RealThrust = new float[4];

            for(int i = 0; i<4 ; i++)
            {   
                // 1. 指令推力作为基准
                float baseThrust = rotorThrust[i]*maxThrust;
                // 2. 空气动力学效率修正
                float aeroEfficiency = CalculateAerodynamicEfficiency(baseThrust);
        
                // 3. 地面效应因子
                    //计算每个电机的离地高度
                    rotor_relativePos = attitude * rotorLocalPositions[i];
                    float rotorHeight = position.y + rotor_relativePos.y;  // 旋翼离地高度
                float groundEffect = CalculateGroundEffectFactor(baseThrust,rotorHeight);
        
                // 4. 综合推力
                realThrust = baseThrust * aeroEfficiency * groundEffect;
        
                // 5. 物理限制
                realThrust = Mathf.Max(0f, realThrust);

                // 6.汇总
                RealThrust[i]=realThrust;
            }
            return RealThrust;
        }
    /**************************************************************************************************/
        //局部辅助函数
        /**************************************************************************************************/
        //延迟电机推力指令
        /*    private void UpdateMotorThrusts()
        {
        
                for (int i = 0; i < 4; i++)
            {
                float target = targetThrust[i];
                float current = rotorThrust[i];
                float alpha = Mathf.Exp(-deltaTime / motorTimeConstant);
                rotorThrust[i] = target - (target - current) * alpha;
                rotorThrust[i] = Mathf.Clamp01(rotorThrust[i]);
            }
        }
        /**************************************************************************************************/
        //空气动力学修正
            private float CalculateAerodynamicEfficiency(float baseThrust)
        {
            Vector3 rotorNormal = attitude * Vector3.up;
            float verticalSpeed = Vector3.Dot(velocity, rotorNormal);
            Vector3 inPlaneVelocity = velocity - verticalSpeed * rotorNormal;
            float inPlaneSpeed = inPlaneVelocity.magnitude;
    
            float rotorArea = Mathf.PI * rotorRadius * rotorRadius;
            float hoverInducedVel = Mathf.Sqrt(baseThrust / (2f * airDensity * rotorArea));
        
            float efficiency = 1.0f;
        
            // 前飞增强
                if (inPlaneSpeed > 0.1f)
            {
                float speedRatio = inPlaneSpeed / hoverInducedVel;
                // 原公式 (错误，无界)
                // efficiency *= Mathf.Sqrt(1f + speedRatio * speedRatio);

                // 新公式 (正确，有界，最大增益 5%)
                float maxGain = 0.05f; // 真实物理中很少超过 15-20%
                float gainFactor = Mathf.Clamp01(speedRatio * 0.01f); 
                efficiency += 0.05f * gainFactor;
            }
        
            /* 爬升惩罚(旧版)
            if (verticalSpeed > 0.3f * hoverInducedVel)
            {
                float climbRatio = verticalSpeed / hoverInducedVel;
                efficiency *= 1f - 0.4f * Mathf.Clamp01((climbRatio - 0.3f) / 0.7f);
            }*/
            // --- 修改后的爬升惩罚逻辑 ---

            // 1. 提高触发阈值：设为 1.0 倍诱导速度 (约3-4 m/s)
            // 之前的 0.3 倍太敏感了，导致正常爬升都被惩罚
            float climbThreshold = 1.0f * hoverInducedVel;

                if (verticalSpeed > climbThreshold)
            {
                // 2. 计算超出部分的速度
                float excessSpeed = verticalSpeed - climbThreshold;
    
                // 3. 计算惩罚系数
                // 逻辑：每超出 1 m/s，效率降低 2%，最大降低不超过 15%
                // 这样即使你以 10 m/s 极速爬升，推力也只会损失 15%，PID 完全可以补偿
                float penaltyFactor = Mathf.Clamp01(excessSpeed / 5.0f) * 0.15f;
    
                // 4. 应用惩罚 (乘法)
                efficiency *= (1.0f - penaltyFactor);
            }

            // 【可选进阶】如果你也想模拟下降时的“涡环状态” (Vortex Ring State)
            // 涡环通常发生在下降速度为 -0.7 ~ -1.5 倍诱导速度时
            /*
            if (verticalSpeed < 0 && verticalSpeed > -2.0f * hoverInducedVel)
            {
                float descentRatio = -verticalSpeed / hoverInducedVel;
                // 在 0.7 ~ 1.5 倍之间效率最低
                if (descentRatio > 0.7f && descentRatio < 1.5f)
                {
                    // 模拟一个凹陷的效率曲线，最大损失 20%
                    float vortexPenalty = 0.2f * Mathf.Sin((descentRatio - 0.7f) * Mathf.PI); 
                    efficiency *= (1.0f - vortexPenalty);
                }
            }
            */
        
            return Mathf.Clamp(efficiency, 0.4f, 1.8f);
        }
        /**************************************************************************************************/
        //地面效应
            private float CalculateGroundEffectFactor(float thrust,float height)
        {
            if (height > rotorRadius * 4f) return 1.0f;
            if (height <= 0) return 1.0f;
        
            float rotorDiameter = 2f * rotorRadius;
            float normalizedHeight = height / rotorDiameter;
        
            float groundEffect = 1.0f;
        
            if (normalizedHeight < 0.5f)
            {
                // 强地面效应
                groundEffect = 1f / (1f - Mathf.Pow(rotorRadius/(4f*height), 2f));
            }
            else if (normalizedHeight < 2f)
            {
                // 弱地面效应
                float decay = Mathf.Exp(-2f * (normalizedHeight - 0.5f));
                groundEffect = 1f + 0.3f * decay;
            }
        
            return Mathf.Clamp(groundEffect, 1.0f, 1.5f);
        }
        /**************************************************************************************************/
    /**************************************************************************************************/
    //*2空气阻力计算函数
        public Vector3 CalculateAnisotropicDrag()
    {
        if (velocity.magnitude < 0.001f)
        return Vector3.zero;
    
        // 1. 将速度转换到物体局部坐标系
        Vector3 localVelocity = Quaternion.Inverse(attitude) * velocity;
        Vector3 localDrag = Vector3.zero;
    
        // 2. 分别在局部坐标系下计算各轴的平方阻力
        // 公式：F = -0.5 * rho * |v| * v * Cd * A
        // 使用 Mathf.Sign(v) * v * v 来实现带方向的平方

        // X 轴 (侧向阻力)
        localDrag.x = -0.5f * airDensity * forwardreferenceArea * sideDragCoeff * 
                  Mathf.Sign(localVelocity.x) * localVelocity.x * localVelocity.x;
    
        // Y 轴 (垂向阻力)
        localDrag.y = -0.5f * airDensity * sidereferenceArea * upDragCoeff * 
                  Mathf.Sign(localVelocity.y) * localVelocity.y * localVelocity.y;
    
        // Z 轴 (前向阻力)
        localDrag.z = -0.5f * airDensity * upreferenceArea * forwardDragCoeff * 
                  Mathf.Sign(localVelocity.z) * localVelocity.z * localVelocity.z;
    
        // 3. 将局部阻力转回世界坐标系
        return attitude * localDrag;
    }
    /**************************************************************************************************/
    //3*电机推力矩计算函数
        private Vector3 CalculateThrustTorque()
    {
        Vector3 totalTorque = Vector3.zero;
    
        for (int i = 0; i < 4; i++)
        {
            if (actualThrust[i] <= 0f) continue;
        
            // 2. 推力方向（沿无人机上方向）
            Vector3 thrustDirection = attitude*Vector3.up;
            Vector3 thrustForce = thrustDirection * actualThrust[i];
        
            // 3. 计算力臂（从无人机重心到电机位置在世界坐标系下表示）
            Vector3 torqueArm = attitude*rotorLocalPositions[i];
        
            // 4. 力矩 = 力臂 × 推力
            Vector3 motorTorque = Vector3.Cross(torqueArm, thrustForce);
        
            // 5. 累加到总力矩
            totalTorque += motorTorque;
        }
    
        return totalTorque;
    }
    /**************************************************************************************************/
    //4*空气阻力矩计算函数
    private Vector3 CalculateDragTorque()
    {
        Vector3 dragTorque = Vector3.zero;
        
        // 假设阻力作用在重心，但可以扩展
        // 计算阻力中心偏移产生的力矩
        
        // 简化：阻力作用在中心，但姿态变化时会产生阻尼力矩
        dragTorque = -angularVelocity * 0.05f;  // 旋转阻尼
        
        return dragTorque;
    }
    /**************************************************************************************************/
    //5*陀螺力矩计算函数
    private Vector3 CalculateGyroscopicTorque()
    {
        Vector3 totalGyroTorque = Vector3.zero;
        
        for (int i = 0; i < 4; i++)
        {
            
            // 旋翼角速度矢量（沿旋翼轴）,我使用推力来平替
            float rotorOmega = Mathf.Sqrt(rotorThrust[i]);
            
            Vector3 rotorAngularVelocity = attitude*Vector3.up * rotorOmega * rotordir[i];
            
            // 旋翼角动量
            Vector3 rotorAngularMomentum = rotorInertia * rotorAngularVelocity;
            
            // 陀螺力矩：τ = ω_drone × H_rotor
            Vector3 gyroTorque = Vector3.Cross(angularVelocity, rotorAngularMomentum);
            
            totalGyroTorque += gyroTorque;
        }
        
        return totalGyroTorque;
    }
    /**************************************************************************************************/
    //6*旋翼反扭距计算函数
        private Vector3 CalculateRotorReactionTorque()
    {
        Vector3 totalReactionTorque = Vector3.zero;
        
        for (int i = 0; i < 4; i++)
        {
            
            // 旋翼转动方向
            int direction = rotordir[i];
            
            // 反扭矩大小与推力成正比
            float torqueMagnitude = rotorThrust[i] * rotorRadius * Anti_torquecoefficient * maxThrust;
            
            // 方向：沿旋翼轴，与旋转方向相反
            Vector3 reactionTorque = attitude*Vector3.up * torqueMagnitude * (-direction);
            
            totalReactionTorque += reactionTorque;
        }
        
        return totalReactionTorque;
    }
    /**************************************************************************************************/
    //7*合力更新函数
        private Vector3 CalculateTotalForce()
    {
        Vector3 totalForce = Vector3.zero;
        
        // 1. 推力
        for (int i = 0; i < 4; i++)
        {
        totalForce += attitude*Vector3.up * actualThrust[i];
        }
        
        // 2. 重力
        totalForce += new Vector3(0,-9.81f,0) * droneMass;
        
        // 3. 阻力
        totalForce += AnisotropicDrag;
        
        return totalForce;
    }
    /**************************************************************************************************/
    //8*总力矩更新函数
        private Vector3 CalculateTotalTorque()
    {
        Vector3 totalTorque = Vector3.zero;
    
    // 1. 电机推力矩（主要控制力矩）
    Vector3 thrustTorque = CalculateThrustTorque();
    totalTorque += thrustTorque;
    
    // 2. 空气阻力矩
    Vector3 dragTorque = CalculateDragTorque();
    totalTorque += dragTorque;
    
    // 3. 陀螺力矩（电机旋转的角动量效应）
    Vector3 gyroscopicTorque = CalculateGyroscopicTorque();
    totalTorque += gyroscopicTorque;
    
    // 4. 旋翼反扭矩
    Vector3 rotorReactionTorque = CalculateRotorReactionTorque();
    totalTorque += rotorReactionTorque;
    
    return totalTorque;
    }
    /**************************************************************************************************/
    //姿态更新函数
        private void UpdateAttitude(float dt)
    {
        // 四元数微分方程：dq/dt = 0.5 * ω * q
        // 其中 ω = [0, ω_x, ω_y, ω_z] 是纯四元数
    
        Vector3 omega = angularVelocity;
    
        // 将角速度转换为纯四元数
        Quaternion omegaQuat = new Quaternion(omega.x, omega.y, omega.z, 0);
    
        // 四元数微分：dq/dt = 0.5 * ω * q
        Quaternion product = omegaQuat * attitude;  // 或 attitude * omega，注意顺序！
        Quaternion dq = new Quaternion(
        product.x * 0.5f,
        product.y * 0.5f,
        product.z * 0.5f,
        product.w * 0.5f
        );
    // 向前欧拉积分
        new_attitude = new Quaternion(
        attitude.x + dq.x * dt,
        attitude.y + dq.y * dt,
        attitude.z + dq.z * dt,
        attitude.w + dq.w * dt
        );
  
        // 归一化（必须！）
        new_attitude.Normalize();
    }
/**************************************************************************************************/  

//位姿更新函数（对外接口） 
    public void UpdateDynamics(Vector3 totalForce, Vector3 totalTorque, float deltaTime)
    {

        //计算当前电机推力
        actualThrust = CalculateCompleteThrust();
        //计算空气阻力
        AnisotropicDrag=CalculateAnisotropicDrag();
        //计算合力
        Vector3 TotalForce = CalculateTotalForce();
        // 平移动力学
        Vector3 acceleration = TotalForce / droneMass;
        new_Velocity += acceleration * deltaTime;
        new_Position += velocity * deltaTime;
        
        //计算总力矩
        Vector3 TotalTorque = CalculateTotalTorque();
        // 旋转动力学
        Vector3 angularAcceleration = new Vector3(
            TotalTorque.x / inertia.x,
            TotalTorque.y / inertia.y,
            TotalTorque.z / inertia.z
        );
        
        new_AngularVelocity += angularAcceleration * deltaTime;
        
        // 更新姿态
         UpdateAttitude(deltaTime);

    }


}
}