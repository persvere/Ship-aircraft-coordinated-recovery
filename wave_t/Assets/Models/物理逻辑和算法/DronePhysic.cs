using UnityEngine;




// ========== 1. 独立的物理引擎 ==========
public class DronePhysics : MonoBehaviour
{
    [Header("Physical Parameters")]
    public float droneMass = 1.0f;
    public Vector3 inertia = new Vector3(0.01f, 0.01f, 0.02f);////////////惯性矩问题
    public float gravity = -9.81f;

    public Vector3 totalTorque;

    [Header("旋翼结构参数")]
    public float rotorRadius = 0.1f;
    public float Anti_torquecoefficient = 0.1f;
    public Vector3[] rotorLocalPositions = new Vector3[4]
    {
        new Vector3(0.2f, 0.05f, 0.2f),   // 前右
        new Vector3(-0.2f, 0.05f, 0.2f),  // 前左
        new Vector3(-0.2f, 0.05f, -0.2f), // 后左
        new Vector3(0.2f, 0.05f, -0.2f)   // 后右
    };

    [Header("环境")]
    public float airDensity = 1.225f;
    public float groundLevel = 0f;
    
    [Header("Current State")]
    public Vector3 position;
    public Vector3 velocity=Vector3.zero;
    public Vector3 acceleration;
    public Vector3 angularVelocity=Vector3.zero;
    public Quaternion attitude;
    public Vector4 motorThrusts=Vector4.zero;  // 当前电机推力 [0, 1]
    public Transform platformtransform;
    public Vector3 relativeposition;
    public Vector3 lastrelativeposition;
    public Quaternion relativeattitude;
    public Vector3 relativeVelocity;

    //public Vector3 testal;



    public float[] actualThrusts = new float[4];//为什么不用vector
    
    [Header("Motor Properties")]
    public float maxThrust = 10f;
    public float motorTimeConstant = 0.05f;
    
    public Vector4 targetThrusts;  // 目标推力
    
    [Header("空气阻力相关")]
    public float forwardDragCoeff=0.8f;
    public float sideDragCoeff=1.0f;
    public float upDragCoeff=1.2f;        // 垂向阻力系数
    public float referenceArea=0.01f;
    public Vector3 AnisotropicDrag;
    [Header("旋翼角动量系数（包含推力系数）")]
    public float rotorInertia;

    //初始化
     void Awake()
    {
         attitude = transform.rotation;
         position = transform.position;
    }

    
    // 被Agent调用来设置控制信号
    public void SetMotorCommands(Vector4 commands)
    {
        targetThrusts = commands;
    }
    
    // 物理更新（在FixedUpdate中调用）
    public void PhysicsUpdate(float deltaTime)
    {
          if (relativeposition != Vector3.zero)
        {
            lastrelativeposition=relativeposition;
        }
        // 0.计算相对位姿
        //Debug.Log($"当前位置{platformtransform.position}");
        (relativeposition,relativeattitude)=CalculateRelativePose(transform,platformtransform);
        //0.1补充计算相对速度
        if (relativeposition != Vector3.zero)
        {
            relativeVelocity=(relativeposition-lastrelativeposition)/deltaTime;
        }
        // 1. 电机响应延迟
        UpdateMotorThrusts(deltaTime);
        
        // 2. 计算总力和力矩
        //2.1计算单个电机的受力
         Vector3 _velocity = velocity;
        Quaternion _rotation = attitude;
        float height = relativeposition.y;
        
        for (int i = 0; i < 4; i++)
        {
            // 考虑每个旋翼的离地高度
            Vector3 rotorWorldPos = _rotation * rotorLocalPositions[i];
            float rotorHeight = height + rotorWorldPos.y;  // 旋翼离地高度
            
            actualThrusts[i] = CompleteThrustCalculator.CalculateCompleteThrust(
               motorThrusts[i],
                _velocity,
                _rotation,
                rotorRadius,
                rotorHeight,  // 传入旋翼高度
                maxThrust,
                airDensity
            );
        }
        //2.2计算空气阻力
        AnisotropicDrag=CalculateAnisotropicDrag(velocity,transform.rotation,forwardDragCoeff,sideDragCoeff,upDragCoeff,referenceArea,airDensity = 1.225f);
       



        Vector3 totalForce = CalculateTotalForce();
        totalTorque = CalculateTotalTorque();
        
        // 3. 动力学方程
        UpdateDynamics(totalForce, totalTorque, deltaTime);
        
        // 4. 更新Transform
        UpdateTransform();
    }
    
    private void UpdateMotorThrusts(float deltaTime)
    {
        
        for (int i = 0; i < 4; i++)
        {
            float target = targetThrusts[i];
            float current = motorThrusts[i];
            float alpha = Mathf.Exp(-deltaTime / motorTimeConstant);
            motorThrusts[i] = target - (target - current) * alpha;
            motorThrusts[i] = Mathf.Clamp01(motorThrusts[i]);
        }
    }
    
    private Vector3 CalculateTotalForce()
    {
        Vector3 totalForce = Vector3.zero;
        
        // 1. 推力
        for (int i = 0; i < 4; i++)
        {
        totalForce += transform.up * actualThrusts[i];
        }
        
        // 2. 重力
        totalForce += new Vector3(0,-9.81f,0) * droneMass;
        
        // 3. 阻力
        totalForce += AnisotropicDrag;
        
        return totalForce;
    }
    
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
    
    private void UpdateDynamics(Vector3 totalForce, Vector3 totalTorque, float deltaTime)
    {
        // 平移动力学
        acceleration = totalForce / droneMass;
        velocity += acceleration * deltaTime;
        position += velocity * deltaTime;
        
        // 旋转动力学
        Vector3 angularAcceleration = new Vector3(
            totalTorque.x / inertia.x,
            totalTorque.y / inertia.y,
            totalTorque.z / inertia.z
        );
        
        angularVelocity += angularAcceleration * deltaTime;
        
        // 更新姿态
         UpdateAttitude(deltaTime);

    }

    //更新姿态
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
    attitude = new Quaternion(
        attitude.x + dq.x * dt,
        attitude.y + dq.y * dt,
        attitude.z + dq.z * dt,
        attitude.w + dq.w * dt
    );
  
    // 归一化（必须！）
    attitude.Normalize();
}
    
    private void UpdateTransform()
    {
        transform.position = position;
        transform.rotation = attitude;
        //testal=attitude.eulerAngles;
    }
    
    public void Reset(Vector3 startPosition, Quaternion startRotation)
    {
        position = startPosition;
        velocity = Vector3.zero;
        acceleration = Vector3.zero;
        attitude = startRotation;
        angularVelocity = Vector3.zero;
        motorThrusts = new Vector4(0.125f, 0.125f, 0.125f, 0.125f);
        targetThrusts = motorThrusts;
    }

     /// 计算B相对于A的位姿
    /// 位姿 = 位置 + 姿态
    /// 返回：B在A坐标系中的位置和姿态
    /// </summary>
    public static (Vector3 position, Quaternion rotation) CalculateRelativePose(
        Transform objectA,  // 参考物体
        Transform objectB)  // 目标物体
    {
        if (objectA == null || objectB == null)
        {
            Debug.LogError("对象不能为空");
            return (Vector3.zero, Quaternion.identity);
        }
        
        // 1. 计算相对位置
        Vector3 relativeposition = objectA.position-objectB.position;
        
        // 2. 计算相对姿态
        Quaternion relativeRotation = Quaternion.Inverse(objectA.rotation) * objectB.rotation;
        
        return (relativeposition, relativeRotation);
    }
    public static class CompleteThrustCalculator
{
    /// <summary>
    /// 完整推力计算：指令推力 + 空气动力学 + 地面效应
    /// 假设电机动态已由控制器处理
    /// </summary>
    public static float CalculateCompleteThrust(
        float commandedThrust,        // 指令推力
        Vector3 droneVelocity,        // 无人机速度
        Quaternion droneRotation,     // 无人机姿态
        float rotorRadius,           // 旋翼半径
        float heightAboveGround,     // 离地高度
        float maxThrust,
        float airDensity = 1.225f
        )
    {
        // 1. 指令推力作为基准
        float baseThrust = commandedThrust*maxThrust;
        
        // 2. 空气动力学效率修正
        float aeroEfficiency = CalculateAerodynamicEfficiency(
            baseThrust, droneVelocity, droneRotation, rotorRadius, airDensity);
        
        // 3. 地面效应因子
        float groundEffect = CalculateGroundEffectFactor(
            baseThrust, rotorRadius, heightAboveGround, airDensity);
        
        // 4. 综合推力
        float realThrust = baseThrust * aeroEfficiency * groundEffect;
        
        // 5. 物理限制
        realThrust = Mathf.Max(0f, realThrust);
        
        return realThrust;
    }
    
    private static float CalculateAerodynamicEfficiency(
        float thrust, Vector3 velocity, Quaternion rotation, 
        float rotorRadius, float airDensity)
    {
        Vector3 rotorNormal = rotation * Vector3.up;
        float verticalSpeed = Vector3.Dot(velocity, rotorNormal);
        Vector3 inPlaneVelocity = velocity - verticalSpeed * rotorNormal;
        float inPlaneSpeed = inPlaneVelocity.magnitude;
        
        float rotorArea = Mathf.PI * rotorRadius * rotorRadius;
        float hoverInducedVel = Mathf.Sqrt(thrust / (2f * airDensity * rotorArea));
        
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
    
    private static float CalculateGroundEffectFactor(
        float thrust, float rotorRadius, float height, float airDensity)
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
}
 public static Vector3 CalculateAnisotropicDrag(
    Vector3 velocity,           // 世界坐标系速度
    Quaternion rotation,        // 无人机姿态
    float forwardDragCoeff,     // 前向 (Z 轴) 阻力系数
    float sideDragCoeff,        // 侧向 (X 轴) 阻力系数
    float upDragCoeff,          // 垂向 (Y 轴) 阻力系数
    float referenceArea,
    float airDensity = 1.225f)
{
    if (velocity.magnitude < 0.001f)
        return Vector3.zero;
    
    // 1. 将速度转换到物体局部坐标系
    Vector3 localVelocity = Quaternion.Inverse(rotation) * velocity;
    
    Vector3 localDrag = Vector3.zero;
    
    // 2. 分别在局部坐标系下计算各轴的平方阻力
    // 公式：F = -0.5 * rho * |v| * v * Cd * A
    // 使用 Mathf.Sign(v) * v * v 来实现带方向的平方
    
    // X 轴 (侧向阻力)
    localDrag.x = -0.5f * airDensity * referenceArea * sideDragCoeff * 
                  Mathf.Sign(localVelocity.x) * localVelocity.x * localVelocity.x;
    
    // Y 轴 (垂向阻力)
    localDrag.y = -0.5f * airDensity * referenceArea * upDragCoeff * 
                  Mathf.Sign(localVelocity.y) * localVelocity.y * localVelocity.y;
    
    // Z 轴 (前向阻力)
    localDrag.z = -0.5f * airDensity * referenceArea * forwardDragCoeff * 
                  Mathf.Sign(localVelocity.z) * localVelocity.z * localVelocity.z;
    
    // 3. 将局部阻力转回世界坐标系
    return rotation * localDrag;
}
    
    /// <summary>
    /// 计算陀螺力矩（角动量效应）
    /// τ_gyro = ω × Jω_rotor
    /// 其中ω是无人机角速度，ω_rotor是旋翼角速度
    /// </summary>
    private Vector3 CalculateGyroscopicTorque()
    {
        Vector3 totalGyroTorque = Vector3.zero;
        
        for (int i = 0; i < 4; i++)
        {
            
            // 旋翼角速度矢量（沿旋翼轴）,我使用推力来平替
            float rotorOmega = Mathf.Sqrt(motorThrusts[i]);
            
            // 旋翼转动方向：对角相反
            int direction = (i % 2 == 0) ? 1 : -1;  // 0,2正转；1,3反转
            
            Vector3 rotorAngularVelocity = transform.up * rotorOmega * direction;
            
            // 旋翼角动量
            Vector3 rotorAngularMomentum = rotorInertia * rotorAngularVelocity;
            
            // 陀螺力矩：τ = ω_drone × H_rotor
            Vector3 gyroTorque = Vector3.Cross(angularVelocity, rotorAngularMomentum);
            
            totalGyroTorque += gyroTorque;
        }
        
        return totalGyroTorque;
    }
    
    /// <summary>
    /// 计算旋翼反扭矩
    /// 旋翼旋转时，电机机体会受到反作用力矩
    /// τ_reaction = -I * α_rotor
    /// 简化：τ_reaction = -k * thrust
    /// </summary>
    private Vector3 CalculateRotorReactionTorque()
    {
        Vector3 totalReactionTorque = Vector3.zero;

        int[] rotordir = new int[4] {1,-1,1,-1};
        
        for (int i = 0; i < 4; i++)
        {
            
            // 旋翼转动方向
            int direction = rotordir[i];
            
            // 反扭矩大小与推力成正比
            float torqueMagnitude = motorThrusts[i] * rotorRadius * Anti_torquecoefficient * maxThrust;
            
            // 方向：沿旋翼轴，与旋转方向相反
            Vector3 reactionTorque = -transform.up * torqueMagnitude * direction;
            
            totalReactionTorque += reactionTorque;
        }
        
        return totalReactionTorque;
    }
    
    /// <summary>
    /// 计算空气阻力矩
    /// 阻力作用点可能不在重心，产生力矩
    /// </summary>
    private Vector3 CalculateDragTorque()
    {
        Vector3 dragTorque = Vector3.zero;
        
        // 假设阻力作用在重心，但可以扩展
        // 计算阻力中心偏移产生的力矩
        
        // 简化：阻力作用在中心，但姿态变化时会产生阻尼力矩
        dragTorque = -angularVelocity * 0.05f;  // 旋转阻尼
        
        return dragTorque;
    }
    private Vector3 CalculateThrustTorque()
{
    Vector3 totalTorque = Vector3.zero;
    
    for (int i = 0; i < 4; i++)
    {
        if (actualThrusts[i] <= 0f) continue;
        
        // 1. 获取电机在世界坐标系中的位置
        Vector3 rotorWorldPos = transform.TransformPoint(rotorLocalPositions[i]);
        
        // 2. 推力方向（沿无人机上方向）
        Vector3 thrustDirection = transform.up;
        Vector3 thrustForce = thrustDirection * actualThrusts[i];
        
        // 3. 计算力臂（从无人机重心到电机位置）
        Vector3 torqueArm = rotorWorldPos - transform.position;
        
        // 4. 力矩 = 力臂 × 推力
        Vector3 motorTorque = Vector3.Cross(torqueArm, thrustForce);
        
        // 5. 累加到总力矩
        totalTorque += motorTorque;
    }
    
    return totalTorque;
}

}