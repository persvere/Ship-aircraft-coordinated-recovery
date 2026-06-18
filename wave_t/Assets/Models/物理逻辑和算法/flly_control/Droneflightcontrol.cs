using UnityEngine;
public class DroneFlightController : MonoBehaviour
{
    [Header("组件引用")]
    public DronePhysics dronePhysics;
    public TorqueAllocator torqueAllocator;
    [Header("目标姿态")]
    public Quaternion targetattitude;
    [Header("控制变量")]
    public Vector3 acceleration;       //预定加速度
    public float biasyaw=3f;          //偏航角度,向右为正
    public float thrustlongth;        //推力模长
    

    [Header("PID控制容器")]
    public PIDConfig attitudepid = new PIDConfig(0.05f,0f,0.005f); //姿态环
    public PIDConfig ratepid = new PIDConfig(0.002f,0f,0.0002f);     //角速度环
    private PIDController attitudecontrol;
    private PIDController ratecontrol;

    [Header("当前状态")]
    [SerializeField] private Vector3 attitudeError;     // 姿态误差（度）
    [SerializeField] private Vector3 targetAngularRate; // 目标角速度（度/秒）
    [SerializeField] private Vector3 rateError;         // 角速度误差
    [SerializeField] private Vector3 torqueOutput;      // 输出力矩（N·m）
    [SerializeField] private Vector4 Thrust;            //输出推力

    void Awake()
    {
        Initialize();
    }
    
    // 数据初始化
    private void Initialize()
    {
        // 获取组件引用
        if (dronePhysics == null)
            dronePhysics = GetComponent<DronePhysics>();
        //PID控制器初始化
        if (torqueAllocator == null)
            torqueAllocator = GetComponent<TorqueAllocator>();
        attitudecontrol = new PIDController(attitudepid);
        ratecontrol = new PIDController(ratepid);
        
    }

    public void pidflyconrol()
    {
        Vector3 worldacceleration = dronePhysics.attitude * acceleration;
        torqueOutput = torqueconvert(worldacceleration);
        thrustlongth =  dronePhysics.droneMass * (acceleration - new Vector3(0,-9.81f,0)).magnitude;
        Thrust = torqueAllocator.GetAllocatedThrust(torqueOutput,thrustlongth);
        Thrust = Thrust/dronePhysics.maxThrust;
        dronePhysics.SetMotorCommands(Thrust);
    }

    public Vector3 torqueconvert(Vector3 accelerationCmd)
    {
        //
        
        Vector3 currentvelocity = dronePhysics.angularVelocity;
        float mass = dronePhysics.droneMass;
        Quaternion currentattitude = dronePhysics.attitude;
        Vector3 thrustVector = mass * (accelerationCmd - Physics.gravity);
         if (thrustVector.magnitude < 0.001f)
         return Vector3.zero;
            
        Vector3 thrustDirection = thrustVector.normalized;
        targetattitude = CalculateAttitudeFromThrust(thrustDirection,currentattitude);//根据加速度向量获取目标姿态
        
        Vector3 Torque = CalculateTorque(targetattitude,currentattitude,currentvelocity,Time.fixedDeltaTime,dronePhysics.inertia[1],biasyaw);
        return Torque;
    }

    public static Quaternion CalculateAttitudeFromThrust( Vector3 thrustDirection,  Quaternion currentAttitude)
    {
        // 1. 确保推力方向是单位向量
        Vector3 up = thrustDirection.normalized;
        
        // 2. 从当前姿态提取偏航（前向方向）
        Vector3 currentForward = currentAttitude * Vector3.forward;


        // 3. 将当前前向投影到垂直于推力的平面
        Vector3 forward = Vector3.ProjectOnPlane(currentForward, up).normalized;
        if (forward.sqrMagnitude < 0.001f)
    {
        // 如果机头方向和推力方向几乎平行（例如垂直爬升或倒飞），
        // 我们不能用机头方向作为参考，必须换一个参考系！
        
        // 尝试使用世界北方 (Z 轴) 作为参考
        forward = Vector3.ProjectOnPlane(Vector3.forward, up);
        
        // 如果世界北方也和推力平行（例如推力指向正北且垂直？不太可能，除非推力水平）
        if (forward.sqrMagnitude < 0.001f)
        {
            // 再尝试使用世界东方 (X 轴)
            forward = Vector3.ProjectOnPlane(Vector3.right, up);
        }
        
        // 再次归一化
        if (forward.sqrMagnitude >= 0.001f)
            forward = forward.normalized;
        else
            forward = Vector3.right; // 最后的保底，防止 NaN
    }
    else
    {
        forward.Normalize();
    }
    
    // 3. 构建正交基
    Vector3 right = Vector3.Cross(up, forward).normalized;
    forward = Vector3.Cross(right, up).normalized;

        

        // 7. 从旋转矩阵构建四元数
        return  Quaternion.LookRotation(forward, up);
    }

    //由目标姿态和当前姿态和角速度计算需要的力矩
    public Vector3 CalculateTorque(
        Quaternion targetAttitude,
        Quaternion currentAttitude,
        Vector3 currentAngularRate,
        float dt,
        float yawinertia,
        float biasyaw)
    {
        // 1. 计算姿态误差
        attitudeError = CalculateAttitudeError(targetAttitude, currentAttitude);
        
        // 2. 姿态环PID：计算目标角速度
        targetAngularRate = CalculateTargetAngularRate(attitudeError, dt ,yawinertia ,biasyaw);
        
        // 3. 转换为当前角速度的单位,并加上偏置
        Vector3 currentAngularRateDeg = currentAngularRate * Mathf.Rad2Deg ;//+ new Vector3(0,biasyaw*yawinertia,0);
        
        // 4. 计算角速度误差
        rateError = (targetAngularRate - currentAngularRateDeg);
        
        // 5. 角速度环PID：
        torqueOutput = CalculateTorqueFromRateError(rateError, dt);

        // 6 .还原到机体坐标系
        Vector3 torqueBody = Quaternion.Inverse(dronePhysics.attitude)*torqueOutput ;

        
        return torqueBody+ new Vector3(0,biasyaw*yawinertia,0); //浏览
    }
    //计算姿态误差函数
    private Vector3 CalculateAttitudeError(Quaternion target, Quaternion current)
{
    
    Quaternion errorQuat = target * Quaternion.Inverse(current);
    
    // 2. 转换为轴角 (世界坐标系)
    errorQuat.ToAngleAxis(out float angle, out Vector3 axisWorld);
    
    // 3. 处理角度范围 [-180, 180]
    if (angle > 180f)
    {
        angle -= 360f;
        axisWorld = -axisWorld;
    }
    
   
    
    // 4. 计算机体坐标系下的误差向量
    
    Vector3 error = axisWorld * angle ;
    
    return error;
}
    // 姿态环：从姿态误差计算目标角速度
    private Vector3 CalculateTargetAngularRate(Vector3 attitudeError, float dt,float yawinertia,float biasyaw)
    {
        // PID控制得到目标角速度
        Vector3 targetRate = new Vector3(
            attitudecontrol.Update(attitudeError.x, dt),    // pitch
            attitudecontrol.Update(attitudeError.y, dt),    // Yaw
            attitudecontrol.Update(attitudeError.z, dt)     // roll
        );
        
        return targetRate ;
    }
    
    // 角速度环：从角速度误差计算控制力矩
    private Vector3 CalculateTorqueFromRateError(Vector3 rateError, float dt)
    {
        // PID控制得到力矩
        Vector3 torque = new Vector3(
            ratecontrol.Update(rateError.x, dt),    // Pitch力矩
            ratecontrol.Update(rateError.y, dt),    // Yaw力矩
            ratecontrol.Update(rateError.z, dt)     // roll力矩
        );
        
        return torque;
    }
    
    // PID控制容器的实现
    [System.Serializable]
    public struct PIDConfig
    {
    public float Kp;  // 比例增益
    public float Ki;  // 积分增益  
    public float Kd;  // 微分增益
    
    // 可选：限制参数
    public float integralLimit;  // 积分限幅
    public float outputLimit;    // 输出限幅
    
    // 构造函数
    public PIDConfig(float kp, float ki, float kd)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        integralLimit = 10f;
        outputLimit = 100f;
    }
    
    public PIDConfig(float kp, float ki, float kd, float intLimit, float outLimit)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        integralLimit = intLimit;
        outputLimit = outLimit;
    }
    }
    //PID控制器的实现
    [System.Serializable]
    public class PIDController
    {
    // 配置参数
    private PIDConfig config;
    
    // 状态变量
    private float integral = 0;
    private float lastError = 0;
    private float lastOutput = 0;
    
    // 属性
    public float Integral => integral;
    public float LastError => lastError;
    public float LastOutput => lastOutput;
    
    /// <summary>
    /// 构造函数
    /// </summary>
    public PIDController(PIDConfig config)
    {
        this.config = config;
    }
    
    /// <summary>
    /// 构造函数重载
    /// </summary>
    public PIDController(float kp, float ki, float kd)
    {
        this.config = new PIDConfig(kp, ki, kd);
    }
    
    /// <summary>
    /// 主控制函数
    /// </summary>
    public float Update(float error, float dt)
    {
        // 1. 积分项（抗饱和）
        float newIntegral = integral + error * dt;
        if (Mathf.Abs(newIntegral) <= config.integralLimit)
        {
            integral = newIntegral;
        }
        
        // 2. 微分项
        float derivative = 0;
        if (dt > Mathf.Epsilon)
        {
            derivative = (error - lastError) / dt;
        }
        
        // 3. PID输出
        float output = config.Kp * error + 
                      config.Ki * integral + 
                      config.Kd * derivative;
        
        // 4. 输出限幅
        output = Mathf.Clamp(output, -config.outputLimit, config.outputLimit);
        
        // 5. 更新状态
        lastError = error;
        lastOutput = output;
        
        return output;
    }
    
    /// <summary>
    /// 重置控制器状态
    /// </summary>
    public void Reset()
    {
        integral = 0;
        lastError = 0;
        lastOutput = 0;
    }
    
    /// <summary>
    /// 更新配置参数
    /// </summary>
    public void UpdateConfig(PIDConfig newConfig)
    {
        config = newConfig;
    }
    
    /// <summary>
    /// 获取当前配置
    /// </summary>
    public PIDConfig GetConfig()
    {
        return config;
    }
    
    /// <summary>
    /// 获取配置参数的字符串表示
    /// </summary>
    public override string ToString()
    {
        return $"PID Config: Kp={config.Kp:F2}, Ki={config.Ki:F2}, Kd={config.Kd:F2}";
    }
    }

    public void setflycontrolcommend(Vector4 commends)
    {
        acceleration[0] = commends[0];
        acceleration[1] = commends[1];
        acceleration[2] = commends[2];
        biasyaw = commends[3];
    }
       
}