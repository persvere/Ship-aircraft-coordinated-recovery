using UnityEngine;

public class PID_control : MonoBehaviour
{
    [Header("组件引用")]
    public DroneFlightController flightController; // 拖入你的飞控脚本
    public Transform targetTransform;              // 目标点

    [Header("位置环 PID (输出: 目标速度)")]
    // 输入：米 (位置误差), 输出：米/秒 (速度指令)
    public float posKp = 1.0f;
    public float posKi = 0.0f;
    public float posKd = 0.0f;

    [Header("速度环 PID (输出: 目标加速度)")]
    // 输入：米/秒 (速度误差), 输出：米/秒² (加速度指令)
    public float velKp = 2.0f;
    public float velKi = 0.0f;
    public float velKd = 0.5f;

    [Header("偏航控制 (输出: 偏航调整强度)")]
    public float yawKp = 0.05f;

    [Header("限制参数")]
    public float maxHorizontalSpeed = 10f;   // 限制水平最大速度
    public float maxVerticalSpeed = 5f;      // 限制垂直最大速度
    public float maxTiltAccel = 15f;         // 限制最大加速度 (对应最大倾角)

    [Header("调试信息")]
    [SerializeField] private Vector3 positionError;
    [SerializeField] private Vector3 targetVelocity;
    [SerializeField] private Vector3 velocityError;
    [SerializeField] private Vector3 calculatedAcceleration;
    [SerializeField] private float yawOutput;

    // 内部状态变量
    // 位置环积分
    private Vector3 posIntegral = Vector3.zero;
    private Vector3 lastPositionError = Vector3.zero;
    
    // 速度环积分
    private Vector3 velIntegral = Vector3.zero;
    private Vector3 lastVelocityError = Vector3.zero;

    void Start()
    {
        if (flightController == null)
            flightController = GetComponent<DroneFlightController>();
    }

    void FixedUpdate()
    {
        if (targetTransform == null || flightController == null) return;

        ControlLoop();
    }

    void ControlLoop()
    {
        // --- 1. 获取状态 ---
        Vector3 currentPos = transform.position;
        Vector3 currentVel = flightController.dronePhysics.velocity; // 假设 physics 脚本里有 velocity 属性
        float currentYaw = transform.eulerAngles.y;

        // --- 2. 位置环 (Position Loop) ---
        // 计算位置误差 (世界坐标)
        positionError = targetTransform.position - currentPos;

        // 将位置误差转换到局部坐标系，方便解耦控制 (前/后, 左/右, 上/下)
        Vector3 localPosError = transform.InverseTransformDirection(positionError);

        // 位置环 P 控制 -> 得到目标速度
        Vector3 localTargetVel = Vector3.zero;
        localTargetVel.x = localPosError.x * posKp; // 左右
        localTargetVel.y = localPosError.y * posKp; // 上下
        localTargetVel.z = localPosError.z * posKp; // 前后

        // 限制最大速度
        Vector2 horizontalVel = new Vector2(localTargetVel.x, localTargetVel.z);
        if (horizontalVel.magnitude > maxHorizontalSpeed)
        {
            horizontalVel = horizontalVel.normalized * maxHorizontalSpeed;
            localTargetVel.x = horizontalVel.x;
            localTargetVel.z = horizontalVel.y;
        }
        localTargetVel.y = Mathf.Clamp(localTargetVel.y, -maxVerticalSpeed, maxVerticalSpeed);

        // 保存调试用
        targetVelocity = transform.TransformDirection(localTargetVel);

        // --- 3. 速度环 (Velocity Loop) ---
        // 获取当前速度 (局部坐标)
        Vector3 currentVelLocal = transform.InverseTransformDirection(currentVel);

        // 计算速度误差
        velocityError = localTargetVel - currentVelLocal;

        // 速度环 PID -> 得到目标加速度
        // P 项
        Vector3 localAccelCmd = velocityError * velKp;

        // D 项 (微分) - 这里的微分是对速度误差的微分，通常可以省略或仅用于阻尼
        // 也可以直接用 -currentVelLocal * damping 来模拟物理阻尼
        Vector3 damping = -currentVelLocal * velKd; 
        localAccelCmd += damping;

        // 限制最大加速度 (防止翻车)
        Vector2 horizontalAccel = new Vector2(localAccelCmd.x, localAccelCmd.z);
        if (horizontalAccel.magnitude > maxTiltAccel)
        {
            horizontalAccel = horizontalAccel.normalized * maxTiltAccel;
            localAccelCmd.x = horizontalAccel.x;
            localAccelCmd.z = horizontalAccel.y;
        }

        // 将加速度转回世界坐标
        calculatedAcceleration = transform.TransformDirection(localAccelCmd);

        // --- 4. 偏航控制 (Yaw Control) ---
        float targetYaw = targetTransform.eulerAngles.y;
        float yawErr = Mathf.DeltaAngle(currentYaw, targetYaw);
        
        // 计算偏航调整强度 (P控制)
        // 这个值会传给底层飞控的 biasyaw
        yawOutput = yawErr * yawKp;
        yawOutput = Mathf.Clamp(yawOutput, -3f, 3.0f); // 限制最大偏航力度

        // --- 5. 发送指令 ---
        Vector4 commands = new Vector4(
            localAccelCmd.x,
            localAccelCmd.y,
            localAccelCmd.z,
            yawOutput
        );

        flightController.setflycontrolcommend(commands);
    }
}