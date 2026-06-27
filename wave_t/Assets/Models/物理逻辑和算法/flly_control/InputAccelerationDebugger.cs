using UnityEngine;
using UnityEngine.InputSystem; // 【新增】引入新输入系统

/// <summary>
/// 键盘调试器：通过按键动态修改 DroneFlightController 的 acceleration 目标值
/// 用于测试 PID 对不同加速度指令的响应
/// </summary>
public class InputAccelerationDebugger : MonoBehaviour
{
    [Header("组件引用")]
    public DroneFlightController flightController;
    
    [Header("调试设置")]
    [Tooltip("按下按键时设定的加速度 magnitude")]
    public float accelerationMagnitude = 2.0f;
    [Tooltip("偏航时的旋转角度")]
    public float targetYaw = 30.0f;
    
    [Tooltip("是否启用此调试器")]
    public bool enableDebugInput = true;

    // 当前计算出的目标加速度
    public Vector3 targetAcceleration;
    public float yawstrength;


    void Start()
    {
        // 自动获取组件，如果Inspector没填的话
        if (flightController == null)
        {
            flightController = GetComponent<DroneFlightController>();
            if (flightController == null)
                flightController = FindObjectOfType<DroneFlightController>();
        }

        if (flightController == null)
        {
            Debug.LogError("[InputDebugger] 未找到 DroneFlightController 组件！");
            enabled = false; // 禁用脚本
        }
        else
        {
            Debug.Log("[InputDebugger] 已就绪。使用 WASD + Space/Shift 控制加速度目标值。");
        }
    }

    void Update()
    {
        if (!enableDebugInput || flightController == null) return;

       

        // --- 1. 检测输入 (假设 Unity 标准坐标系: X=前, Y=上, Z=右) ---
        // 如果你的无人机坐标系不同，请调整对应的轴
         var keyboard = Keyboard.current;
        // 前后 (X轴)
        if (keyboard.wKey.isPressed)
            targetAcceleration.z = accelerationMagnitude;
        if (keyboard.sKey.isPressed)
            targetAcceleration.z = -accelerationMagnitude;
        if (!keyboard.wKey.isPressed && !keyboard.sKey.isPressed)
            targetAcceleration.z = 0;
        
            
    

            

        // 左右 (Z轴 - Unity 中通常 Z 是右，如果是 Y 轴请自行修改)
        // 注意：很多无人机项目习惯用 Z 做前后，Y 做上下，X 做左右。
        // 请根据你 DroneFlightController 内部的实际定义调整这里！
        if (keyboard.dKey.isPressed)
            targetAcceleration.x = accelerationMagnitude; // 向右
        if (keyboard.aKey.isPressed)
            targetAcceleration.x = -accelerationMagnitude; // 向左
        if (!keyboard.aKey.isPressed && !keyboard.dKey.isPressed)
            targetAcceleration.x = 0;

        // 上下 (Y轴)
        if (keyboard.jKey.isPressed)
            targetAcceleration.y = accelerationMagnitude; // 向上
        if (keyboard.kKey.isPressed)
            targetAcceleration.y = -accelerationMagnitude; // 向下
        if (!keyboard.jKey.isPressed && !keyboard.kKey.isPressed)
            targetAcceleration.y = 0;

        //偏航
        if (keyboard.uKey.isPressed)
            yawstrength = targetYaw;
        if (keyboard.iKey.isPressed)
            yawstrength = -targetYaw;
        if (!keyboard.uKey.isPressed && !keyboard.iKey.isPressed)
            yawstrength = 0;

        // --- 2. 应用加速度到飞控 ---
        // 直接覆盖 acceleration 字段
        
        flightController.acceleration = Vector3.Lerp(
        flightController.acceleration, 
        targetAcceleration, 
        Time.deltaTime * 50f );
        flightController.biasyaw = Mathf.Lerp(flightController.biasyaw,yawstrength,Time.deltaTime * 50f );


        // --- 3. (可选) 简单的 UI 显示 ---
        // 如果想在 Game 视图看到当前值，可以取消下面这行的注释
        // Debug.DrawText(transform.position, $"Accel: {targetAcceleration}", Color.cyan); 
    }
    
    // 在编辑器中绘制一些提示信息
    void OnDrawGizmosSelected()
    {
        if (!enableDebugInput) return;
        
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(transform.position, transform.position + transform.forward * targetAcceleration.x);
        Gizmos.DrawLine(transform.position, transform.position + transform.up * targetAcceleration.y);
        Gizmos.DrawLine(transform.position, transform.position + transform.right * targetAcceleration.z);
    }
}