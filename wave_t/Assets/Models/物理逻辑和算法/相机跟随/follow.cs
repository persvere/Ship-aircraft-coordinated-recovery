using UnityEngine;

/// <summary>
/// 第三人称跟随脚本：相机保持在目标的局部后方和上方
/// </summary>
public class DroneCameraFollow : MonoBehaviour
{
    [Header("设置")]
    [Tooltip("要跟随的无人机物体")]
    public Transform target;

    [Header("相对偏移 (局部坐标)")]
    [Tooltip("X: 左右偏移, Y: 上下偏移 (正数为上), Z: 前后偏移 (负数为后)")]
    [Space(5)]
    public Vector3 localOffset = new Vector3(0f, 5f, -5f); 
    // 默认值：X=0 (正中), Y=5 (上方5米), Z=-5 (后方5米)

    [Tooltip("是否平滑跟随？(勾选后会有延迟感，更自然；不勾选则硬跟随)")]
    public bool smoothFollow = true;
    
    [Tooltip("平滑速度 (越大越快)")]
    [Range(1f, 20f)]
    public float smoothSpeed = 10f;

    void LateUpdate()
    {
        if (target == null) return;

        // --- 核心逻辑 ---
        
        // 1. 计算目标位置：将"局部偏移"转换为"世界坐标"
        // TransformPoint 会自动根据 target 的旋转和缩放，计算出正确的世界位置
        Vector3 desiredPosition = target.TransformPoint(localOffset);

        // 2. 应用位置
        if (smoothFollow)
        {
            // 使用插值平滑移动相机，避免画面抖动
            transform.position = Vector3.Lerp(transform.position, desiredPosition, Time.deltaTime * smoothSpeed);
        }
        else
        {
            // 硬跟随，瞬间到达
            transform.position = desiredPosition;
        }

        // 3. (可选) 让相机始终看着无人机，或者保持无人机的前向朝向
        // 方案 A: 相机死死盯着无人机中心 (最常用，保证无人机在屏幕内)
        transform.LookAt(target.position);
        
        // 方案 B: 如果希望相机完全复制无人机的旋转 (不推荐用于第三人称，容易晕)
        // transform.rotation = target.rotation; 
    }

    
}