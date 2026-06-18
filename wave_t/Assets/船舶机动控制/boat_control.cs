
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class boat_control : MonoBehaviour
{
    [Header("推进力")]
    [Tooltip("前向推力（牛顿）")]
    public float forwardForce = 5000f;

    [Header("转向力矩")]
    [Tooltip("偏航力矩（牛顿·米）")]
    public float yawTorque = 2000f;

    private Rigidbody rb;

    /*void Start()
    {
        rb = GetComponent<Rigidbody>();

        // 推荐的基础物理参数
        rb.useGravity = false;   // 船舶通常由浮力支撑
        rb.linearDamping = 1f;             // 水的线性阻力
        rb.angularDamping = 3f;      // 水的旋转阻力
    }*/

    void FixedUpdate()
    {
        // 1️⃣ 前向力（沿船体自身前方）
        Vector3 thrust = transform.forward * forwardForce;
        rb.AddForce(thrust, ForceMode.Force);

        // 2️⃣ 偏航力矩（绕船体上方）
        Vector3 torque = transform.up * yawTorque;
        rb.AddTorque(torque, ForceMode.Force);
    }
}