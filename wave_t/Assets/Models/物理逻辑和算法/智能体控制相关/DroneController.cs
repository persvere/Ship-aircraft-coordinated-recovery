using UnityEngine;

// ========== 3. 主控制器 ==========
public class DroneController : MonoBehaviour
{
    [Header("Components")]
    public DronePhysics dronePhysics;
    //public DroneAgent droneAgent;
    public DroneFlightController droneFlightController;
    
    [Header("Update Settings")]
    public bool useFixedUpdate = true;
    public float physicsUpdateRate = 50f;  // 50Hz

    [Header("是否使用飞控")]
    public bool if_fly = true;
    
    private float physicsDeltaTime;
    
    void Start()
    {
        physicsDeltaTime = 1f / physicsUpdateRate;
        
        if (dronePhysics == null)
            dronePhysics = GetComponent<DronePhysics>();
        
        //if (droneAgent == null)
            //droneAgent = GetComponent<DroneAgent>();
        if (droneFlightController==null)
            droneFlightController = GetComponent<DroneFlightController>();
    }
    
    // 选项1：使用FixedUpdate
    void FixedUpdate()
    {
        if (useFixedUpdate)
        {
            if (if_fly)// 物理更新
            {droneFlightController.pidflyconrol();}
            dronePhysics.PhysicsUpdate(Time.fixedDeltaTime);
        }
    }
    
    // 选项2：自定义更新频率
    void Update()
    {
       
    }
    
    private void UpdateVisualEffects()
    {
        // 更新螺旋桨动画、粒子效果等
        // 这些不需要物理精度
    }
}