using UnityEngine;
using UnityEngine.Rendering;
using static Unity.Mathematics.math;
using Unity.Collections;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;

public class FFTOcean_Buoyancy : MonoBehaviour
{
    public FFTOcean_Script waterSource;
    public int voxelsPerAxisX = 2;
    public int voxelsPerAxisY = 2;
    public int voxelsPerAxisZ = 2;

    [Range(0.1f, 20.0f)]
    public float linearDragIntensity;
    [Range(0.1f, 20.0f)]
    public float angularDragIntensity;

    private Rigidbody rigidBody;
    private Collider cachedCollider;

    private Voxel[,,] voxels;
    private List<Vector3> receiverVoxels;
    private Vector3 voxelSize;

    private Quaternion targetRotation;

    List<Vector3> voxelPoint;
    Queue<Vector3> cachedDirections;

    public struct Voxel
    {
        public Vector3 position { get; }
        private float cachedBuoyancyData;
        private FFTOcean_Script waterSource;
        private AsyncGPUReadbackRequest voxelWaterRequest;

        public Voxel(Vector3 position, FFTOcean_Script waterSource)
        {
            this.position = position;
            this.cachedBuoyancyData = 0.0f;
            this.waterSource = waterSource;

            voxelWaterRequest = AsyncGPUReadback.Request(waterSource.GetBuoyancyData(), 0, 0, 1, 0, 1, 0, 1, null);
        }

        public float GetWaterHeight()
        {
            return cachedBuoyancyData;
        }

        public void Update(Transform parentTransform)
        {
            if (voxelWaterRequest.done)
            {
                if (voxelWaterRequest.hasError)
                {
                    Debug.Log("Buoyancy Request Error");
                    return;
                }

                NativeArray<ushort> buoyancyDataQuery = voxelWaterRequest.GetData<ushort>();

                cachedBuoyancyData = Mathf.HalfToFloat(buoyancyDataQuery[0]);

                Vector3 worldPos = parentTransform.TransformPoint(this.position);
                Vector2 pos = new Vector2(worldPos.x, worldPos.z);
                Vector2 uv = pos * waterSource.Tile0;

                int x = Mathf.FloorToInt(frac(uv.x) * 1023);
                int y = Mathf.FloorToInt(frac(uv.y) * 1023);

                voxelWaterRequest = AsyncGPUReadback.Request(waterSource.GetBuoyancyData(), 0, x, 1, y, 1, 0, 1, null);
            }
        }
    }
        private void CreateVoxels()
    {
        receiverVoxels = new List<Vector3>();

        // 1. 获取本地坐标系下的大小
        // 我们不再使用 cachedCollider.bounds (世界坐标)，而是直接获取 Collider 的本地尺寸
        // 假设这是一个 BoxCollider，我们可以直接读取 size
        BoxCollider boxCollider = cachedCollider as BoxCollider;
        
        // 推荐做法：直接使用 BoxCollider 的 size 属性（这是本地坐标）
        Vector3 localSize = Vector3.zero;
        Vector3 localCenter = Vector3.zero;

        if (boxCollider != null)
        {
            localSize = boxCollider.size;
            localCenter = boxCollider.center;
        }
        else
        {
            // 如果是其他 Collider，使用 bounds.extents 作为近似，但要注意旋转影响
            // 这里为了代码能跑，暂时使用 bounds.extents，但建议最好用 BoxCollider
            localSize = cachedCollider.bounds.size; 
            localCenter = cachedCollider.bounds.center - transform.position; // 近似本地中心
        }

        // 2. 计算每个格子的大小 (基于本地尺寸)
        voxelSize = new Vector3(
            localSize.x / voxelsPerAxisX,
            localSize.y / voxelsPerAxisY,
            localSize.z / voxelsPerAxisZ
        );

        voxels = new Voxel[voxelsPerAxisX, voxelsPerAxisY, voxelsPerAxisZ];

        // 3. 在本地坐标系中生成 Voxel
        // 我们以本地中心为基准，向四周扩展
        for (int x = 0; x < voxelsPerAxisX; ++x)
        {
            for (int y = 0; y < voxelsPerAxisY; ++y)
            {
                for (int z = 0; z < voxelsPerAxisZ; ++z)
                {
                    // 计算本地坐标中的位置
                    // 公式：中心 + (索引偏移 - 总范围的一半)
                    Vector3 localPos = localCenter + new Vector3(
                        (x + 0.5f) * voxelSize.x - localSize.x * 0.5f,
                        (y + 0.5f) * voxelSize.y - localSize.y * 0.5f,
                        (z + 0.5f) * voxelSize.z - localSize.z * 0.5f
                    );

                    // 存入 Voxel (存的是本地坐标)
                    voxels[x, y, z] = new Voxel(localPos, waterSource);
                    receiverVoxels.Add(new Vector3(x, y, z));
                }
            }
        }
    }

    private void OnEnable()
    {
        if (waterSource == null) return;
        rigidBody = GetComponent<Rigidbody>();
        cachedCollider = GetComponent<Collider>();
        targetRotation = Quaternion.identity;
        cachedDirections = new Queue<Vector3>();
    }

    void Update()
    {
        if (voxels == null && waterSource.GetBuoyancyData() != null) CreateVoxels();
        for (int i = 0; i < receiverVoxels.Count; ++i) {
            voxels[ (int)receiverVoxels[i].x,
                    (int)receiverVoxels[i].y,
                    (int)receiverVoxels[i].z].Update(this.transform);
        }
    }

    void FixedUpdate()
    {
        float boundsVolume = cachedCollider.bounds.size.x * cachedCollider.bounds.size.y * cachedCollider.bounds.size.z;
        float density = rigidBody.mass / boundsVolume;

        float submergedVolume = 0.0f;
        float voxelHeight = voxelSize.y;
        float UnitForce = (1.0f - density) / voxels.Length;

        for (int x = 0; x < voxelsPerAxisX; ++x)
        {
            for (int y = 0; y < voxelsPerAxisY; ++y)
            {
                for (int z = 0; z < voxelsPerAxisZ; ++z)
                {
                    Vector3 worldPos = this.transform.TransformPoint(voxels[x, y, z].position);

                    float waterLevel = voxels[x, y, z].GetWaterHeight();
                    float depth = waterLevel - worldPos.y + voxelHeight;
                    float submergedFactor = Mathf.Clamp(depth / voxelHeight, 0, 1);
                    submergedVolume += submergedFactor;

                    float displacement = Mathf.Max(0.0f, depth);

                    Vector3 force = -Physics.gravity * displacement * UnitForce;
                    rigidBody.AddForceAtPosition(force, worldPos);
                }
            }
        }

        submergedVolume /= voxels.Length;

        this.rigidBody.drag = Mathf.Lerp(2.0f, linearDragIntensity, submergedVolume);
        this.rigidBody.angularDrag = Mathf.Lerp(2.0f, angularDragIntensity, submergedVolume);
    }

    void OnDisable()
    {
        voxels = null;
    }

    private void OnDrawGizmos()
    {
        if (this.voxels != null)
        {
            //            Fit.Plane(points, out origin, out direction, 50, true);

            for (int x = 0; x < voxelsPerAxisX; ++x)
            {
                for (int y = 0; y < voxelsPerAxisY; ++y)
                {
                    for (int z = 0; z < voxelsPerAxisZ; ++z)
                    {
                        Gizmos.color = Color.green;
                        Gizmos.DrawCube(this.transform.TransformPoint(this.voxels[x, y, z].position), this.voxelSize * 0.8f);
                    }
                }
            }
        }
    }
}
