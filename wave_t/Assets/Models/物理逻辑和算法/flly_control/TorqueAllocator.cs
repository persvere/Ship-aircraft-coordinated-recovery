using UnityEngine;
using System.Collections.Generic;

public class TorqueAllocator : MonoBehaviour
{
    [Header("组件引用")]
    public DronePhysics dronePhysics;
    
    [Header("转子配置")]
    public float scale = 10;
    private Vector3[] rotorPositions = new Vector3[4];
    private int[] rotorDirections = new int[4];
    private float kTau;
    private float maxThrustPerMotor;
    
    // --- 预计算数据 (不再每次重复计算) ---
    private Matrix4x4 allocationMatrix; // A (4x4)
    private float[,] AtA_Full;          // A^T * A (4x4)
    private float[,] At_Full;           // A^T (4x4)
    
    // --- 运行时复用缓冲区 (避免 GC) ---
    private float[,] M_Work;            // 工作矩阵 (用于高斯消元)
    private float[] V_Work;             // 工作向量
    private float[] X_Work;             // 解向量
    private int[] freeIndices;          // 自由变量索引缓存
    private Vector4 lockedContribution; // 已锁定电机的贡献总和
    
    [Header("调试信息")]
    [SerializeField] private Vector4 lastThrustAllocation;
    [SerializeField] private int lastIterationCount;
    
    void Awake()
    {
        Initialize();
    }
    
    private void Initialize()
    {
        if (dronePhysics == null) dronePhysics = GetComponent<DronePhysics>();
        if (dronePhysics == null) { Debug.LogError("Missing DronePhysics"); return; }
        
        // 1. 初始化配置
        for (int i = 0; i < 4; i++) rotorPositions[i] = dronePhysics.rotorLocalPositions[i];
        rotorDirections = new int[4] { 1, -1, 1, -1 }; 
        kTau = dronePhysics.rotorRadius * dronePhysics.Anti_torquecoefficient;
        maxThrustPerMotor = dronePhysics.maxThrust;
        
        // 2. 构建分配矩阵 A (只构建一次)
        allocationMatrix = BuildAllocationMatrix(rotorPositions, rotorDirections, kTau);
        
        // 3. 预计算 A^T 和 A^T * A (只计算一次)
        At_Full = new float[4, 4];
        AtA_Full = new float[4, 4];
        
        // 填充 At_Full
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++) {
                At_Full[c, r] = GetMatrixElement(allocationMatrix, r, c); // 转置
            }
        }
        
        // 填充 AtA_Full = At * A
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                float sum = 0f;
                for (int k = 0; k < 4; k++) {
                    sum += At_Full[i, k] * GetMatrixElement(allocationMatrix, k, j);
                }
                AtA_Full[i, j] = sum;
            }
        }
        
        // 4. 初始化运行时缓冲区 (只在启动时 new 一次)
        M_Work = new float[4, 4];
        V_Work = new float[4];
        X_Work = new float[4];
        freeIndices = new int[4];
        lockedContribution = Vector4.zero;
        
        Debug.Log("扭矩分配器初始化完成 (高性能零GC模式)");
        // 在 TorqueAllocator.Initialize 或 BuildAllocationMatrix 中
Debug.Log("=== Allocation Matrix Debug ===");
for(int i=0; i<4; i++) {
    // 打印每一列：[Roll_Coeff, Yaw_Coeff, Pitch_Coeff, Thrust_Coeff]
    Vector4 col = new Vector4(-rotorPositions[i].z, -rotorDirections[i] * kTau, rotorPositions[i].x, 1.0f);
    Debug.Log($"Motor {i}: Pos={rotorPositions[i]}, Dir={rotorDirections[i]}, YawTerm={-rotorDirections[i] * kTau}, FullCol={col}");
}
    }
    
    // 主接口
    public Vector4 GetAllocatedThrust(Vector3 torque, float thrust)
    {
        Vector4 desiredWrench = new Vector4(torque.x, torque.y, torque.z, thrust);
        Vector4 result = SolveConstrainedAllocationFast(desiredWrench);
        lastThrustAllocation = result;
        return result;
    }

    /// <summary>
    /// 高性能迭代求解器 (无 GC 分配)
    /// </summary>
    private Vector4 SolveConstrainedAllocationFast(Vector4 desiredWrench)
    {
        Vector4 result = Vector4.zero;
        // status: 0=自由, -1=锁定0, 1=锁定Max
        int[] status = new int[4] { 0, 0, 0, 0 }; 
        int freeCount = 4;
        
        int maxIter = 6; // 4个电机最多迭代几次
        int iter = 0;
        
        while (iter < maxIter)
        {
            iter++;
            freeCount = 0;
            lockedContribution = Vector4.zero;
            
            // 1. 准备阶段：分离自由变量和锁定变量
            for (int i = 0; i < 4; i++)
            {
                if (status[i] == -1)
                {
                    result[i] = 0f;
                    // 贡献为0
                }
                else if (status[i] == 1)
                {
                    result[i] = maxThrustPerMotor;
                    // 累加锁定部分的贡献: A_col * Max
                    lockedContribution += GetColumn(allocationMatrix, i) * maxThrustPerMotor;
                }
                else
                {
                    freeIndices[freeCount++] = i;
                }
            }
            
            if (freeCount == 0) break; // 全部锁定
            
            // 2. 构建缩减后的线性方程组 (AtA * x = At * b)
            // 目标向量修正: b' = b - lockedContribution
            Vector4 correctedTarget = desiredWrench - lockedContribution;
            
            // 我们需要解: (A_free^T * A_free) * x_free = A_free^T * correctedTarget
            // 直接从预计算的 AtA_Full 和 At_Full 中提取子矩阵，无需重新乘法！
            
            // 填充工作矩阵 M (freeCount x freeCount)
            for (int i = 0; i < freeCount; i++)
            {
                int rowOrig = freeIndices[i];
                for (int j = 0; j < freeCount; j++)
                {
                    int colOrig = freeIndices[j];
                    // 直接从预计算的 AtA 中取值
                    M_Work[i, j] = AtA_Full[rowOrig, colOrig];
                }
                
                // 填充工作向量 V (freeCount)
                // V[i] = (A_free^T * correctedTarget)[i] = Sum(At[rowOrig, k] * target[k])
                float sumV = 0f;
                for (int k = 0; k < 4; k++)
                {
                    sumV += At_Full[rowOrig, k] * correctedTarget[k];
                }
                V_Work[i] = sumV;
                
                // 正则化 (防止奇异，加在对角线上)
                M_Work[i, i] += 1e-6f;
            }
            
            // 3. 求解线性方程组 (原地求解，结果存入 X_Work 前 freeCount 个元素)
            GaussianEliminationInPlace(M_Work, V_Work, X_Work, freeCount);
            
            // 4. 映射回结果并检查违规
            bool violationFound = false;
            
            for (int i = 0; i < freeCount; i++)
            {
                int origIdx = freeIndices[i];
                float val = X_Work[i];
                result[origIdx] = val;
                
                // 检查下界
                if (val < -0.005f) // 稍微放宽容差
                {
                    status[origIdx] = -1; // 锁定为0
                    violationFound = true;
                    break; // 一次处理一个，保证稳定性
                }
                
                // 检查上界
                if (val > maxThrustPerMotor + 0.005f)
                {
                    status[origIdx] = 1; // 锁定为Max
                    violationFound = true;
                    break;
                }
            }
            
            if (!violationFound) break; // 成功收敛
        }
        
        lastIterationCount = iter;
        return result;
    }
    
    /// <summary>
    /// 原地高斯消元 (不返回新数组，直接写入 solution 数组)
    /// </summary>
    private void GaussianEliminationInPlace(float[,] M, float[] v, float[] solution, int n)
    {
        // 注意：这里会修改 M 和 v 的内容，因为它们每轮都是重新填充的，所以没问题
        
        for (int i = 0; i < n; i++)
        {
            // 选主元
            int maxRow = i;
            for (int k = i + 1; k < n; k++)
            {
                if (Mathf.Abs(M[k, i]) > Mathf.Abs(M[maxRow, i]))
                    maxRow = k;
            }
            
            // 交换行
            if (maxRow != i)
            {
                for (int k = i; k < n; k++)
                {
                    float tmp = M[i, k]; M[i, k] = M[maxRow, k]; M[maxRow, k] = tmp;
                }
                float tmpV = v[i]; v[i] = v[maxRow]; v[maxRow] = tmpV;
            }
            
            if (Mathf.Abs(M[i, i]) < 1e-8f) continue;
            
            // 消元
            for (int k = i + 1; k < n; k++)
            {
                float factor = M[k, i] / M[i, i];
                for (int j = i; j < n; j++)
                {
                    M[k, j] -= factor * M[i, j];
                }
                v[k] -= factor * v[i];
            }
        }
        
        // 回代
        for (int i = n - 1; i >= 0; i--)
        {
            float sum = 0f;
            for (int j = i + 1; j < n; j++)
            {
                sum += M[i, j] * solution[j];
            }
            solution[i] = (Mathf.Abs(M[i, i]) > 1e-8f) ? ((v[i] - sum) / M[i, i]) : 0f;
        }
    }
    
    // 辅助：安全获取 Matrix4x4 元素
    private float GetMatrixElement(Matrix4x4 m, int row, int col)
    {
        if (col == 0) return (row == 0) ? m.m00 : (row == 1) ? m.m10 : (row == 2) ? m.m20 : m.m30;
        if (col == 1) return (row == 0) ? m.m01 : (row == 1) ? m.m11 : (row == 2) ? m.m21 : m.m31;
        if (col == 2) return (row == 0) ? m.m02 : (row == 1) ? m.m12 : (row == 2) ? m.m22 : m.m32;
        if (col == 3) return (row == 0) ? m.m03 : (row == 1) ? m.m13 : (row == 2) ? m.m23 : m.m33;
        return 0;
    }
    
    // 辅助：获取列向量
    private Vector4 GetColumn(Matrix4x4 m, int col)
    {
        if (col == 0) return new Vector4(m.m00, m.m10, m.m20, m.m30);
        if (col == 1) return new Vector4(m.m01, m.m11, m.m21, m.m31);
        if (col == 2) return new Vector4(m.m02, m.m12, m.m22, m.m32);
        if (col == 3) return new Vector4(m.m03, m.m13, m.m23, m.m33);
        return Vector4.zero;
    }
    
    private Matrix4x4 BuildAllocationMatrix(Vector3[] positions, int[] dirs, float k_tau)
    {
        // 保持与你原逻辑一致
        Vector4 c0 = new Vector4(-positions[0].z, -dirs[0] * k_tau * scale, positions[0].x, 1.0f);
        Vector4 c1 = new Vector4(-positions[1].z, -dirs[1] * k_tau * scale, positions[1].x, 1.0f);
        Vector4 c2 = new Vector4(-positions[2].z, -dirs[2] * k_tau * scale, positions[2].x, 1.0f);
        Vector4 c3 = new Vector4(-positions[3].z, -dirs[3] * k_tau * scale, positions[3].x, 1.0f);
        return new Matrix4x4(c0, c1, c2, c3);
    }
}