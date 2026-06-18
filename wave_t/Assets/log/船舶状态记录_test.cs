using System;
using System.IO;
using UnityEngine;

public class DataRecorder : MonoBehaviour
{
    [Header("录制设置")]
    public bool isRecording = true;       // 是否开启录制
    public float recordInterval = 0.1f;   // 录制间隔（秒），0.1f 表示 100ms 一次，即 10Hz
    public string fileName = "data_log";  // 文件名（不含扩展名）

    [Header("目标船")]
    public GameObject boat;       // 你想记录数据的物体（比如你的 UDP 控制物体）

    private string filePath;
    private float nextRecordTime = 0f;
    private object lockObj = new object(); // 线程锁

    void Start()
    {
        if (!isRecording) return;

        // 1. 设置文件路径：保存在 持久化数据路径 下
        // 这样在编辑器和打包后都能正常写入
        string directory = Path.Combine(Application.persistentDataPath, "Logs");
        if (!Directory.Exists(directory)) Directory.CreateDirectory(directory);

        filePath = Path.Combine(directory, $"{fileName}.csv");

        // 2. 写入 CSV 表头
        // 这里的列名对应你之后画图的 X轴、Y轴等
        string header = "RunTime,height,pitch,roll";
        
        // 如果文件已存在，我们选择覆盖（或者你可以改成追加）
        File.WriteAllText(filePath, header + Environment.NewLine);
        
        Debug.Log($"数据录制已启动 -> {filePath}");
    }

    void Update()
    {
        if (!isRecording) return;

        // 3. 定时检查
        if (Time.time >= nextRecordTime)
        {
            nextRecordTime = Time.time + recordInterval;
            RecordData();
        }
    }

    // 核心记录方法
    void RecordData()
    {
        // --- 数据采集区 ---
        
        // 1. 获取时间数据
        float runTime = Time.time;                             // 游戏运行时间 (X轴)
        float pitch;
        float roll;
        // 2. 获取物体数据 
        //获取船舶起伏数据
        float y = boat.transform.position.y ;
        //获取船舶角度数据
        Vector3 angles = boat.transform.eulerAngles;
        //俯仰角
        if (angles.x>300)
        {
        pitch = angles.x -360;
        }
        else
         pitch = angles.x;
        //横滚角
          if (angles.z>300)
        {
         roll = angles.z -360;
        }
        else
         roll = angles.z; 

        // 你可以在这里添加更多数据，比如速度、分数、UDP信号强度等
        // float speed = ...;
        // int score = ...;

        // 3. 格式化数据行 (CSV 格式)
        // 使用 invariant culture 防止小数点被转成逗号（欧洲系统常见坑）
        string dataLine = string.Format(System.Globalization.CultureInfo.InvariantCulture,
            "{0:F3},{1:F4},{2:F4},{3:F4}", 
             runTime, y, pitch, roll);

        // --- 写入文件区 ---
        WriteToFile(dataLine);
    }

    // 线程安全的写入方法
    void WriteToFile(string content)
    {
        lock (lockObj)
        {
            try
            {
                // 使用 AppendAllText 进行追加写入
                File.AppendAllText(filePath, content + Environment.NewLine);
            }
            catch (Exception e)
            {
                Debug.LogError("写入日志失败: " + e.Message);
            }
        }
    }
}