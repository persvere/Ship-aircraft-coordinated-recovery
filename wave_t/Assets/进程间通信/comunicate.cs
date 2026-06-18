using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.InputSystem; 
public class UnityUdpClient : MonoBehaviour
{
    private UdpClient udpClient;
    private IPEndPoint remoteEndPoint; // Python 的地址
    private Thread receiveThread;      // 接收数据的线程

    // 配置项
    public string pythonIP = "127.0.0.1";
    public int pythonPort = 5005;      // 对应 Python 端的端口
    public int localPort = 5006;       // Unity 本地监听端口 (任意未占用端口即可)

    void Start()
    {
        Init();
    }

    void Init()
    {
        // 1. 设置 Python 的目标地址
        remoteEndPoint = new IPEndPoint(IPAddress.Parse(pythonIP), pythonPort);

        // 2. 创建客户端并绑定本地端口 (用于接收 Python 的回复)
        udpClient = new UdpClient(localPort);

        // 3. 开启线程用于接收数据
        receiveThread = new Thread(new ThreadStart(ReceiveData));
        receiveThread.IsBackground = true; // 设置为后台线程，程序关闭时自动结束
        receiveThread.Start();

        Debug.Log($"🎮 Unity 客户端已启动，监听端口: {localPort}");
    }

    void Update()
    {
        // 按下空格键发送消息
        var keyboard = Keyboard.current;
        if (keyboard.spaceKey.isPressed)
        {
            SendMessage("Hello Python!");
        }
        
        // 按下 Q 键发送退出指令
        if (keyboard.qKey.isPressed)
        {
            SendMessage("exit");
        }
    }

    // 发送数据的方法
    public void SendMessage(string message)
    {
        try
        {
            byte[] data = Encoding.UTF8.GetBytes(message);
            udpClient.Send(data, data.Length, remoteEndPoint);
            Debug.Log($"📤 发送: {message}");
        }
        catch (Exception err)
        {
            Debug.LogError($"发送失败: {err}");
        }
    }

    // 接收数据的线程方法
    private void ReceiveData()
    {
        while (true)
        {
            try
            {
                // 这里的 anyIP 用于接收来自任意 IP 的数据
                IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, 0);
                
                // 阻塞等待数据
                byte[] data = udpClient.Receive(ref anyIP);
                string message = Encoding.UTF8.GetString(data);

                // ⚠️ 注意：这里是在子线程中，不能直接操作 UI 或 Debug.Log
                // 如果要更新 UI，需要使用 MainThreadDispatcher 或协程
                Debug.Log($"📥 收到 Python 回复: {message}");
            }
            catch (Exception err)
            {
                Debug.LogError(err);
            }
        }
    }

    // 程序退出时关闭连接
    void OnDestroy()
    {
        if (udpClient != null) udpClient.Close();
        if (receiveThread != null && receiveThread.IsAlive) receiveThread.Abort();
    }
}