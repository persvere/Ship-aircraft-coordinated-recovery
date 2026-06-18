using UnityEngine;
using UnityEngine.UI;

public class SimpleCameraToTexture : MonoBehaviour
{
    public Camera targetCamera;      // 1. 找到画家
    public RawImage displayImage;    // 要显示图片的UI

    public int width;
    public int height;

    public RenderTexture canvas;
    
    void Start()
    {
        // 核心三行代码：
      
        targetCamera.targetTexture = canvas;                        // 3. 让画家在画布上画
        displayImage.texture = canvas;                             // 4. 把画布给别人看
    }
    void Update()
    {
        
    }
}