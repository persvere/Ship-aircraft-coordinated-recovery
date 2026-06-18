using UnityEngine;
using UnityEngine.InputSystem.XR.Haptics;

public class CameraDepth : MonoBehaviour
{
        void OnEnable()
    {
        GetComponent<Camera>().depthTextureMode = DepthTextureMode.DepthNormals;
    }
}
