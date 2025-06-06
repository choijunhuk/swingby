using UnityEngine;

[RequireComponent(typeof(Camera))]
public class AutoFramer : MonoBehaviour
{
    [Header("Simulation Manager")]
    public SimulationManager sim;

    [Header("Targets")]
    public Transform target;    // 우주선(ShipTransform)을 할당

    [Header("FOV Settings")]
    [Range(1f, 179f)]
    [Tooltip("Inspector에서 실시간 조정할 FOV (°)")]
    public float referenceFOV = 60f;
    public float minFOV = 1f;
    public float maxFOV = 179f;

    private Camera cam;

    void Awake()
    {
        cam = GetComponent<Camera>();
        if (cam == null)
            Debug.LogError("[AutoFramer] Camera 컴포넌트를 찾을 수 없습니다!");

        // 기본적으로 비활성화해 두고, UIManager가 켜줌
        enabled = false;

        cam.orthographic  = false;
        cam.nearClipPlane = 0.0001f;
        cam.farClipPlane  = 1e12f;
        cam.eventMask     = 0;
    }

    void LateUpdate()
    {
        // 단순히 FOV만 제어 (위치/회전은 UIManager에서 처리)
        cam.fieldOfView = Mathf.Clamp(referenceFOV, minFOV, maxFOV);
    }
}
