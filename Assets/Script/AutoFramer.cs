using UnityEngine;

[RequireComponent(typeof(Camera))]
public class AutoFramer : MonoBehaviour
{
    public SimulationManager sim;
    public Transform target;
    [Range(1f, 179f)]
    [Tooltip("Inspector에서 실시간 조정할 FOV (°}")]
    public float referenceFOV = 60f;
    public float minFOV = 1f, maxFOV = 179f;
    public Vector3 offsetBase = new Vector3(0f, 0f, -20f);

    private Camera cam;

    void Start()
    {
        cam = GetComponent<Camera>();
        if (cam == null) Debug.LogError("[AutoFramer] Camera 컴포넌트를 찾을 수 없습니다!");
        cam.orthographic = false;
        cam.nearClipPlane = 0.0001f;
        cam.farClipPlane = 1e9f;
        cam.eventMask = 0;
    }

    void LateUpdate()
    {
        if (sim == null || !sim.IsRunning || target == null) return;

        cam.fieldOfView = referenceFOV; // 클램핑 간소화
        Vector3 scale = target.localScale;
        // 요소별 곱셈으로 오프셋 계산
        Vector3 offset = new Vector3(
            offsetBase.x * scale.x,
            offsetBase.y * scale.y,
            offsetBase.z * scale.z
        );
        cam.transform.position = target.position + offset;
        cam.transform.rotation = Quaternion.Euler(0f, 0f, 0f);
    }
}